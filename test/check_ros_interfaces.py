#!/usr/bin/env python3
"""Check one running MuJoCo robot; --commands also checks its base safety path."""
import argparse
import time
import xml.etree.ElementTree as ET

import rclpy
from builtin_interfaces.msg import Time
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--commands", action="store_true",
                        help="Move the simulated base with Nav2 disabled")
    options = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("menagerie_interface_check")
    descriptions, states, transforms, clock, odometry = {}, {}, {}, [], []
    durable = QoSProfile(
        depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def wait(predicate, timeout=20):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError("Timed out waiting for ROS interfaces")

    def remember_tf(message, kind):
        for tf in message.transforms:
            transforms[kind, tf.child_frame_id] = tf.header.frame_id

    def controllers():
        future = client.call_async(ListControllers.Request())
        wait(future.done, 3)
        return future.result().controller

    try:
        client = node.create_client(
            ListControllers, "/controller_manager/list_controllers")
        assert client.wait_for_service(
            timeout_sec=10), "Launch a MuJoCo robot first"
        active = controllers()
        assert active and all(c.state == "active" for c in active)
        parts = ["body", *[f"hand_{side}" for side in ("left", "right")
                           if any(c.name == f"hand_{side}_joint_state_broadcaster" for c in active)]]
        for topic in ["/robot_description", *[f"/control/{p}/robot_description" for p in parts]]:
            node.create_subscription(
                String, topic, lambda msg, t=topic: descriptions.__setitem__(t, msg.data), durable)
        for part in parts:
            node.create_subscription(JointState, f"/sensors/proprio/{part}/joint_states",
                                     lambda msg, p=part: states.__setitem__(p, msg), 10)
        node.create_subscription(
            Clock, "/clock", lambda msg: clock.append(msg.clock.sec + msg.clock.nanosec / 1e9), 10)
        for topic, kind, qos in [("/tf", "dynamic", 100), ("/tf_static", "static", durable)]:
            node.create_subscription(
                TFMessage, topic, lambda msg, k=kind: remember_tf(msg, k), qos)
        wait(lambda: len(descriptions) == len(parts) +
             1 and len(states) == len(parts) and clock and transforms)
        full = ET.fromstring(descriptions["/robot_description"])
        assert full.findtext(
            "ros2_control/hardware/plugin") == "mujoco_ros2_control/MujocoSystemInterface"
        joints, measured = [], []
        for part in parts:
            robot = ET.fromstring(
                descriptions[f"/control/{part}/robot_description"])
            assert robot.find("ros2_control") is None
            names = {j.get("name") for j in robot.findall("joint")}
            joints.extend(names)
            measured.extend(states[part].name)
            assert set(states[part].name) <= names
            stamp = states[part].header.stamp.sec + \
                states[part].header.stamp.nanosec / 1e9
            assert 0 < stamp <= clock[-1] + 0.25 and clock[-1] - \
                stamp < 0.5, "Wrong clock domain"
        assert len(joints) == len(set(joints)) and set(joints) == {
            j.get("name") for j in full.findall("joint")}
        assert len(measured) == len(set(measured)) and set(measured) == {
            j.get("name") for j in full.findall("ros2_control/joint")}
        topics = dict(node.get_topic_names_and_types())
        assert all(
            f"/sensors/proprio/{p}/dynamic_joint_states" in topics for p in parts)
        assert any(i.node_name == "controller_manager" for i in node.get_subscriptions_info_by_topic(
            "/robot_description"))
        for c in active:
            if "JointTrajectoryController" in c.type:
                part = c.name.removesuffix(
                    "_controller") if c.name.startswith("hand_") else "body"
                prefix = f"/control/{part}/{c.name}"
                assert all(prefix + suffix in topics for suffix in ("/joint_trajectory",
                           "/controller_state", "/follow_joint_trajectory/_action/status"))
        root, = {link.get("name") for link in full.findall(
            "link")} - {j.find("child").get("link") for j in full.findall("joint")}
        if any(c.name == "base_pose_broadcaster" for c in active):
            wait(lambda: transforms.get(("dynamic", root)) == "map")
            assert ("static", root) not in transforms
        elif "/cmd_vel" in topics:
            wait(lambda: transforms.get(("dynamic", root)) == "odom"
                 and transforms.get(("static", "odom")) == "map")
        else:
            wait(lambda: transforms.get(("static", root)) == "map")
        if "/cmd_vel" in topics:
            assert topics["/cmd_vel"] == ["geometry_msgs/msg/TwistStamped"]
        if options.commands:
            assert "/cmd_vel" in topics, "Robot has no mobile base"
            publisher = node.create_publisher(TwistStamped, "/cmd_vel", 10)
            node.create_subscription(Odometry, "/odom", odometry.append, 10)
            wait(lambda: publisher.get_subscription_count() and odometry)
            assert publisher.get_subscription_count() == 1 and node.count_publishers(
                "/cmd_vel") == 1, "Disable Nav2 and other command sources"
            end, moved = time.monotonic() + 1, False
            while time.monotonic() < end:
                msg = TwistStamped()
                msg.header.stamp = Time(
                    sec=int(clock[-1]), nanosec=int((clock[-1] % 1) * 1e9))
                msg.twist.linear.x = 0.08
                publisher.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.05)
                moved |= abs(odometry[-1].twist.twist.linear.x) > 0.02
            assert moved, "Base never moved"
            wait(lambda: abs(odometry[-1].twist.twist.linear.x) < 0.01, 4)
            if any("SwerveDriveController" in c.type for c in active):
                for stamp, value in [(Time(sec=-1), 0.08), (Time(sec=1, nanosec=1000000000), 0.08),
                                     (Time(
                                         sec=max(0, int(clock[-1]) - 2)), 0.08),
                                     (None, float("nan"))]:
                    if stamp is None:
                        stamp = Time(
                            sec=int(clock[-1]), nanosec=int((clock[-1] % 1) * 1e9))
                    msg = TwistStamped()
                    msg.header.stamp, msg.twist.linear.x = stamp, value
                    publisher.publish(msg)
                    before = clock[-1]
                    wait(lambda: clock[-1] > before + 0.2)
                    assert abs(odometry[-1].twist.twist.linear.x) < 0.01
                assert all(c.state == "active" for c in controllers())
        print("PASS description, state, clock, TF and controller interfaces")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
