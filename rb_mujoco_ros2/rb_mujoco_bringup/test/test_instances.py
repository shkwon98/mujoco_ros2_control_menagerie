"""Exercise installed RB launches in an isolated ROS domain without teleoperation."""

import os
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import pytest
import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


@pytest.mark.parametrize("named", [False, True])
def test_rb_instances(tmp_path, named):
    assert os.environ.get("ROS_DOMAIN_ID") == "230"
    rclpy.init()
    probe = rclpy.create_node("rb_instance_probe")
    processes, files = [], []
    received, frames = {}, set()
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    probe.create_subscription(
        TFMessage,
        "/tf_static",
        lambda message: frames.update(
            transform.child_frame_id for transform in message.transforms
        ),
        QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    probe.create_subscription(
        TFMessage,
        "/tf",
        lambda message: frames.update(
            transform.child_frame_id for transform in message.transforms
        ),
        100,
    )
    instances = [("rb_01", "left"), ("rb_02", "right")] if named else [("", "right")]
    try:
        for index, (instance, side) in enumerate(instances):
            log = tmp_path / f"rb_{index}.log"
            output = log.open("w")
            files.append(output)
            command = [
                "ros2",
                "launch",
                "rb_mujoco_bringup",
                "robot.launch.py",
                "robot_model:=rb5_850e",
                "hand_model:=wuji_hand2",
                f"hand_side:={side}",
                "headless:=true",
            ]
            if instance:
                command += [f"instance_id:={instance}", f"base_xyz:=0 {index} 0"]
            processes.append(
                subprocess.Popen(
                    command,
                    stdout=output,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    env={
                        **os.environ,
                        "ROS_LOG_DIR": str(tmp_path / "ros_logs"),
                        "ROS_HOME": str(tmp_path / f"ros_home_{index}"),
                    },
                )
            )
            body = f"/{instance}" if named else "/sensors/proprio/body"
            hand = (
                f"/wuji_hand_01/{side}_eef"
                if named
                else f"/sensors/proprio/hand_{side}"
            )
            descriptions = (
                [f"/{instance}/robot_description", hand + "/robot_description"]
                if named
                else [
                    "/control/body/robot_description",
                    f"/control/hand_{side}/robot_description",
                ]
            )
            if named:
                descriptions += [
                    topic.replace(
                        "/robot_description", "/visualization/robot_description"
                    )
                    for topic in descriptions
                ]
            for message_type, topics, profile in (
                (JointState, (body + "/joint_states", hand + "/joint_states"), 10),
                (String, descriptions, qos),
            ):
                for topic in topics:
                    probe.create_subscription(
                        message_type,
                        topic,
                        lambda message, topic=topic: received.update({topic: message}),
                        profile,
                    )
        expected_frames = {
            frame
            for instance, side in instances
            for frame in (
                (instance + "_link0", f"wuji_hand_01_{side}_flange")
                if named
                else ("link0",)
            )
        }
        expected_messages = (6 if named else 4) * len(instances)
        deadline = time.monotonic() + 30
        while (
            len(received) < expected_messages or not expected_frames <= frames
        ) and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)
        logs = "\n".join(
            (tmp_path / f"rb_{index}.log").read_text()
            for index in range(len(instances))
        )
        assert len(received) == expected_messages and expected_frames <= frames, (
            frames,
            logs,
        )
        for instance, side in instances:
            manager = (
                f"/{instance}/controller_manager" if named else "/controller_manager"
            )
            client = probe.create_client(ListControllers, manager + "/list_controllers")
            assert client.wait_for_service(timeout_sec=5), logs
            expected = f"{side}_eef_controller" if named else "hand_right_controller"
            deadline = time.monotonic() + 10
            required = {"arm_controller", expected}
            active = set()
            while time.monotonic() < deadline:
                future = client.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(probe, future, timeout_sec=2)
                assert future.done() and future.result() is not None, logs
                active = {
                    controller.name
                    for controller in future.result().controller
                    if controller.state == "active"
                }
                if required <= active:
                    break
                rclpy.spin_once(probe, timeout_sec=0.1)
            assert required <= active, (active, logs)
            arm_namespace = f"/{instance}" if named else "/control/body"
            hand_namespace = "/wuji_hand_01" if named else "/control/hand_right"
            for namespace, controller in (
                (arm_namespace, "arm_controller"),
                (hand_namespace, expected),
            ):
                action = ActionClient(
                    probe,
                    FollowJointTrajectory,
                    f"{namespace}/{controller}/follow_joint_trajectory",
                )
                assert action.wait_for_server(timeout_sec=5), logs
                action.destroy()
            body = f"/{instance}" if named else "/sensors/proprio/body"
            assert (
                received[body + "/joint_states"].name
                == "base shoulder elbow wrist1 wrist2 wrist3".split()
            )
            if named:
                assert received[body + "/joint_states"].header.stamp.sec > 1_000_000_000
                for namespace, prefix in (
                    (f"/{instance}", instance + "_"),
                    (f"/wuji_hand_01/{side}_eef", f"wuji_hand_01_{side}_"),
                ):
                    original = ET.fromstring(
                        received[namespace + "/robot_description"].data
                    )
                    visual = ET.fromstring(
                        received[namespace + "/visualization/robot_description"].data
                    )
                    links = {link.get("name") for link in visual.findall("link")}
                    assert links == {
                        prefix + link.get("name") for link in original.findall("link")
                    }
                    assert [joint.get("name") for joint in visual.findall("joint")] == [
                        joint.get("name") for joint in original.findall("joint")
                    ]
                    assert all(
                        reference.get("link") in links
                        for reference in visual.findall("joint/parent")
                        + visual.findall("joint/child")
                    )
                    assert links <= frames, links - frames
                assert probe.count_publishers(f"/{instance}/clock") == 1
        if named:
            assert probe.count_publishers("/clock") == 0
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        for output in files:
            output.close()
        probe.destroy_node()
        rclpy.shutdown()
