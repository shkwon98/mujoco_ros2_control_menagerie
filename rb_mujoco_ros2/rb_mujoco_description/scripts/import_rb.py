#!/usr/bin/env python3
"""Import an official rbpodo_description checkout and generate fixed-base MJCF."""

import argparse
import math
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh
import xacro


ET.register_namespace("xacro", "http://www.ros.org/wiki/xacro")
XACRO = "{http://www.ros.org/wiki/xacro}"
ROOT = Path(__file__).resolve().parents[1]

# Use RB5-850E's finish for every model, independent of CAD exporter colors.
SILVER_CAD_COLORS = {
    (0.0, 0.318547, 1.0),
    (0.0, 0.6, 1.0),
    (0.033105, 0.318547, 1.0),
    (0.2, 0.6, 1.0),
    (0.527115, 0.768151, 0.730461),
    (0.752941, 0.890196, 0.870588),
}
ARM_COVER_MATERIALS = {
    ("rb20_1900es_u", "link3_visual_2"),
    ("rb30_1400es_u", "link2_visual_5"),
    ("rb20_1900es_u", "link4_visual_3"),
    ("rb30_1400es_u", "link4_visual_3"),
}


def product_rgba(source: str, model: str, material: str) -> str:
    """Map housings, covers and seals to the RB5 silver/gray/black palette."""
    rgb = tuple(round(float(value), 6) for value in source.split()[:3])
    if (model, material) in ARM_COVER_MATERIALS:
        return "0.3 0.3 0.3 1"
    if rgb == (0.072272, 0.090842, 0.097587):
        return ("0.0722719 0.0908417 0.0975874 1" if material.startswith("link0_")
                else "0.3 0.3 0.3 1")
    if rgb in SILVER_CAD_COLORS:
        return "0.8 0.8 0.8 1"
    if min(rgb) >= 0.79 or (material.startswith("link6_") and (
            min(rgb) > 0.65 or (max(rgb) == min(rgb) and (
                rgb[0] > 0.35 or rgb[0] in (0.184314, 0.028426))))):
        return "0.8 0.8 0.8 1"
    # Seal/collar grays occur in both sRGB and linear-light CAD exports.
    if max(rgb) <= 0.1 or (max(rgb) == min(rgb) and rgb[0] in {
            0.184314, 0.25098, 0.501961, 0.051269, 0.215861, 0.028426}):
        return "0.1 0.1 0.1 1"
    return "0.3 0.3 0.3 1"


def rb1_visual_parts(mesh: trimesh.Trimesh, link: str) -> list[tuple[trimesh.Trimesh, str]]:
    """Paint RB1's uncolored CAD triangles at its measured cover/seal boundaries.

    Coordinates are in the upstream link frame, in metres. Partitioning keeps
    the source surface; it changes neither the shape nor collision meshes.
    """
    cuts = {
        "link0": ((2, 0.095), (2, 0.0972)),
        "link1": ((1, 0.01), (2, -0.05), (2, -0.047)),
        "link2": ((2, 0.044), (2, 0.047), (2, 0.146), (2, 0.149), (1, -0.12)),
        "link3": ((2, -0.012), (1, 0.012), (2, 0.015), (2, 0.049), (2, 0.052)),
        "link4": ((2, 0.1), (2, 0.103), (2, 0.185), (2, 0.188), (1, -0.11), (2, 0.2)),
        "link5": ((2, -0.012), (1, 0.012), (2, 0.015), (2, 0.049), (2, 0.052)),
        "link6": ((2, 0.0566), (2, 0.0616)),
    }
    pieces = [mesh]
    for axis, value in cuts[link]:
        origin, normal = np.zeros(3), np.eye(3)[axis]
        origin[axis] = value
        divided = []
        for piece in pieces:
            if piece.bounds[0, axis] < value < piece.bounds[1, axis]:
                for direction in (normal, -normal):
                    vertices, faces, _ = trimesh.intersections.slice_faces_plane(
                        piece.vertices, piece.faces, direction, origin)
                    if len(faces):
                        part = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
                        part.remove_unreferenced_vertices()
                        divided.append(part)
            else:
                divided.append(piece)
        pieces = divided
    mesh = trimesh.util.concatenate(pieces)
    _, y, z = mesh.triangles_center.T
    finish = np.zeros(len(z), dtype=np.uint8)  # silver housing
    if link == "link0":
        finish[(z > 0.095) & (z < 0.0972)] = 2
    elif link == "link1":
        finish[y > 0.01] = 1
        finish[(z > -0.05) & (z < -0.047)] = 2
    elif link == "link2":
        finish[((z > 0.044) & (z < 0.149)) | (y < -0.12)] = 1
        finish[((z > 0.044) & (z < 0.047)) | ((z > 0.146) & (z < 0.149))] = 2
    elif link in ("link3", "link5"):
        finish[(z < -0.012) | ((y > 0.012) & (z < 0.015))] = 1
        finish[(z > 0.049) & (z < 0.052)] = 2
    elif link == "link4":
        finish[((z > 0.1) & (z < 0.188)) | ((y < -0.11) & (z > 0.2))] = 1
        finish[((z > 0.1) & (z < 0.103)) | ((z > 0.185) & (z < 0.188))] = 2
    else:  # tool flange
        finish[(z > 0.0566) & (z < 0.0616)] = 2
    return [(mesh.submesh([np.flatnonzero(finish == index)], append=True), rgba)
            for index, rgba in enumerate(("0.8 0.8 0.8 1", "0.3 0.3 0.3 1", "0.1 0.1 0.1 1"))
            if (finish == index).any()]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("upstream", type=Path, help="Official rbpodo_description directory")
    args = parser.parse_args()
    upstream = args.upstream.resolve()
    for directory in ("urdf", "mjcf", "meshes"):
        (ROOT / directory).mkdir(exist_ok=True)
    shutil.copy2(upstream / "LICENSE", ROOT / "LICENSE.rb_description")

    with tempfile.TemporaryDirectory() as directory:
        temporary = Path(directory)
        geometry = temporary / "rb_6dof.xacro"
        geometry.write_text((upstream / "robots" / "rb_6dof.xacro").read_text().replace(
            "$(find rbpodo_description)", str(upstream)
        ))
        for source in sorted((upstream / "robots").glob("rb*.urdf.xacro")):
            model_name = source.name.removesuffix(".urdf.xacro")
            for mesh in (upstream / "meshes" / model_name).rglob("*"):
                if mesh.suffix in (".dae", ".stl"):
                    destination = ROOT / mesh.relative_to(upstream)
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(mesh, destination)
                    if mesh.suffix == ".stl":
                        with mesh.open("rb") as stream:
                            stream.seek(80)
                            face_count = struct.unpack("<I", stream.read(4))[0]
                        # MuJoCo accepts at most 200,000 faces in binary STL.
                        if face_count > 200_000:
                            trimesh.load(mesh, force="mesh").export(
                                destination.with_suffix(".obj")
                            )

            robot = ET.parse(source).getroot()
            for child in list(robot):
                if child.tag == XACRO + "include":
                    if "ros2_control" in child.get("filename"):
                        robot.remove(child)
                    else:
                        child.set("filename", str(geometry))
                elif child.tag == XACRO + "rb_6dof_ros2_control":
                    robot.remove(child)
            input_file = temporary / "robot.urdf.xacro"
            ET.ElementTree(robot).write(input_file, encoding="unicode")
            description = xacro.process_file(str(input_file)).toxml().replace(
                "package://rbpodo_description/", "package://rb_mujoco_description/"
            )
            robot = ET.fromstring(description)
            ET.indent(robot, space="  ")
            ET.ElementTree(robot).write(ROOT / "urdf" / f"{model_name}.urdf",
                                        encoding="unicode", xml_declaration=True)

            for mesh in robot.findall(".//mesh"):
                path = ROOT / mesh.get("filename").split("/", 3)[3]
                if path.with_suffix(".obj").is_file():
                    path = path.with_suffix(".obj")
                mesh.set("filename", str(path))
            ET.SubElement(ET.SubElement(robot, "mujoco"), "compiler",
                          fusestatic="false", discardvisual="true", strippath="false")
            input_file = temporary / "robot.urdf"
            ET.ElementTree(robot).write(input_file, encoding="unicode")
            compiled = mujoco.MjModel.from_xml_path(str(input_file))
            output_file = ROOT / "mjcf" / f"{model_name}.xml"
            mujoco.mj_saveLastXML(str(output_file), compiled)
            model = ET.parse(output_file).getroot()
            for mesh in model.findall("asset/mesh"):
                mesh.set("file", "../" + str(Path(mesh.get("file")).relative_to(ROOT)))
            ET.SubElement(model, "option", timestep="0.002", integrator="implicitfast")
            visual_settings = ET.SubElement(model, "visual")
            ET.SubElement(visual_settings, "headlight", diffuse="0.6 0.6 0.6",
                          ambient="0.3 0.3 0.3", specular="0 0 0")
            ET.SubElement(visual_settings, "rgba", haze="0.15 0.25 0.35 1")
            ET.SubElement(visual_settings, "global", azimuth="140", elevation="-20")
            asset = model.find("asset")
            ET.SubElement(asset, "texture", type="skybox", builtin="gradient",
                          rgb1="0.3 0.5 0.7", rgb2="0 0 0", width="512", height="3072")
            ET.SubElement(asset, "texture", type="2d", name="groundplane", builtin="checker",
                          mark="edge", rgb1="0.2 0.3 0.4", rgb2="0.1 0.2 0.3",
                          markrgb="0.8 0.8 0.8", width="300", height="300")
            ET.SubElement(asset, "material", name="groundplane", texture="groundplane",
                          texuniform="true", texrepeat="5 5", reflectance="0.2")
            world = model.find("worldbody")
            ET.SubElement(world, "light", pos="0 0 4", dir="0 0 -1", directional="true")
            ET.SubElement(world, "geom", name="floor", type="plane", size="0 0 0.05",
                          material="groundplane")
            # The fixed root is welded to world, so MuJoCo's parent collision
            # filter does not cover its overlapping base-joint mounting faces.
            ET.SubElement(ET.SubElement(model, "contact"), "exclude",
                          body1="link0", body2="link1")
            # The simulated servos include ideal gravity compensation.
            for body in world.findall(".//body"):
                body.set("gravcomp", "1")
            for geom in world.findall(".//geom[@type='mesh']"):
                geom.set("group", "3")
            initial = mujoco.MjData(compiled)
            mujoco.mj_forward(compiled, initial)
            bounds = []
            ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
            for visual in sorted((ROOT / "meshes" / model_name / "visual").glob("*.dae")):
                collada = ET.parse(visual).getroot()
                effects = {
                    effect.get("id"): effect.findtext(".//c:diffuse/c:color", namespaces=ns)
                    for effect in collada.findall(".//c:effect", ns)
                }
                colors = {material.get("name"): effects[
                    material.find("c:instance_effect", ns).get("url")[1:]]
                    for material in collada.findall(".//c:material", ns)}
                glb = temporary / "visual.glb"
                # Bake node transforms before glTF export, which can omit small rotations.
                subprocess.run(["assimp", "export", str(visual), str(glb), "-fglb2", "-ptv"],
                               check=True, capture_output=True, text=True)
                scene = trimesh.load_scene(glb, process=False)
                # Assimp changes Collada Z_UP to glTF Y_UP; restore the URDF link frame.
                scene.apply_transform(trimesh.transformations.rotation_matrix(
                    math.pi / 2, (1, 0, 0)))
                body = world.find(f".//body[@name='{visual.stem}']")
                parts = []
                for node in scene.graph.nodes_geometry:
                    transform, geometry_name = scene.graph[node]
                    mesh = scene.geometry[geometry_name].copy()
                    material = mesh.visual.material
                    rgba = colors[material.name] if colors else " ".join(
                        str(value / 255) for value in material.baseColorFactor)
                    mesh.apply_transform(transform)
                    if model_name == "rb1_500es_u":
                        parts.extend(rb1_visual_parts(mesh, visual.stem))
                    else:
                        parts.append((mesh, rgba))
                for index, (mesh, rgba) in enumerate(parts):
                    link = initial.body(visual.stem)
                    points = mesh.vertices @ link.xmat.reshape(3, 3).T + link.xpos
                    bounds.extend([points.min(axis=0), points.max(axis=0)])
                    name = f"{visual.stem}_visual_{index}"
                    rgba = product_rgba(rgba, model_name, name)
                    destination = visual.with_name(f"{name}.obj")
                    mesh.export(destination, include_texture=False, include_color=False)
                    ET.SubElement(asset, "mesh", name=name,
                                  file="../" + str(destination.relative_to(ROOT)))
                    ET.SubElement(asset, "material", name=name, rgba=rgba, specular="0")
                    ET.SubElement(body, "geom", type="mesh", mesh=name, material=name,
                                  group="2", contype="0", conaffinity="0", density="0")
                for obsolete in visual.parent.glob(f"{visual.stem}_visual_*.obj"):
                    if asset.find(f"mesh[@name='{obsolete.stem}']") is None:
                        obsolete.unlink()
            low, high = np.min(bounds, axis=0), np.max(bounds, axis=0)
            ET.SubElement(model, "statistic",
                          center=" ".join(f"{value:.8g}" for value in (low + high) / 2),
                          extent=f"{np.linalg.norm(high - low):.8g}")
            actuator = ET.SubElement(model, "actuator")
            for joint in robot.findall("joint[@type='revolute']"):
                limit = joint.find("limit")
                effort = limit.get("effort")
                ET.SubElement(actuator, "position", name=joint.get("name"),
                              joint=joint.get("name"), kp="2000", kv="200",
                              ctrlrange=f"{limit.get('lower')} {limit.get('upper')}",
                              forcerange=f"-{effort} {effort}")
            ET.indent(model, space="  ")
            ET.ElementTree(model).write(output_file, encoding="unicode")
            print(model_name)


if __name__ == "__main__":
    main()
