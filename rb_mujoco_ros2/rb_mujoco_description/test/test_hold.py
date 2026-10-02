"""Compose the actual hand-mounted model and test using ROS's native MuJoCo library."""
from pathlib import Path
from runpy import run_path
from tempfile import TemporaryDirectory
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
import xacro

root = Path(__file__).resolve().parents[1]
compose = run_path(str(root / "urdf/compose_wuji.py"))["compose_wuji"]
hand = Path(get_package_share_directory("wuji_hand2_beta2_description"))
for model in sorted((root / "mjcf").glob("rb*.xml")):
    print(f"Checking {model.stem}: bare arm", flush=True)
    subprocess.run([sys.argv[1], str(model)], check=True)
    xml = xacro.process_file(str(root / "urdf/rb_mujoco.urdf.xacro"),
                             mappings={"robot_model": model.stem, "headless": "true"}).toxml()
    for side in ("left", "right"):
        with TemporaryDirectory() as directory:
            compose(xml, root, hand, model.stem, "wuji_hand2_beta2", side, Path(directory))
            print(f"Checking {model.stem}: {side} Hand 2", flush=True)
            subprocess.run([sys.argv[1], str(Path(directory) / "model.xml")], check=True)
