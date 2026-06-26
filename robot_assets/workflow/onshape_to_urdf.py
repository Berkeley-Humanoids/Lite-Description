"""Stage 1 of the converter: export the raw URDF + meshes from Onshape CAD.

Wraps the external `onshape-to-robot` tool and normalises its output into the
robots/<robot>/ layout:

    robots/<robot>/
      cad/config.json          (input: Onshape doc + export options)
      cad/scad/                (input: custom collider sources)
      urdf/<robot>.urdf        (output: flat URDF -- the committed kinematic hub)
      meshes/visual/*.stl      (output: merged visual meshes, one copy)

The flat URDF is the committed, cacheable hub; the `urdf` finalize stage then
harmonizes its effort limits, and the mjcf/xacro stages derive from it. With a
committed hub, iterating on physics.json / ros2_control.json via `--from-cache`
never re-hits the Onshape API (~1000 requests per export).
"""

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys

from . import robot_model


def _onshape_to_robot_bin() -> str:
    """Locate the onshape-to-robot CLI (PATH, else alongside the running interpreter)."""
    return shutil.which("onshape-to-robot") or str(Path(sys.executable).parent / "onshape-to-robot")


def export(robot_dir: Path, *, keep_assets: bool = False, convert: bool = False) -> Path:
    """Run onshape-to-robot for ``robot_dir`` and place outputs in the cad/ layout."""
    robot_dir = Path(robot_dir)
    robot = robot_dir.name
    cad_dir = robot_dir / "cad"
    config_path = cad_dir / "config.json"
    if not config_path.exists():
        raise FileNotFoundError(f"Config file {config_path} does not exist!")

    output_filename = json.loads(config_path.read_text()).get("output_filename", robot)
    assets_dir = cad_dir / "assets"
    scad_dir = cad_dir / "scad"

    # onshape-to-robot reads collider sources from <workdir>/assets/.
    if scad_dir.exists():
        assets_dir.mkdir(exist_ok=True)
        for scad_file in scad_dir.iterdir():
            shutil.copy(scad_file, assets_dir / scad_file.name)

    arguments = [_onshape_to_robot_bin(), str(cad_dir)]
    if convert:
        # Offline: reload the existing robot.pkl instead of hitting the Onshape API.
        # --save-pickle must NOT be combined with --convert: onshape-to-robot runs its
        # save branch before the convert branch, but no robot is built on the convert
        # path, so the two together raise a NameError.
        arguments.append("--convert")
    elif keep_assets:
        # Build path: persist robot.pkl so later runs can re-derive offline via --convert.
        arguments.append("--save-pickle")
    subprocess.run(arguments, check=True)

    # Merged visual meshes -> the single meshes/visual/ copy.
    merged = assets_dir / "merged"
    visual_dir = robot_dir / "meshes" / "visual"
    if merged.exists():
        visual_dir.mkdir(parents=True, exist_ok=True)
        shutil.copytree(merged, visual_dir, dirs_exist_ok=True)

    # Flat URDF -> urdf/<robot>.urdf, with mesh refs pointed at meshes/visual/
    # and the ROS package:// scheme stripped. The `urdf` finalize stage then
    # harmonizes effort limits in place.
    produced_urdf = cad_dir / f"{output_filename}.urdf"
    urdf_dir = robot_dir / "urdf"
    urdf_dir.mkdir(parents=True, exist_ok=True)
    hub_urdf = urdf_dir / f"{robot}.urdf"

    content = produced_urdf.read_text()
    content = content.replace("assets/merged/", "../meshes/visual/")
    content = content.replace("package://", "")
    hub_urdf.write_text(content)
    produced_urdf.unlink(missing_ok=True)

    if not keep_assets:
        shutil.rmtree(assets_dir, ignore_errors=True)
        if convert:
            (cad_dir / "robot.pkl").unlink(missing_ok=True)

    print(f"[onshape] export -> {hub_urdf}")
    return hub_urdf


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Export a raw URDF from Onshape CAD.")
    parser.add_argument("robot", help="Robot name (under ./robots/) or a path to a robot dir.")
    parser.add_argument("--keep-assets", action="store_true", help="Keep the assets dir and robot.pkl.")
    parser.add_argument("--convert", action="store_true", help="Convert from a local robot.pkl.")
    args = parser.parse_args(argv)
    export(robot_model.resolve_robot_dir(args.robot),
           keep_assets=args.keep_assets, convert=args.convert)


if __name__ == "__main__":
    main()
