"""One converter, three formats: CAD -> URDF + MJCF + xacro.

Orchestrates the generation DAG for a ``robots/<robot>/`` directory. The committed flat
URDF (``urdf/<robot>.urdf`` + ``meshes/visual/``) is the kinematic hub; the MJCF and
xacro are both derived from it, so the three formats share one kinematic origin (parity
by construction).

Stages (each runnable alone via ``--only``). The three format stages are sibling modules,
each exposing ``generate(robot_dir) -> list[Path]``:
  onshape  onshape_to_urdf : Onshape CAD -> urdf/<robot>.urdf + meshes/visual/  (--from-cache skips)
  urdf     finalize_urdf   : finalize the hub (harmonise effort, ../meshes/visual/, base_link-free)
  mjcf     urdf_to_mjcf    : MuJoCo compile + post-process + <option>
  xacro    urdf_to_xacro   : description macro (+base_link) + ros2_control + assembly
  package                  : register the robot in the repo-root CMakeLists

Usage:
  robot-assets-generate lite_dummy                 # full pipeline (auto --from-cache if hub present)
  robot-assets-generate lite_dummy --only mjcf,xacro
  robot-assets-generate ./robots/lite_dummy --from-cache
"""

import argparse
from pathlib import Path
import re

from . import finalize_urdf, robot_model, urdf_to_mjcf, urdf_to_xacro

STAGES = ["onshape", "urdf", "mjcf", "xacro", "package"]


def _report(stage: str, written: list[Path]) -> None:
    for path in written:
        print(f"[{stage}] wrote {path}")


def stage_onshape(robot_dir: Path) -> None:
    from . import onshape_to_urdf

    print(f"[onshape] exporting {robot_dir.name} from Onshape (this hits the API)...")
    onshape_to_urdf.export(robot_dir)


def stage_package(robot_dir: Path, package: str) -> None:
    """Idempotently register the robot in the repo-root CMakeLists ROBOTS list."""
    robot = robot_dir.name
    cmake = robot_dir.parent.parent / "CMakeLists.txt"  # robots/<robot>/ -> repo root
    if not cmake.exists():
        print(f"[package] no CMakeLists at {cmake}; skipping")
        return
    text = cmake.read_text()
    match = re.search(r"set\(ROBOTS(?P<body>.*?)\)", text, re.DOTALL)
    if not match:
        print("[package] ROBOTS block not found; skipping")
        return
    listed = match.group("body").split()
    if robot in listed:
        return
    listed.append(robot)
    new_block = "set(ROBOTS\n  " + "\n  ".join(sorted(listed)) + "\n)"
    cmake.write_text(text[: match.start()] + new_block + text[match.end():])
    print(f"[package] registered {robot} in {cmake.name}")


def generate(
    robot: str,
    *,
    only: list[str] | None = None,
    from_cache: bool = False,
    package: str = robot_model.DEFAULT_PACKAGE,
) -> None:
    robot_dir = robot_model.resolve_robot_dir(robot)
    stages = only if only else list(STAGES)

    hub = robot_dir / "urdf" / f"{robot_dir.name}.urdf"
    if "onshape" in stages and (from_cache or (only is None and hub.exists())):
        if from_cache:
            print("[onshape] skipped (--from-cache)")
        else:
            print(f"[onshape] skipped (committed URDF present at {hub}); pass --force to re-export")
        stages = [s for s in stages if s != "onshape"]

    for stage in stages:
        if stage == "onshape":
            stage_onshape(robot_dir)
        elif stage == "urdf":
            _report("urdf", finalize_urdf.generate(robot_dir))
        elif stage == "mjcf":
            _report("mjcf", urdf_to_mjcf.generate(robot_dir))
        elif stage == "xacro":
            _report("xacro", urdf_to_xacro.generate(robot_dir, package=package))
        elif stage == "package":
            stage_package(robot_dir, package)
        else:
            raise ValueError(f"Unknown stage '{stage}'. Valid: {STAGES}")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Generate URDF + MJCF + xacro for a robot from CAD.")
    parser.add_argument("robot", help="Robot name (under ./robots/) or a path to a robot dir.")
    parser.add_argument("--from-cache", action="store_true",
                        help="Skip the Onshape export; reuse the committed urdf/ hub.")
    parser.add_argument("--force", action="store_true",
                        help="Run the Onshape export even if a cached export exists.")
    parser.add_argument("--only", help="Comma-separated subset of stages to run, e.g. 'mjcf,xacro'.")
    parser.add_argument("--package", default=robot_model.DEFAULT_PACKAGE, help="Owning ament package name.")
    args = parser.parse_args(argv)

    only = [s.strip() for s in args.only.split(",")] if args.only else None
    from_cache = args.from_cache and not args.force
    generate(args.robot, only=only, from_cache=from_cache, package=args.package)


if __name__ == "__main__":
    main()
