"""Finalize the flat URDF in place -- the kinematic hub the mjcf/xacro stages derive from.

Harmonizes each ``<limit effort>`` to ``joint_properties.json``, normalizes mesh paths to
``../meshes/visual/``, and stamps the generated-asset banner. The hub stays
``base_link``-free (``base_link`` is a ROS/KDL concern injected only into the description
xacro), which keeps this stage idempotent and the MJCF rooted at the CAD root link.

Parallel to urdf_to_mjcf / urdf_to_xacro: exposes ``generate(robot_dir) -> list[Path]``.
"""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from . import robot_model


def ensure_autogen_comment(root: ET.Element, text: str) -> None:
    """Idempotently make the first child of <robot> the generated-asset banner comment."""
    while len(root) and not isinstance(root[0].tag, str):  # comment nodes have a non-str tag
        root.remove(root[0])
    root.insert(0, ET.Comment(text))


def generate(robot_dir: Path) -> list[Path]:
    robot_dir = Path(robot_dir)
    robot = robot_dir.name
    cad_dir = robot_dir / "cad"
    hub_urdf = robot_dir / "urdf" / f"{robot}.urdf"
    if not hub_urdf.exists():
        raise FileNotFoundError(
            f"{hub_urdf} not found. Run the onshape stage first (full export), or commit "
            f"the URDF so --from-cache has a hub to finalize.",
        )
    joint_properties = json.loads((cad_dir / "joint_properties.json").read_text())

    tree = robot_model.parse(hub_urdf)
    root = tree.getroot()
    robot_model.harmonize_effort(root, joint_properties)
    robot_model.rewrite_mesh_filenames(root, lambda name: f"../meshes/visual/{name}")
    ensure_autogen_comment(root, robot_model.autogen_comment(robot))

    robot_model.serialize(tree, hub_urdf)
    return [hub_urdf]


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Finalize a robot's flat URDF hub in place.")
    parser.add_argument("robot", help="Robot name (under ./robots/) or a path to a robot dir.")
    args = parser.parse_args(argv)
    for path in generate(robot_model.resolve_robot_dir(args.robot)):
        print(f"Finalized {path}")


if __name__ == "__main__":
    main()
