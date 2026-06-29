"""Full-fidelity URDF parsing + structural transforms shared by the generator stages.

The committed flat URDF (``urdf/<robot>.urdf``) is the single kinematic source/hub.
This module parses it once and exposes the transforms every emitter needs:

* :func:`harmonize_effort`  -- tighten ``<limit effort>`` to the Robstride spec in
  ``joint_properties.json`` (the raw export leaves it at the Onshape default of 100).
* :func:`rewrite_mesh_filenames` -- retarget mesh references per consumer
  (``package://`` for ROS, ``../meshes/visual/`` for the flat URDF, etc.).
* :func:`inject_base_link` -- add the massless ``base_link`` root that KDL /
  robot_state_publisher want (the CAD root link carries inertia).

Comments and child order are preserved so generated files stay readable/reviewable.
"""

from collections.abc import Callable
from pathlib import Path
import re
import xml.etree.ElementTree as ET

XACRO_NS = "http://www.ros.org/wiki/xacro"


def parse(urdf_path: str | Path) -> ET.ElementTree:
    """Parse a URDF, preserving comments (so re-emitted files keep their structure)."""
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    return ET.parse(urdf_path, parser=parser)


def link_names(root: ET.Element) -> list[str]:
    return [link.get("name") for link in root.findall("link") if link.get("name")]


def child_link_names(root: ET.Element) -> set[str]:
    children: set[str] = set()
    for joint in root.findall("joint"):
        child = joint.find("child")
        if child is not None and child.get("link"):
            children.add(child.get("link"))
    return children


def root_link(root: ET.Element) -> str:
    """The single link that is never a joint's child (the kinematic root)."""
    children = child_link_names(root)
    roots = [name for name in link_names(root) if name not in children]
    if len(roots) != 1:
        raise ValueError(f"Expected exactly one root link, found {roots}.")
    return roots[0]


def joint_limits(root: ET.Element) -> dict[str, dict[str, float]]:
    """Map joint name -> {lower, upper, effort, velocity} from ``<limit>`` (radians)."""
    limits: dict[str, dict[str, float]] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if name is None or limit is None:
            continue
        limits[name] = {
            key: float(limit.get(key))
            for key in ("lower", "upper", "effort", "velocity")
            if limit.get(key) is not None
        }
    return limits


def resolve_properties(name: str, properties: dict) -> dict | None:
    """Look up per-joint config by exact name, then by regex key (``_``-keys ignored)."""
    if name in properties:
        return properties[name]
    for pattern, config in properties.items():
        if pattern.startswith("_"):
            continue
        try:
            if re.fullmatch(pattern, name):
                return config
        except re.error:
            continue
    return None


def harmonize_effort(root: ET.Element, joint_properties: dict) -> None:
    """Set each revolute ``<limit effort>`` to ``effort_limit`` from joint_properties."""
    for joint in root.findall("joint"):
        name = joint.get("name")
        if joint.get("type") not in {"revolute", "prismatic"} or name is None:
            continue
        config = resolve_properties(name, joint_properties)
        if config is None or "effort_limit" not in config:
            continue
        limit = joint.find("limit")
        if limit is not None:
            limit.set("effort", str(config["effort_limit"]))


def weld_joints(root: ET.Element, joint_properties: dict) -> None:
    """Convert joints flagged ``"fixed": true`` in joint_properties to fixed joints.

    Drops the movable-only children (``<axis>`` / ``<limit>`` / ``<dynamics>`` /
    ``<mimic>``) so the joint rigidly welds its child to its parent. This retires a
    CAD-real DoF (e.g. a locked waist) from the kinematic hub -- so the description
    xacro, the MJCF (MuJoCo welds a fixed joint), and URDF<->MJCF parity all see a
    rigid weld -- without re-exporting from Onshape.
    """
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name is None or joint.get("type") == "fixed":
            continue
        config = resolve_properties(name, joint_properties)
        if not (config and config.get("fixed")):
            continue
        joint.set("type", "fixed")
        for tag in ("axis", "limit", "dynamics", "mimic"):
            for element in joint.findall(tag):
                joint.remove(element)


def rewrite_mesh_filenames(root: ET.Element, rewrite: Callable[[str], str]) -> None:
    """Rewrite every ``<mesh filename>``; ``rewrite`` receives the file basename."""
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename")
        if filename:
            mesh.set("filename", rewrite(Path(filename).name))


def inject_base_link(root: ET.Element, base_name: str, child_name: str) -> None:
    """Prepend a massless ``base_name`` link + fixed joint to ``child_name`` (the CAD root)."""
    base_link = ET.Element("link", {"name": base_name})
    joint = ET.Element("joint", {"name": f"{base_name}_to_{child_name}", "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": base_name})
    ET.SubElement(joint, "child", {"link": child_name})
    ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    root.insert(0, joint)
    root.insert(0, base_link)


def serialize(tree: ET.ElementTree, path: str | Path) -> None:
    """Indent and write a URDF tree with an XML declaration."""
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def to_string(element: ET.Element) -> str:
    ET.indent(element, space="  ")
    return ET.tostring(element, encoding="unicode")


def autogen_comment(robot: str) -> str:
    """Banner text (no ``<!-- -->`` markers) marking a generated asset file.

    Used verbatim as an XML comment at the top of every generated URDF / MJCF / xacro
    so the source and the regenerate command are obvious to anyone who opens the file.
    """
    # NB: an XML comment may not contain '--', so keep the text dash-free.
    return (
        f" GENERATED by robot_assets from Onshape CAD. Do not edit by hand. "
        f"Regenerate: robot-assets-generate {robot} "
    )


# ---------------------------------------------------------------------------
# Repository layout (shared by the converter stages and their CLIs)
# ---------------------------------------------------------------------------

# Per-robot assets live under ./<ROBOTS_DIR>/<robot>/ (franka_description layout).
ROBOTS_DIR = "robots"
# Owning ament package name, used in package:// URLs and $(find ...).
DEFAULT_PACKAGE = "lite_description"


def resolve_robot_dir(robot: str) -> Path:
    """Accept a path to a robot dir, or a robot name under ./<ROBOTS_DIR>/<name>."""
    path = Path(robot)
    if path.is_dir() and (path / "cad").is_dir():
        return path
    candidate = Path(ROBOTS_DIR) / robot
    if candidate.is_dir():
        return candidate
    raise FileNotFoundError(
        f"Could not resolve robot '{robot}'. Expected a robot dir with a cad/ subdir, "
        f"or a name under ./{ROBOTS_DIR}/.",
    )
