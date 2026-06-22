"""Compile a raw URDF into a post-processed MJCF (training + deployment sim asset).

Pipeline (called by ``generate.py``; the flat URDF is the hub):
  raw URDF + meshes  ->  MuJoCo compile  ->  post-process:
    * replace cylinder geoms with capsules
    * inject <option> physics tuning (from cad/physics.json)
    * synthesize <motor> actuators (forcerange from joint_properties effort_limit;
      no <sensor> block -- see add_actuators)
    * set per-joint frictionloss / armature (from joint_properties)
  ->  mjcf/<robot>.xml  (meshdir -> ../meshes/visual/)

Mesh resolution is decoupled from the URDF's relative paths: mesh refs are rewritten
to bare basenames and MuJoCo is pointed at the real meshes dir during compilation, so
this works regardless of where the raw URDF lives.
"""

import argparse
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco

from . import robot_model

# Compiler options applied to the URDF before MuJoCo import (meshdir is set separately
# to the real meshes dir so compilation resolves the STLs).
MUJOCO_COMPILER_OPTIONS = {
    "discardvisual": "false",
    "fusestatic": "false",
    "angle": "radian",
}


def add_mujoco_compiler_tag(urdf_file_path: Path, meshdir: str) -> None:
    tree = ET.parse(urdf_file_path)
    root = tree.getroot()

    mujoco_tag = root.find("mujoco")
    if mujoco_tag is None:
        mujoco_tag = ET.SubElement(root, "mujoco")

    compiler_tag = mujoco_tag.find("compiler")
    if compiler_tag is None:
        compiler_tag = ET.SubElement(mujoco_tag, "compiler")

    compiler_tag.set("meshdir", meshdir)
    for key, value in MUJOCO_COMPILER_OPTIONS.items():
        compiler_tag.set(key, value)

    tree.write(urdf_file_path, encoding="utf-8", xml_declaration=True)


def save_mjcf_from_urdf(urdf_path: Path, out_xml_path: Path) -> None:
    model = mujoco.MjModel.from_xml_path(str(urdf_path))
    mujoco.mj_saveLastXML(str(out_xml_path), model)


def set_compiler_meshdir(xml_file_path: Path, meshdir: str) -> None:
    tree = ET.parse(xml_file_path)
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", meshdir)
    tree.write(xml_file_path, encoding="utf-8", xml_declaration=True)


def prepend_comment(xml_file_path: Path, text: str) -> None:
    """Insert ``text`` as an XML comment immediately inside the MJCF root element."""
    tree = ET.parse(xml_file_path)
    root = tree.getroot()
    comment = ET.Comment(text)
    comment.tail = "\n  "
    root.insert(0, comment)
    tree.write(xml_file_path, encoding="utf-8", xml_declaration=True)


# ---------------------------------------------------------------------------
# joint_properties lookup
# ---------------------------------------------------------------------------


def resolve_joint_properties(joint_name: str, joint_properties: dict) -> dict:
    config = robot_model.resolve_properties(joint_name, joint_properties)
    if config is None:
        raise ValueError(
            f"No joint properties found for joint '{joint_name}'. "
            "Add an exact or regex entry to joint_properties.json.",
        )
    return config


def require_joint_attribute(joint_name: str, joint_config: dict, attribute_name: str):
    if attribute_name not in joint_config:
        raise ValueError(
            f"Joint '{joint_name}' is missing required attribute '{attribute_name}' "
            "in joint_properties.json.",
        )
    return joint_config[attribute_name]


# ---------------------------------------------------------------------------
# MJCF post-processing
# ---------------------------------------------------------------------------


def ensure_section(root: ET.Element, tag_name: str, before_tag_name: str | None = None) -> ET.Element:
    section = root.find(tag_name)
    if section is not None:
        section.clear()
        return section

    section = ET.Element(tag_name)
    if before_tag_name is None:
        root.append(section)
        return section

    before_section = root.find(before_tag_name)
    if before_section is not None:
        root.insert(list(root).index(before_section), section)
    else:
        root.append(section)
    return section


def add_option_tag(xml_file_path: Path, physics_options: dict) -> None:
    if not physics_options:
        return
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    option = root.find("option")
    if option is None:
        option = ET.Element("option")
        compiler = root.find("compiler")
        index = list(root).index(compiler) + 1 if compiler is not None else 0
        root.insert(index, option)

    for key, value in physics_options.items():
        if key.startswith("_"):
            continue
        option.set(key, str(value))

    tree.write(xml_file_path, encoding="utf-8", xml_declaration=True)


def format_motor_forcerange_from_effort_limit(effort_limit) -> str:
    """MuJoCo motor `forcerange` as symmetric +/-|effort_limit| (Nm for revolute joints)."""
    if isinstance(effort_limit, (int, float)):
        mag = abs(effort_limit)
        return f"-{mag} {mag}"
    raise ValueError(
        f"effort_limit must be a number, got {type(effort_limit).__name__}: {effort_limit!r}",
    )


def add_actuators(xml_file_path: Path, joint_properties: dict) -> None:
    """Synthesize one <motor> actuator per actuated joint. Emits no <sensor> block.

    jointpos/jointvel sensors are intentionally omitted. They are redundant with
    mjData.qpos/qvel -- mjlab reads joint state from the Entity/Articulation data,
    not from named MJCF sensors -- and they make the model unloadable under
    mujoco_ros2_control: its plugin init loops over every sensor and builds
    ``std::string(mj_id2name(model, mjOBJ_SITE, sensor_objid))``
    (mujoco_ros2_control.cpp:124), which is null for a joint sensor when the model
    has no <site>s -> SIGABRT. Keeping only actuators leaves the single MJCF usable
    by both the RL/training sim and ros2_control.
    """
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    joints: list[str] = []
    for joint in root.iter("joint"):
        joint_name = joint.get("name")
        if joint_name and joint.get("actuatorfrcrange"):
            joints.append(joint_name)

    if not joints:
        print("No joints with actuatorfrcrange found in XML")
        return

    actuator_section = ensure_section(root, "actuator")

    for joint_name in joints:
        motor = ET.SubElement(actuator_section, "motor")
        motor.set("name", joint_name)
        motor.set("joint", joint_name)
        joint_config = resolve_joint_properties(joint_name, joint_properties)
        effort_limit = require_joint_attribute(joint_name, joint_config, "effort_limit")
        motor.set("forcerange", format_motor_forcerange_from_effort_limit(effort_limit))

    tree.write(xml_file_path, encoding="utf-8", xml_declaration=True)


def add_freejoint(xml_file_path: Path) -> None:
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    worldbody = root.find("worldbody")
    first_body = worldbody.find("body") if worldbody is not None else None
    if first_body is None:
        print("No body element found in worldbody")
        return

    freejoint = ET.Element("joint")
    freejoint.set("name", "floating_base_joint")
    freejoint.set("type", "free")
    freejoint.set("limited", "false")
    freejoint.set("actuatorfrclimited", "false")
    first_body.insert(0, freejoint)

    tree.write(xml_file_path, encoding="utf-8", xml_declaration=True)
    print("Added floating_base_joint to first body element")


def apply_joint_properties(xml_file_path: Path, joint_properties: dict) -> None:
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    for joint in root.iter("joint"):
        joint_name = joint.get("name")
        if not joint_name or joint.get("type") == "free":
            continue
        joint_config = resolve_joint_properties(joint_name, joint_properties)
        joint.set("frictionloss", str(require_joint_attribute(joint_name, joint_config, "friction_loss")))
        joint.set("armature", str(require_joint_attribute(joint_name, joint_config, "armature")))

    tree.write(xml_file_path, encoding="utf-8", xml_declaration=True)


def replace_cylinders_with_capsules(xml_file_path: Path) -> int:
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    count = 0
    for geom in root.iter("geom"):
        if geom.get("type") == "cylinder":
            geom.set("type", "capsule")
            count += 1

    if count:
        tree.write(xml_file_path, encoding="utf-8", xml_declaration=True)
    return count


# ---------------------------------------------------------------------------
# Orchestration
# ---------------------------------------------------------------------------


def convert(
    urdf_path: Path,
    output_xml_path: Path,
    joint_properties: dict,
    *,
    meshes_dir: Path,
    out_meshdir: str = "../meshes/visual/",
    physics_options: dict | None = None,
    freejoint: bool = False,
    notice: str | None = None,
) -> Path:
    urdf_path = Path(urdf_path)
    output_xml_path = Path(output_xml_path)
    meshes_dir = Path(meshes_dir)
    output_xml_path.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="rd_mjcf_") as tmp:
        temp_urdf = Path(tmp) / urdf_path.name
        # Rewrite mesh refs to bare basenames so compilation resolves them via meshdir,
        # independent of the raw URDF's own relative paths.
        tree = robot_model.parse(urdf_path)
        robot_model.rewrite_mesh_filenames(tree.getroot(), lambda name: name)
        tree.write(temp_urdf, encoding="utf-8", xml_declaration=True)
        add_mujoco_compiler_tag(temp_urdf, meshdir=str(meshes_dir.resolve()))

        temp_xml = Path(tmp) / "mjmodel.xml"
        save_mjcf_from_urdf(temp_urdf, temp_xml)

        if freejoint:
            add_freejoint(temp_xml)
        replaced = replace_cylinders_with_capsules(temp_xml)
        if replaced:
            print(f"Replaced {replaced} cylinder geom(s) with capsules")
        add_option_tag(temp_xml, physics_options or {})
        add_actuators(temp_xml, joint_properties)
        apply_joint_properties(temp_xml, joint_properties)
        set_compiler_meshdir(temp_xml, out_meshdir)

        output_xml_path.write_text(temp_xml.read_text())

    if notice:
        prepend_comment(output_xml_path, notice)

    return output_xml_path


def generate(robot_dir: Path, *, freejoint: bool = False) -> list[Path]:
    """Generate mjcf/<robot>.xml for a robots/<robot>/ directory."""
    robot_dir = Path(robot_dir)
    robot = robot_dir.name
    cad_dir = robot_dir / "cad"
    hub_urdf = robot_dir / "urdf" / f"{robot}.urdf"
    joint_properties = json.loads((cad_dir / "joint_properties.json").read_text())
    physics_path = cad_dir / "physics.json"
    physics_options = json.loads(physics_path.read_text()) if physics_path.exists() else {}
    mjcf_path = convert(
        hub_urdf,
        robot_dir / "mjcf" / f"{robot}.xml",
        joint_properties,
        meshes_dir=robot_dir / "meshes" / "visual",
        physics_options=physics_options,
        freejoint=freejoint,
        notice=robot_model.autogen_comment(robot),
    )
    return [mjcf_path]


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Generate mjcf/<robot>.xml for a robot.")
    parser.add_argument("robot", help="Robot name (under ./robots/) or a path to a robot dir.")
    parser.add_argument("--freejoint", action="store_true", help="Add a free joint under the first body.")
    args = parser.parse_args(argv)
    for path in generate(robot_model.resolve_robot_dir(args.robot), freejoint=args.freejoint):
        print(f"Wrote {path}")


if __name__ == "__main__":
    main()
