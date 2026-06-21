"""Generate the ROS 2 xacro artifacts for a robot from its raw URDF + ros2_control.json.

Emits three files into ``<robot_dir>/xacro/`` (the ``ros2_control_demos`` convention:
a reusable model macro + a separate ``*.ros2_control.xacro`` + a thin top assembly):

* ``<robot>.description.xacro``  -- ``<xacro:macro name="<robot>_description">`` wrapping
  the CAD kinematics: a ``base_link`` root, ``${mesh_root}``-parameterised mesh paths
  (default ``package://``), effort limits harmonised to ``joint_properties.json``.
* ``<robot>.ros2_control.xacro`` -- the hardware macros: a per-joint MIT-mode interface
  macro, per-group joint macros, a combined block (sim/mock) and per-bus blocks (real),
  and a top dispatch macro that selects the backend via ``use_sim``/``use_fake_hardware``.
* ``<robot>.urdf.xacro``         -- top assembly: declares args, includes + instantiates
  the two macros above.

All `${...}` are xacro expressions written as ``${{...}}`` inside f-strings (the doubled
braces collapse to single braces at format time); ``$(arg ...)`` / ``$(find ...)`` use
parentheses and need no escaping. Limits (lower/upper) are read from the URDF, never
re-declared in ros2_control.json.
"""

import argparse
import json
from pathlib import Path
import textwrap
import xml.etree.ElementTree as ET

from . import robot_model

DEFAULT_PACKAGE = robot_model.DEFAULT_PACKAGE

def _banner(robot: str) -> str:
    """The generated-asset banner as a literal XML comment (xacro files are string-built)."""
    return f"<!--{robot_model.autogen_comment(robot)}-->"

# Standard backend-switch arg names (franka_ros2 / Universal_Robots convention).
SIM_ARG = "use_sim"
MOCK_ARG = "use_fake_hardware"


# ---------------------------------------------------------------------------
# Description macro (model kinematics)
# ---------------------------------------------------------------------------


def build_description_xacro(
    urdf_path: Path,
    robot: str,
    joint_properties: dict,
    base_link: dict,
    package: str = DEFAULT_PACKAGE,
) -> str:
    tree = robot_model.parse(urdf_path)
    root = tree.getroot()

    robot_model.harmonize_effort(root, joint_properties)
    robot_model.rewrite_mesh_filenames(root, lambda name: f"${{mesh_root}}/{name}")
    child = base_link.get("child") or robot_model.root_link(root)
    robot_model.inject_base_link(root, base_link.get("name", "base_link"), child)

    mesh_root_default = f"package://{package}/robots/{robot}/meshes/visual"
    ET.register_namespace("xacro", robot_model.XACRO_NS)
    out_root = ET.Element("robot")
    macro = ET.SubElement(
        out_root,
        f"{{{robot_model.XACRO_NS}}}macro",
        {"name": f"{robot}_description", "params": f"mesh_root:={mesh_root_default}"},
    )
    for element in list(root):
        macro.append(element)

    body = robot_model.to_string(out_root)
    return f'<?xml version="1.0"?>\n{_banner(robot)}\n{body}\n'


# ---------------------------------------------------------------------------
# ros2_control macros
# ---------------------------------------------------------------------------


def _joint_macro(robot: str, command: list[str], state: list[str]) -> str:
    cmd_lines = "\n".join(f'      <command_interface name="{n}"/>' for n in command)
    state_lines = "\n".join(f'      <state_interface name="{n}"/>' for n in state)
    return f"""  <!-- One joint: {len(command)} command + {len(state)} state interfaces (MIT mode).
       Real-only hardware params are emitted under xacro:unless (sim/mock ignore them). -->
  <xacro:macro name="{robot}_joint"
               params="name can_id model direction
                       lower_limit upper_limit torque_limit current_limit
                       use_fake_hardware use_sim">
    <joint name="${{name}}">
{cmd_lines}
{state_lines}
      <xacro:unless value="${{use_sim or use_fake_hardware}}">
        <param name="can_id">${{can_id}}</param>
        <param name="model">${{model}}</param>
        <param name="direction">${{direction}}</param>
        <param name="lower_limit">${{lower_limit}}</param>
        <param name="upper_limit">${{upper_limit}}</param>
        <param name="torque_limit">${{torque_limit}}</param>
        <param name="current_limit">${{current_limit}}</param>
      </xacro:unless>
    </joint>
  </xacro:macro>"""


def _joint_call(robot: str, joint: dict, limits: dict) -> str:
    limit = limits.get(joint["name"], {})
    lower = limit.get("lower", 0.0)
    upper = limit.get("upper", 0.0)
    return (
        f'    <xacro:{robot}_joint name="{joint["name"]}" can_id="{joint["can_id"]}" '
        f'model="{joint["model"]}" direction="{joint["direction"]}"\n'
        f'                      lower_limit="{lower}" upper_limit="{upper}"\n'
        f'                      torque_limit="{joint["torque_limit"]}" '
        f'current_limit="{joint["current_limit"]}"\n'
        f'                      use_fake_hardware="${{use_fake_hardware}}" '
        f'use_sim="${{use_sim}}"/>'
    )


def _group_macro(robot: str, group: str, joints: list[dict], limits: dict) -> str:
    if not joints:
        return f"""  <!-- {group}: stub. Populate when the CAN ids / model codes are confirmed on hardware. -->
  <xacro:macro name="{robot}_{group}_joints" params="use_fake_hardware use_sim">
    <!-- TODO: {group} joint declarations land here. -->
  </xacro:macro>"""
    calls = "\n".join(_joint_call(robot, j, limits) for j in joints)
    return f"""  <xacro:macro name="{robot}_{group}_joints" params="use_fake_hardware use_sim">
{calls}
  </xacro:macro>"""


def _combined_macro(robot: str, name: str, active_groups: list[dict], backends: dict) -> str:
    group_calls = "\n".join(
        f'      <xacro:{robot}_{g["name"]}_joints '
        f'use_fake_hardware="${{use_fake_hardware}}" use_sim="${{use_sim}}"/>'
        for g in active_groups
    )
    return f"""  <!-- Combined single-block layout for the sim and mock backends. -->
  <xacro:macro name="{robot}_ros2_control_combined" params="name use_fake_hardware use_sim">
    <ros2_control name="${{name}}" type="system">
      <hardware>
        <xacro:if value="${{use_sim}}">
          <plugin>{backends["sim"]}</plugin>
        </xacro:if>
        <xacro:unless value="${{use_sim}}">
          <plugin>{backends["mock"]}</plugin>
        </xacro:unless>
      </hardware>
{group_calls}
    </ros2_control>
  </xacro:macro>"""


def _real_block(robot: str, group: dict, real_plugin: str) -> str:
    """One <ros2_control> bus block, indented as a direct child of the real macro."""
    can_arg = group["can_interface_arg"]
    return f"""    <ros2_control name="{group["block_name"]}" type="system">
      <hardware>
        <plugin>{real_plugin}</plugin>
        <param name="can_interface">${{{can_arg}}}</param>
        <param name="calibration_file">${{calibration_file}}</param>
      </hardware>
      <xacro:{robot}_{group["name"]}_joints use_fake_hardware="false" use_sim="false"/>
    </ros2_control>"""


def _real_macro(robot: str, groups: list[dict], backends: dict) -> str:
    real_plugin = backends["real"]
    blocks = []
    for group in groups:
        block = _real_block(robot, group, real_plugin)
        if group.get("stub"):
            # Stub buses only exist in mode=arms_neck; nest the block inside the guard.
            block = (
                f"    <!-- {group['name']} bus: only with mode=arms_neck; "
                f"stub until the actuators are wired. -->\n"
                f"    <xacro:if value=\"${{mode == 'arms_neck'}}\">\n"
                f"{textwrap.indent(block, '  ')}\n"
                f"    </xacro:if>"
            )
        blocks.append(block)
    body = "\n".join(blocks)
    return f"""  <!-- Real hardware: one <ros2_control> block per CAN bus. The controller_manager
       runs them concurrently and exposes a single flat joint list to controllers. -->
  <xacro:macro name="{robot}_ros2_control_real"
               params="mode can_interface_left can_interface_right calibration_file">
{body}
  </xacro:macro>"""


def _top_macro(robot: str, name: str) -> str:
    return f"""  <!-- Top-level dispatch: combined block for sim/mock, per-bus blocks for real
       (use_sim wins over use_fake_hardware, matching franka_ros2 / UR precedence). -->
  <xacro:macro name="{robot}_ros2_control"
               params="name use_fake_hardware use_sim mode
                       can_interface_left can_interface_right calibration_file">
    <xacro:if value="${{use_sim or use_fake_hardware}}">
      <xacro:{robot}_ros2_control_combined name="${{name}}"
        use_fake_hardware="${{use_fake_hardware}}" use_sim="${{use_sim}}"/>
    </xacro:if>
    <xacro:unless value="${{use_sim or use_fake_hardware}}">
      <xacro:{robot}_ros2_control_real mode="${{mode}}"
        can_interface_left="${{can_interface_left}}"
        can_interface_right="${{can_interface_right}}"
        calibration_file="${{calibration_file}}"/>
    </xacro:unless>
  </xacro:macro>"""


def build_ros2_control_xacro(robot: str, ros2_control: dict, limits: dict) -> str:
    command = ros2_control["interfaces"]["command"]
    state = ros2_control["interfaces"]["state"]
    backends = ros2_control["backends"]
    groups = ros2_control["groups"]
    combined_name = ros2_control.get("combined_block_name", f"{robot}_system")

    joints_by_group: dict[str, list[dict]] = {}
    for joint in ros2_control["joints"]:
        joints_by_group.setdefault(joint["group"], []).append(joint)

    active_groups = [g for g in groups if not g.get("stub") and joints_by_group.get(g["name"])]

    parts = [_joint_macro(robot, command, state)]
    for group in groups:
        parts.append(_group_macro(robot, group["name"], joints_by_group.get(group["name"], []), limits))
    parts.append(_combined_macro(robot, combined_name, active_groups, backends))
    parts.append(_real_macro(robot, groups, backends))
    parts.append(_top_macro(robot, combined_name))

    body = "\n\n".join(parts)
    return (
        f'<?xml version="1.0"?>\n{_banner(robot)}\n'
        f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n\n{body}\n\n</robot>\n'
    )


# ---------------------------------------------------------------------------
# Top assembly
# ---------------------------------------------------------------------------


def build_assembly_xacro(robot: str, ros2_control: dict | None, package: str = DEFAULT_PACKAGE) -> str:
    """Top assembly. With a ros2_control spec it wires the hardware macro; without one
    (model-only robots like the full lite) it just instantiates the model."""
    if not ros2_control:
        return f"""<?xml version="1.0"?>
{_banner(robot)}
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{robot}">

  <xacro:include filename="$(find {package})/robots/{robot}/xacro/{robot}.description.xacro"/>

  <xacro:{robot}_description/>
</robot>
"""

    args = ros2_control.get("args", {})
    arg_decls = "\n".join(f'  <xacro:arg name="{k}" default="{v}"/>' for k, v in args.items())
    name_attr = ros2_control.get("combined_block_name", f"{robot}_system")
    return f"""<?xml version="1.0"?>
{_banner(robot)}
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{robot}">
{arg_decls}

  <xacro:include filename="$(find {package})/robots/{robot}/xacro/{robot}.description.xacro"/>
  <xacro:include filename="$(find {package})/robots/{robot}/xacro/{robot}.ros2_control.xacro"/>

  <xacro:{robot}_description/>

  <xacro:{robot}_ros2_control
    name="{name_attr}"
    use_fake_hardware="$(arg {MOCK_ARG})"
    use_sim="$(arg {SIM_ARG})"
    mode="$(arg mode)"
    can_interface_left="$(arg can_interface_left)"
    can_interface_right="$(arg can_interface_right)"
    calibration_file="$(arg calibration_file)"/>
</robot>
"""


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------


def generate(robot_dir: Path, package: str = DEFAULT_PACKAGE) -> list[Path]:
    robot = robot_dir.name
    cad_dir = robot_dir / "cad"
    # The committed, finalized flat URDF (base_link-free) is the kinematic hub.
    hub_urdf = robot_dir / "urdf" / f"{robot}.urdf"
    joint_properties = json.loads((cad_dir / "joint_properties.json").read_text())

    # ros2_control.json is optional: robots without it (e.g. the full lite)
    # generate a model-only package -- description macro + a thin assembly, no
    # <ros2_control>. Deployment wiring lives only where the hardware is known.
    ros2_control_path = cad_dir / "ros2_control.json"
    ros2_control = json.loads(ros2_control_path.read_text()) if ros2_control_path.exists() else None

    tree = robot_model.parse(hub_urdf)
    root = tree.getroot()
    limits = robot_model.joint_limits(root)
    base_link = (ros2_control or {}).get("base_link") or {"name": "base_link", "child": robot_model.root_link(root)}

    xacro_dir = robot_dir / "xacro"
    xacro_dir.mkdir(parents=True, exist_ok=True)

    outputs = {
        f"{robot}.description.xacro": build_description_xacro(hub_urdf, robot, joint_properties, base_link, package),
        f"{robot}.urdf.xacro": build_assembly_xacro(robot, ros2_control, package),
    }
    if ros2_control is not None:
        outputs[f"{robot}.ros2_control.xacro"] = build_ros2_control_xacro(robot, ros2_control, limits)

    written = []
    for filename, content in outputs.items():
        path = xacro_dir / filename
        path.write_text(content)
        written.append(path)
    return written


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Generate ROS 2 xacro files for a robot.")
    parser.add_argument("robot", help="Robot name (under ./robots/) or a path to a robot dir.")
    parser.add_argument("--package", default=DEFAULT_PACKAGE, help="Owning ament package name.")
    args = parser.parse_args(argv)
    for path in generate(robot_model.resolve_robot_dir(args.robot), package=args.package):
        print(f"Wrote {path}")


if __name__ == "__main__":
    main()
