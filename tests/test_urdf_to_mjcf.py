import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import pytest

from robot_assets.workflow.urdf_to_mjcf import add_actuators, replace_cylinders_with_capsules

REPO_ROOT = Path(__file__).resolve().parents[1]
MJCF_FILES = sorted((REPO_ROOT / "robots").glob("*/mjcf/*.xml"))
MJCF_IDS = [p.parent.parent.name for p in MJCF_FILES]


def test_replace_cylinders_with_capsules(tmp_path):
    xml_path = tmp_path / "robot.xml"
    xml_path.write_text(
        """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="base">
      <geom name="upper_arm" type="cylinder" size="0.03 0.04" pos="1 2 3"/>
      <geom name="foot" type="box" size="0.1 0.2 0.3"/>
      <geom name="forearm" type="cylinder" size="0.02 0.05" quat="1 0 0 0"/>
    </body>
  </worldbody>
</mujoco>
""",
    )

    count = replace_cylinders_with_capsules(xml_path)

    root = ET.parse(xml_path).getroot()
    geoms = {geom.get("name"): geom for geom in root.iter("geom")}
    assert count == 2
    assert geoms["upper_arm"].get("type") == "capsule"
    assert geoms["upper_arm"].get("size") == "0.03 0.04"
    assert geoms["upper_arm"].get("pos") == "1 2 3"
    assert geoms["foot"].get("type") == "box"
    assert geoms["forearm"].get("type") == "capsule"


def test_replace_cylinders_with_capsules_returns_zero_when_no_cylinders(tmp_path):
    xml_path = tmp_path / "robot.xml"
    xml_path.write_text(
        """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="base">
      <geom name="foot" type="box" size="0.1 0.2 0.3"/>
    </body>
  </worldbody>
</mujoco>
""",
    )

    assert replace_cylinders_with_capsules(xml_path) == 0


@pytest.mark.skipif(not MJCF_FILES, reason="no generated MJCFs")
@pytest.mark.parametrize("mjcf_path", MJCF_FILES, ids=MJCF_IDS)
def test_committed_mjcf_loads_and_has_no_sensors(mjcf_path):
    """ros2_control safety guard: every committed MJCF must load and carry no sensors.

    mujoco_ros2_control's plugin init loops over every sensor and builds
    ``std::string(mj_id2name(model, mjOBJ_SITE, sensor_objid))``
    (mujoco_ros2_control.cpp:124) -- null for a joint sensor when the model has no
    <site>s, which aborts the process. We ship sensor-free MJCFs so the one asset
    loads under both mjlab (reads qpos/qvel from Entity data) and mujoco_ros2_control.
    """
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    assert model.nsensor == 0, f"{mjcf_path} has {model.nsensor} sensor(s); must be 0"


def test_add_actuators_emits_motors_and_no_sensors(tmp_path):
    xml_path = tmp_path / "robot.xml"
    xml_path.write_text(
        """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="base">
      <joint name="j1" actuatorfrcrange="-2 2"/>
      <geom name="g" type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
</mujoco>
""",
    )

    add_actuators(xml_path, {"j1": {"effort_limit": 5}})

    root = ET.parse(xml_path).getroot()
    assert root.find("sensor") is None
    motors = root.findall("./actuator/motor")
    assert [m.get("name") for m in motors] == ["j1"]
    assert motors[0].get("forcerange") == "-5 5"
