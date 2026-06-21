import xml.etree.ElementTree as ET

from robot_assets.workflow.urdf_to_mjcf import replace_cylinders_with_capsules


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
