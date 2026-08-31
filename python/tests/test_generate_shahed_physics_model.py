import importlib.util
import tempfile
import unittest
import xml.etree.ElementTree as element_tree
from pathlib import Path

PROJECT_ROOT = Path(__file__).parents[2]
GENERATOR_PATH = (
    PROJECT_ROOT / "scripts" / "generate_shahed_136_physics_model.py"
)


def load_generator():
    spec = importlib.util.spec_from_file_location("shahed_generator", GENERATOR_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("Could not load the Shahed model generator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


UPSTREAM_FIXTURE = """<?xml version="1.0"?>
<sdf version="1.7">
  <model name="skywalker_x8">
    <pose>0 0 0.246 0 0 0</pose>
    <link name="base_link">
      <inertial><mass>4.5</mass><inertia>
        <ixx>0.45</ixx><ixy>0</ixy><ixz>0.06</ixz>
        <iyy>0.325</iyy><iyz>0</iyz><izz>0.75</izz>
      </inertia></inertial>
      <collision><pose>0 0 -0.07 0 0 0</pose>
        <geometry><box><size>0.8 2.15 0.2</size></box></geometry>
      </collision>
      <visual name="x8"><geometry><mesh><uri>x8.dae</uri></mesh></geometry></visual>
    </link>
    <link name="rotor_pusher">
      <pose>-0.385 0 0 0 90 0</pose>
      <inertial><mass>0.025</mass></inertial>
      <collision><geometry><cylinder>
        <length>0.005</length><radius>0.065</radius>
      </cylinder></geometry></collision>
      <visual name="prop" />
    </link>
    <link name="left_elevon"><inertial><mass>0.1</mass></inertial></link>
    <link name="right_elevon"><inertial><mass>0.1</mass></inertial></link>
    <link name="imu_link"><inertial><mass>0.15</mass></inertial></link>
    <joint name="left_elevon_joint"><pose>0 0 0 0 0 0</pose></joint>
    <joint name="right_elevon_joint"><pose>0 0 0 0 0 0</pose></joint>
    <joint name="skywalker_x8/imu_joint" />
    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <fdm_port_in>9002</fdm_port_in>
      <control channel="2"><multiplier>838</multiplier></control>
    </plugin>
  </model>
</sdf>
"""


class ShahedPhysicsModelGeneratorTests(unittest.TestCase):
    def test_generates_isolated_physics_backed_target(self) -> None:
        generator = load_generator()

        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            source = temp_path / "x8.sdf"
            output = temp_path / "shahed"
            source.write_text(UPSTREAM_FIXTURE, encoding="utf-8")

            generator.generate(source, output)

            model = element_tree.parse(output / "model.sdf").getroot().find(
                "model"
            )
            self.assertIsNotNone(model)
            self.assertEqual(model.attrib["name"], "shahed_136_arduplane")
            self.assertEqual(model.findtext("enable_wind"), "true")
            self.assertEqual(
                model.findtext("plugin[@name='ArduPilotPlugin']/fdm_port_in"),
                "9012",
            )
            self.assertEqual(
                sum(float(item.text) for item in model.findall("link/inertial/mass")),
                200.0,
            )
            self.assertEqual(
                model.findtext(
                    "link[@name='base_link']/visual/geometry/mesh/uri"
                ),
                generator.VISUAL_URI,
            )
            self.assertIsNone(model.find("plugin[@name='VelocityControl']"))
            self.assertTrue((output / "PROVENANCE.md").is_file())


if __name__ == "__main__":
    unittest.main()
