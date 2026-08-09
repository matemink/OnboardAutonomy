import json
import math
import unittest
import xml.etree.ElementTree as element_tree
from pathlib import Path

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).parents[2]
CAMERA_MODEL = (
    PROJECT_ROOT
    / "simulation"
    / "models"
    / "iris_with_landing_camera"
    / "model.sdf"
)
PAD_MODEL = (
    PROJECT_ROOT
    / "simulation"
    / "models"
    / "apriltag_landing_pad"
    / "model.sdf"
)
WORLD = (
    PROJECT_ROOT
    / "simulation"
    / "worlds"
    / "apriltag_landing.sdf"
)
CALIBRATION = (
    PROJECT_ROOT / "config" / "gazebo-landing-camera-640x480.json"
)
SOURCE_TAG = (
    PROJECT_ROOT
    / "assets"
    / "apriltag"
    / "tagStandard41h12-id0.png"
)
TEXTURE = (
    PROJECT_ROOT
    / "simulation"
    / "models"
    / "apriltag_landing_pad"
    / "materials"
    / "textures"
    / "tagStandard41h12-id0-quiet-zone.png"
)


class GazeboAprilTagWorldTests(unittest.TestCase):
    def test_analytic_calibration_matches_camera_sdf(self) -> None:
        model = element_tree.parse(CAMERA_MODEL).getroot()
        sensor = model.find(
            ".//sensor[@name='Raspberry_Pi_Camera_Module_3_Wide']"
        )
        self.assertIsNotNone(sensor)

        width = int(sensor.findtext("camera/image/width"))
        height = int(sensor.findtext("camera/image/height"))
        horizontal_fov = float(sensor.findtext("camera/horizontal_fov"))
        update_rate = float(sensor.findtext("update_rate"))
        plugin = sensor.find("plugin")
        self.assertIsNotNone(plugin)
        udp_port = int(sensor.findtext("plugin/udp_port"))

        document = json.loads(CALIBRATION.read_text(encoding="utf-8"))
        expected_focal_length = width / (2.0 * math.tan(horizontal_fov / 2.0))

        self.assertEqual((width, height), (640, 480))
        self.assertEqual(update_rate, 30.0)
        self.assertEqual(plugin.attrib["name"], "GstCameraPlugin")
        self.assertEqual(plugin.attrib["filename"], "GstCameraPlugin")
        self.assertEqual(udp_port, 5601)
        self.assertEqual(document["result"], "PASS")
        self.assertEqual(document["camera"]["width"], width)
        self.assertEqual(document["camera"]["height"], height)
        self.assertAlmostEqual(
            document["intrinsics"]["fx_px"],
            expected_focal_length,
        )
        self.assertAlmostEqual(
            document["intrinsics"]["fy_px"],
            expected_focal_length,
        )
        self.assertEqual(document["distortion"]["coefficients"], [0.0] * 5)

    def test_pad_geometry_has_a_two_metre_detection_span(self) -> None:
        model = element_tree.parse(PAD_MODEL).getroot()
        plane_size = model.findtext(".//visual/geometry/plane/size")
        width_m, height_m = (float(value) for value in plane_size.split())

        self.assertEqual((width_m, height_m), (4.4, 4.4))
        self.assertAlmostEqual(width_m * 5.0 / 11.0, 2.0)

    def test_texture_preserves_the_pinned_tag_and_quiet_zone(self) -> None:
        source = cv2.imread(str(SOURCE_TAG), cv2.IMREAD_GRAYSCALE)
        texture = cv2.imread(str(TEXTURE), cv2.IMREAD_GRAYSCALE)

        self.assertIsNotNone(source)
        self.assertIsNotNone(texture)
        self.assertEqual(source.shape, (9, 9))
        self.assertEqual(texture.shape, (1100, 1100))

        for cell_y in range(11):
            for cell_x in range(11):
                expected = 255
                if 0 < cell_x < 10 and 0 < cell_y < 10:
                    expected = int(source[cell_y - 1, cell_x - 1])
                block = texture[
                    cell_y * 100 : (cell_y + 1) * 100,
                    cell_x * 100 : (cell_x + 1) * 100,
                ]
                self.assertTrue(np.all(block == expected))

    def test_world_uses_project_camera_and_pad_models(self) -> None:
        world = element_tree.parse(WORLD).getroot()
        includes = {
            include.findtext("uri"): include
            for include in world.findall(".//world/include")
        }
        self.assertEqual(
            set(includes),
            {
                "model://apriltag_landing_pad",
                "model://iris_with_landing_camera",
            },
        )

        pad_pose = [
            float(value)
            for value in includes[
                "model://apriltag_landing_pad"
            ].findtext("pose").split()
        ]
        vehicle_pose = [
            float(value)
            for value in includes[
                "model://iris_with_landing_camera"
            ].findtext("pose").split()
        ]
        separation_m = math.hypot(
            pad_pose[0] - vehicle_pose[0],
            pad_pose[1] - vehicle_pose[1],
        )

        self.assertAlmostEqual(separation_m, 3.0)
        self.assertEqual(
            includes[
                "model://iris_with_landing_camera"
            ].findtext("name"),
            "Holybro_S500",
        )

    def test_vehicle_exposes_the_real_development_rig_names(self) -> None:
        model = element_tree.parse(CAMERA_MODEL).getroot()
        component_links = {
            link.attrib["name"]
            for link in model.findall(".//model/link")
        }

        self.assertTrue(
            {
                "Pixhawk_6C",
                "Raspberry_Pi_5",
                "Raspberry_Pi_Camera_Module_3_Wide",
            }.issubset(component_links)
        )


if __name__ == "__main__":
    unittest.main()
