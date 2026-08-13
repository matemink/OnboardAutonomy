import importlib.util
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2 as cv
import numpy as np

MODULE_PATH = Path(__file__).parents[1] / "calibrate_camera.py"
SPEC = importlib.util.spec_from_file_location("calibrate_camera", MODULE_PATH)
assert SPEC is not None
assert SPEC.loader is not None
CALIBRATE_CAMERA = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = CALIBRATE_CAMERA
SPEC.loader.exec_module(CALIBRATE_CAMERA)


class CameraCalibrationTests(unittest.TestCase):
    def test_printable_target_has_expected_checkerboard_geometry(self) -> None:
        target = (
            Path(__file__).parents[2]
            / "assets/calibration/checkerboard-9x6-25mm-a4.svg"
        )
        root = ET.parse(target).getroot()
        namespace = {"svg": "http://www.w3.org/2000/svg"}
        board_group = root.find(
            "svg:g[@transform='translate(23.5 8)']",
            namespace,
        )
        self.assertIsNotNone(board_group)
        assert board_group is not None
        squares = {
            (float(rect.attrib["x"]), float(rect.attrib["y"]))
            for rect in board_group.findall("svg:rect", namespace)
            if rect.attrib["width"] == "25"
            and rect.attrib["height"] == "25"
        }
        expected = {
            (float(column * 25), float(row * 25))
            for row in range(7)
            for column in range(10)
            if (column + row) % 2 == 0
        }
        self.assertEqual(squares, expected)

    def test_pattern_parser_rejects_invalid_dimensions(self) -> None:
        self.assertEqual(CALIBRATE_CAMERA.parse_pattern_size("9x6"), (9, 6))
        self.assertEqual(CALIBRATE_CAMERA.parse_pattern_size("7*5"), (7, 5))
        with self.assertRaises(ValueError):
            CALIBRATE_CAMERA.parse_pattern_size("6")
        with self.assertRaises(ValueError):
            CALIBRATE_CAMERA.parse_pattern_size("1x6")

    def test_synthetic_views_recover_camera_intrinsics(self) -> None:
        image_size = (640, 480)
        expected_matrix = np.array(
            [[800.0, 0.0, 320.0], [0.0, 805.0, 240.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        expected_distortion = np.array(
            [-0.08, 0.02, 0.001, -0.0005, 0.0],
            dtype=np.float64,
        )
        template = CALIBRATE_CAMERA.checkerboard_object_points(
            (9, 6),
            0.025,
        )
        object_points = []
        image_points = []
        for index in range(14):
            rotation = np.array(
                [
                    -0.18 + index * 0.025,
                    0.12 - index * 0.014,
                    -0.08 + index * 0.011,
                ],
                dtype=np.float64,
            )
            translation = np.array(
                [
                    -0.08 + (index % 5) * 0.035,
                    -0.05 + (index % 4) * 0.025,
                    0.65 + index * 0.025,
                ],
                dtype=np.float64,
            )
            projected, _ = cv.projectPoints(
                template,
                rotation,
                translation,
                expected_matrix,
                expected_distortion,
            )
            object_points.append(template.copy())
            image_points.append(projected.astype(np.float32))

        solution = CALIBRATE_CAMERA.calibrate_points(
            object_points,
            image_points,
            image_size,
        )

        self.assertLess(solution.rms_error_px, 0.001)
        self.assertAlmostEqual(solution.camera_matrix[0, 0], 800.0, delta=2.0)
        self.assertAlmostEqual(solution.camera_matrix[1, 1], 805.0, delta=2.0)
        self.assertAlmostEqual(solution.camera_matrix[0, 2], 320.0, delta=1.0)
        self.assertAlmostEqual(solution.camera_matrix[1, 2], 240.0, delta=1.0)

    def test_document_records_inputs_and_quality_gate(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            image = Path(temporary_directory) / "view001.jpg"
            image.write_bytes(b"calibration-view")
            views = CALIBRATE_CAMERA.CalibrationViews(
                image_size=(640, 480),
                object_points=[],
                image_points=[],
                accepted=[image],
                rejected=[{"file": "bad.jpg", "reason": "not found"}],
            )
            solution = CALIBRATE_CAMERA.CalibrationSolution(
                camera_matrix=np.array(
                    [[500.0, 0.0, 320.0], [0.0, 501.0, 240.0], [0.0, 0.0, 1.0]],
                    dtype=np.float64,
                ),
                distortion_coefficients=np.zeros(5, dtype=np.float64),
                rotation_vectors=[],
                translation_vectors=[],
                rms_error_px=0.25,
                per_view_rms_error_px=[0.3],
            )
            document = CALIBRATE_CAMERA.build_document(
                camera_model="imx708_wide",
                focus_mode="manual",
                lens_position="default",
                pattern_size=(9, 6),
                square_size_m=0.025,
                minimum_views=1,
                maximum_rms_error_px=1.0,
                maximum_view_error_px=1.5,
                views=views,
                solution=solution,
            )

            self.assertEqual(document["schema_version"], 1)
            self.assertEqual(document["result"], "PASS")
            self.assertEqual(document["camera"]["width"], 640)
            self.assertEqual(document["camera"]["focus_mode"], "manual")
            self.assertEqual(document["camera"]["lens_position"], "default")
            self.assertEqual(document["intrinsics"]["fx_px"], 500.0)
            self.assertEqual(
                document["distortion"]["coefficient_order"],
                ["k1", "k2", "p1", "p2", "k3"],
            )
            self.assertEqual(document["views"]["provided"], 2)
            self.assertEqual(len(document["views"]["accepted"][0]["sha256"]), 64)


if __name__ == "__main__":
    unittest.main()
