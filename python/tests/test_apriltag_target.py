import struct
import unittest
import xml.etree.ElementTree as element_tree
from pathlib import Path

PROJECT_ROOT = Path(__file__).parents[2]
TARGET_PATH = (
    PROJECT_ROOT
    / "assets"
    / "apriltag"
    / "tagStandard41h12-id0-90mm-a4.svg"
)
PNG_TARGET_PATH = (
    PROJECT_ROOT
    / "assets"
    / "apriltag"
    / "tagStandard41h12-id0-90mm-a4-300dpi.png"
)


class AprilTagTargetTests(unittest.TestCase):
    def test_printable_target_preserves_pose_span(self) -> None:
        root = element_tree.parse(TARGET_PATH).getroot()
        namespace = {"svg": "http://www.w3.org/2000/svg"}
        marker = root.find("svg:image", namespace)

        self.assertEqual(root.attrib["width"], "210mm")
        self.assertEqual(root.attrib["height"], "297mm")
        self.assertEqual(root.attrib["data-family"], "tagStandard41h12")
        self.assertEqual(root.attrib["data-id"], "0")
        self.assertEqual(root.attrib["data-total-cells"], "9")
        self.assertEqual(root.attrib["data-border-cells"], "5")
        self.assertEqual(root.attrib["data-tag-size-mm"], "90")
        self.assertIsNotNone(marker)

        rendered_width_mm = float(marker.attrib["width"])
        pose_span_mm = rendered_width_mm * 5.0 / 9.0
        self.assertAlmostEqual(pose_span_mm, 90.0)

    def test_png_target_is_a4_at_300_dpi(self) -> None:
        png = PNG_TARGET_PATH.read_bytes()
        self.assertEqual(png[:8], b"\x89PNG\r\n\x1a\n")

        width, height = struct.unpack(">II", png[16:24])
        self.assertEqual((width, height), (2480, 3508))

        phys_offset = png.index(b"pHYs") + 4
        pixels_per_metre_x, pixels_per_metre_y, unit = struct.unpack(
            ">IIB", png[phys_offset : phys_offset + 9]
        )
        self.assertEqual(unit, 1)
        self.assertEqual(
            (pixels_per_metre_x, pixels_per_metre_y),
            (11811, 11811),
        )


if __name__ == "__main__":
    unittest.main()
