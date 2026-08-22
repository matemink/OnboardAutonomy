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
SHOWCASE_WORLD = (
    PROJECT_ROOT
    / "simulation"
    / "worlds"
    / "apriltag_showcase.sdf"
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
WEATHER_DEFAULTS = (
    PROJECT_ROOT / "config" / "onboard_autonomy-gazebo-weather.parm"
)
WEATHER_APP_SCRIPT = (
    PROJECT_ROOT
    / "scripts"
    / "run_onboard_autonomy_gazebo_weather_vision.sh"
)
WEATHER_GAZEBO_SCRIPT = (
    PROJECT_ROOT / "scripts" / "run_gazebo_apriltag_weather.sh"
)
WEATHER_PROFILE_LOADER = (
    PROJECT_ROOT / "scripts" / "weather_profile.sh"
)
GAZEBO_GUI_CONFIG = (
    PROJECT_ROOT / "simulation" / "gui" / "onboard_autonomy.config"
)
WIND_INDICATOR_QML = (
    PROJECT_ROOT / "simulation" / "gui" / "WindIndicator.qml"
)
GAZEBO_GUI_LAUNCHER = PROJECT_ROOT / "scripts" / "run_gazebo_gui.sh"
TARGET_MODEL = (
    PROJECT_ROOT
    / "simulation"
    / "models"
    / "scripted_fixed_wing_target"
    / "model.sdf"
)
TARGET_SPAWNER = (
    PROJECT_ROOT / "scripts" / "spawn_gazebo_aerial_target.sh"
)
TARGET_LAUNCHER = (
    PROJECT_ROOT / "StartOnboardAutonomyAerialTracking.cmd"
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

    def test_showcase_world_is_opt_in_and_starts_clean(self) -> None:
        deterministic = element_tree.parse(WORLD).getroot().find("world")
        showcase = element_tree.parse(SHOWCASE_WORLD).getroot().find("world")
        self.assertIsNotNone(deterministic)
        self.assertIsNotNone(showcase)
        self.assertEqual(showcase.attrib["name"], deterministic.attrib["name"])

        self.assertIsNone(deterministic.find("model[@name='showcase_hangar']"))
        self.assertIsNone(showcase.find("model[@name='showcase_probe_box']"))
        self.assertIsNone(showcase.find("model[@name='showcase_hangar']"))

        deterministic_uris = {
            include.findtext("uri") for include in deterministic.findall("include")
        }
        showcase_uris = {
            include.findtext("uri") for include in showcase.findall("include")
        }
        decorative_uris = {
            (
                "https://fuel.gazebosim.org/1.0/OpenRobotics/models/"
                "Oak%20tree"
            ),
            (
                "https://fuel.gazebosim.org/1.0/OpenRobotics/models/"
                "FoodCourtBenchShort"
            ),
            (
                "https://fuel.gazebosim.org/1.0/OpenRobotics/models/"
                "Hatchback%20red"
            ),
        }
        self.assertEqual(
            showcase_uris,
            deterministic_uris | decorative_uris,
        )

        demo_launcher = (
            PROJECT_ROOT / "StartOnboardAutonomyGazeboDemo.cmd"
        ).read_text(encoding="utf-8")
        showcase_launcher = (
            PROJECT_ROOT / "StartOnboardAutonomyGazeboShowcase.cmd"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "simulation/worlds/apriltag_landing.sdf",
            demo_launcher,
        )
        self.assertIn(
            'ONBOARD_AUTONOMY_GAZEBO_WORLD="%GAZEBO_WORLD%"',
            demo_launcher,
        )
        self.assertIn(
            "simulation/worlds/apriltag_showcase.sdf",
            showcase_launcher,
        )

    def test_scripted_aerial_target_has_a_deterministic_route(self) -> None:
        model = element_tree.parse(TARGET_MODEL).getroot().find("model")
        self.assertIsNotNone(model)
        self.assertEqual(model.attrib["name"], "scripted_fixed_wing_target")

        pose = [float(value) for value in model.findtext("pose").split()]
        self.assertEqual(pose[2], 8.0)
        self.assertEqual(model.findtext("link/gravity"), "false")

        plugin = model.find("plugin")
        self.assertIsNotNone(plugin)
        self.assertEqual(
            plugin.attrib["filename"],
            "gz-sim-velocity-control-system",
        )
        linear = [
            float(value) for value in plugin.findtext("initial_linear").split()
        ]
        angular = [
            float(value) for value in plugin.findtext("initial_angular").split()
        ]
        self.assertEqual(linear, [2.0, 0.0, 0.0])
        self.assertEqual(angular[:2], [0.0, 0.0])
        self.assertAlmostEqual(linear[0] / angular[2], 12.0)

    def test_aerial_target_is_opt_in_and_uses_gazebo_create(self) -> None:
        world = element_tree.parse(WORLD).getroot()
        base_uris = {
            include.findtext("uri")
            for include in world.findall(".//world/include")
        }
        self.assertNotIn("model://scripted_fixed_wing_target", base_uris)

        spawner = TARGET_SPAWNER.read_text(encoding="utf-8")
        launcher = TARGET_LAUNCHER.read_text(encoding="utf-8")
        self.assertIn("/world/${world_name}/create", spawner)
        self.assertIn("gz.msgs.EntityFactory", spawner)
        self.assertIn('readonly target_name="Generic_Fixed_Wing_Target"', spawner)
        self.assertIn("StartOnboardAutonomyGazeboDemo.cmd", launcher)
        self.assertIn("spawn_gazebo_aerial_target.sh", launcher)

    def test_windows_launcher_cleans_up_only_the_demo_before_start(self) -> None:
        demo_launcher = (
            PROJECT_ROOT / "StartOnboardAutonomyGazeboDemo.cmd"
        ).read_text(encoding="utf-8")
        stop_launcher = (
            PROJECT_ROOT / "StopOnboardAutonomyGazeboDemo.cmd"
        ).read_text(encoding="utf-8")
        cleanup_script = (
            PROJECT_ROOT / "scripts" / "stop_onboard_autonomy_gazebo.sh"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "bash scripts/stop_onboard_autonomy_gazebo.sh",
            demo_launcher,
        )
        self.assertIn(
            "bash scripts/stop_onboard_autonomy_gazebo.sh",
            stop_launcher,
        )
        self.assertNotIn("wsl --shutdown", demo_launcher)
        self.assertIn("pkill", cleanup_script)
        self.assertIn("arducopter", cleanup_script)
        self.assertIn("onboard_autonomy", cleanup_script)
        self.assertIn("gz", cleanup_script)

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

    def test_weather_system_is_opt_in_and_gust_capable(self) -> None:
        world = element_tree.parse(WORLD).getroot().find("world")
        self.assertIsNotNone(world)

        plugins = {
            plugin.attrib["name"]: plugin
            for plugin in world.findall("plugin")
        }
        wind = plugins["gz::sim::systems::WindEffects"]

        self.assertEqual(
            world.findtext("wind/linear_velocity"),
            "0 0 0",
        )
        self.assertIn("gz::sim::systems::AirPressure", plugins)
        self.assertEqual(
            wind.findtext("horizontal/magnitude/sin/amplitude_percent"),
            "0.15",
        )
        self.assertEqual(
            wind.findtext("horizontal/direction/sin/amplitude"),
            "8",
        )
        self.assertIsNotNone(wind.find("vertical/noise"))

    def test_pixhawk_exposes_air_pressure_telemetry(self) -> None:
        model = element_tree.parse(CAMERA_MODEL).getroot()
        self.assertEqual(model.findtext("model/enable_wind"), "true")

        sensor = model.find(
            ".//sensor[@name='Pixhawk_6C_barometer']"
        )
        self.assertIsNotNone(sensor)
        self.assertEqual(sensor.attrib["type"], "air_pressure")
        self.assertEqual(sensor.findtext("update_rate"), "50")
        self.assertEqual(
            sensor.findtext("topic"),
            "/onboard_autonomy/sensors/pixhawk_6c/air_pressure",
        )
        self.assertEqual(
            sensor.findtext("air_pressure/reference_altitude"),
            "584",
        )

    def test_sitl_weather_matches_the_gazebo_wind_seed(self) -> None:
        parameters = {}
        for line in WEATHER_DEFAULTS.read_text(encoding="utf-8").splitlines():
            content = line.partition("#")[0].strip()
            if not content:
                continue
            name, value = content.split(maxsplit=1)
            parameters[name] = value

        self.assertEqual(parameters["SIM_WIND_SPD"], "3")
        self.assertEqual(parameters["SIM_WIND_DIR"], "270")
        self.assertGreater(float(parameters["SIM_WIND_TURB"]), 0.0)
        self.assertEqual(parameters["SIM_WIND_T"], "1")
        self.assertEqual(parameters["SIM_BARO_RND"], "0.2")

        for script in (WEATHER_APP_SCRIPT, WEATHER_GAZEBO_SCRIPT):
            self.assertIn(
                'source "${script_dir}/weather_profile.sh"',
                script.read_text(encoding="utf-8"),
            )

        loader = WEATHER_PROFILE_LOADER.read_text(encoding="utf-8")
        for parameter in (
            "SIM_WIND_SPD",
            "SIM_WIND_DIR",
            "SIM_WIND_TURB",
        ):
            self.assertIn(parameter, loader)

        ardupilot_launcher = (
            PROJECT_ROOT / "scripts" / "run_arducopter_gazebo_weather.sh"
        ).read_text(encoding="utf-8")
        self.assertIn(
            'source "${script_dir}/weather_profile.sh"',
            ardupilot_launcher,
        )
        self.assertIn(
            'ONBOARD_AUTONOMY_SITL_WEATHER_DEFAULTS="${weather_profile_file}"',
            ardupilot_launcher,
        )

        windows_launcher = (
            PROJECT_ROOT / "StartOnboardAutonomyGazeboDemo.cmd"
        ).read_text(encoding="utf-8")
        self.assertIn(
            'if not "%~1"=="" set "WEATHER_PROFILE=%~1"',
            windows_launcher,
        )
        self.assertEqual(
            windows_launcher.count("ONBOARD_AUTONOMY_WEATHER_PROFILE="),
            4,
        )

    def test_gazebo_gui_owns_the_weather_indicator(self) -> None:
        config_text = GAZEBO_GUI_CONFIG.read_text(encoding="utf-8")
        config_text = config_text.replace(
            '<?xml version="1.0"?>',
            "",
            1,
        )
        config = element_tree.fromstring(f"<config>{config_text}</config>")
        plugins = {
            plugin.attrib["filename"]: plugin
            for plugin in config.findall("plugin")
        }

        self.assertIn("MinimalScene", plugins)
        self.assertIn("WindIndicator", plugins)
        self.assertTrue(
            {
                "ComponentInspector",
                "CopyPaste",
                "EntityTree",
                "Lights",
                "Screenshot",
                "Shapes",
                "Spawn",
                "TransformControl",
                "VisualizationCapabilities",
            }.issubset(plugins),
            "The custom GUI config must retain Gazebo's standard panels",
        )

        for side_panel in ("ComponentInspector", "EntityTree"):
            properties = plugins[side_panel].find("gz-gui").findall(
                "property"
            )
            state = next(
                item.text for item in properties if item.attrib["key"] == "state"
            )
            self.assertEqual("docked", state)

        wind_gui = plugins["WindIndicator"].find("gz-gui")
        self.assertIsNotNone(wind_gui)
        self.assertEqual(
            wind_gui.find("anchors").attrib["target"],
            "3D View",
        )

        qml = WIND_INDICATOR_QML.read_text(encoding="utf-8")
        self.assertIn("WindIndicator.speedMetersPerSecond", qml)
        self.assertIn("WindIndicator.directionFromDegrees", qml)
        self.assertIn("WindIndicator.directionLabel", qml)
        for cardinal in ('text: "N"', 'text: "E"', 'text: "S"', 'text: "W"'):
            self.assertIn(cardinal, qml)

        launcher = GAZEBO_GUI_LAUNCHER.read_text(encoding="utf-8")
        self.assertIn("build_gazebo_gui_plugins.sh", launcher)
        self.assertIn('GZ_GUI_PLUGIN_PATH=', launcher)
        self.assertIn('--gui-config "${gui_config}"', launcher)


if __name__ == "__main__":
    unittest.main()
