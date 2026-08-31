#!/usr/bin/env python3

import argparse
import xml.etree.ElementTree as element_tree
from pathlib import Path

MODEL_NAME = "shahed_136_arduplane"
FDM_PORT = "9012"
REFERENCE_LENGTH_M = 3.5
REFERENCE_SPAN_M = 2.5
REFERENCE_HEIGHT_M = 0.5
REFERENCE_MASS_KG = 200.0
REFERENCE_CRUISE_SPEED_M_S = 50.0
# The retained X8 control and sensor links contribute 0.375 kg.
BASE_LINK_MASS_KG = 199.625
WING_AREA_M2 = 3.2
ELEVON_AREA_M2 = 0.18
WINGLET_AREA_M2 = 0.12
PROPELLER_AREA_M2 = 0.16
PROPELLER_RADIUS_M = 0.3
PROPELLER_X_M = -1.65

VISUAL_URI = (
    "model://scripted_fixed_wing_target/meshes/shahed_136/scene.gltf"
)
VISUAL_POSE = "-1.328877220 0 -0.193101322 0 0 1.5707963267948966"
VISUAL_SCALE = "1.422263684 1.356554469 1.453943853"


def require_element(
    parent: element_tree.Element,
    path: str,
) -> element_tree.Element:
    element = parent.find(path)
    if element is None:
        raise ValueError(f"Required upstream SDF element is missing: {path}")
    return element


def set_text(parent: element_tree.Element, path: str, value: object) -> None:
    require_element(parent, path).text = str(value)


def remove_visuals(model: element_tree.Element) -> None:
    for link in model.findall("link"):
        for visual in link.findall("visual"):
            link.remove(visual)


def add_shahed_visual(base_link: element_tree.Element) -> None:
    visual = element_tree.SubElement(
        base_link,
        "visual",
        {"name": "shahed_136_airframe"},
    )
    element_tree.SubElement(visual, "pose").text = VISUAL_POSE
    geometry = element_tree.SubElement(visual, "geometry")
    mesh = element_tree.SubElement(geometry, "mesh")
    element_tree.SubElement(mesh, "uri").text = VISUAL_URI
    element_tree.SubElement(mesh, "scale").text = VISUAL_SCALE


def configure_base_link(model: element_tree.Element) -> None:
    base_link = require_element(model, "link[@name='base_link']")
    set_text(base_link, "inertial/mass", BASE_LINK_MASS_KG)
    set_text(base_link, "inertial/inertia/ixx", "108.333333")
    set_text(base_link, "inertial/inertia/ixy", 0)
    set_text(base_link, "inertial/inertia/ixz", 0)
    set_text(base_link, "inertial/inertia/iyy", "208.333333")
    set_text(base_link, "inertial/inertia/iyz", 0)
    set_text(base_link, "inertial/inertia/izz", "308.333333")
    set_text(base_link, "collision/pose", "0 0 0 0 0 0")
    set_text(
        base_link,
        "collision/geometry/box/size",
        f"{REFERENCE_LENGTH_M} {REFERENCE_SPAN_M} {REFERENCE_HEIGHT_M}",
    )

    gravity = base_link.find("gravity")
    if gravity is None:
        gravity = element_tree.SubElement(base_link, "gravity")
    gravity.text = "true"
    add_shahed_visual(base_link)


def configure_control_geometry(model: element_tree.Element) -> None:
    rotor = require_element(model, "link[@name='rotor_pusher']")
    set_text(rotor, "pose", f"{PROPELLER_X_M} 0 0 0 90 0")
    set_text(
        rotor,
        "collision/geometry/cylinder/radius",
        PROPELLER_RADIUS_M,
    )
    set_text(rotor, "collision/geometry/cylinder/length", 0.05)

    left_joint = require_element(model, "joint[@name='left_elevon_joint']")
    right_joint = require_element(model, "joint[@name='right_elevon_joint']")
    set_text(left_joint, "pose", "-1.05 0.72 0 0 0 0.265")
    set_text(right_joint, "pose", "-1.05 -0.72 0 0 0 -0.265")

    imu_joint = require_element(model, "joint[@name='skywalker_x8/imu_joint']")
    imu_joint.attrib["name"] = "shahed_136/imu_joint"


def configure_aerodynamics(model: element_tree.Element) -> None:
    for plugin in model.findall("plugin[@filename='gz-sim-lift-drag-system']"):
        link_name = plugin.findtext("link_name")
        control_joint = plugin.findtext("control_joint_name")
        upward = plugin.findtext("upward")

        if link_name == "rotor_pusher":
            set_text(plugin, "area", PROPELLER_AREA_M2)
            continue
        if control_joint == "left_elevon_joint":
            set_text(plugin, "area", ELEVON_AREA_M2)
            set_text(plugin, "cp", "-1.05 0.72 0")
            continue
        if control_joint == "right_elevon_joint":
            set_text(plugin, "area", ELEVON_AREA_M2)
            set_text(plugin, "cp", "-1.05 -0.72 0")
            continue
        if link_name == "base_link" and upward == "0 0 1":
            set_text(plugin, "area", WING_AREA_M2)
            set_text(plugin, "cp", "-0.25 0 0")
            continue
        if link_name == "base_link" and upward == "0 1 0":
            set_text(plugin, "area", WINGLET_AREA_M2)
            set_text(plugin, "cp", "-0.95 1.15 0.12")
            continue
        if link_name == "base_link" and upward == "0 -1 0":
            set_text(plugin, "area", WINGLET_AREA_M2)
            set_text(plugin, "cp", "-0.95 -1.15 0.12")


def configure_ardupilot(model: element_tree.Element) -> None:
    plugin = require_element(model, "plugin[@name='ArduPilotPlugin']")
    set_text(plugin, "fdm_port_in", FDM_PORT)
    throttle = require_element(plugin, "control[@channel='2']")
    set_text(throttle, "multiplier", 360)


def configure_model(tree: element_tree.ElementTree) -> None:
    root = tree.getroot()
    model = require_element(root, "model")
    model.attrib["name"] = MODEL_NAME
    set_text(model, "pose", "0 0 0 0 0 0")

    enable_wind = model.find("enable_wind")
    if enable_wind is None:
        enable_wind = element_tree.Element("enable_wind")
        model.insert(1, enable_wind)
    enable_wind.text = "true"

    remove_visuals(model)
    configure_base_link(model)
    configure_control_geometry(model)
    configure_aerodynamics(model)
    configure_ardupilot(model)


def write_model_config(output_dir: Path) -> None:
    config = """<?xml version="1.0"?>
<model>
  <name>Shahed-136 physics-backed ArduPlane approximation</name>
  <version>0.1</version>
  <sdf version="1.7">model.sdf</sdf>
  <author><name>OnboardAutonomy</name></author>
  <description>
    Simulation-only physics approximation generated from pinned ArduPilot
    Skywalker X8 dynamics and the attributed Shahed-136 visual.
  </description>
</model>
"""
    (output_dir / "model.config").write_text(config, encoding="utf-8")


def write_provenance(output_dir: Path, source_model: Path) -> None:
    notice = f"""# Generated physics model

This directory is generated and must remain outside Git.

- Upstream dynamics template: `{source_model}`
- Published reference envelope: {REFERENCE_LENGTH_M} x {REFERENCE_SPAN_M} x {REFERENCE_HEIGHT_M} m
- Published reference mass: {REFERENCE_MASS_KG} kg
- Published cruise-speed target: {REFERENCE_CRUISE_SPEED_M_S} m/s
- FDM port: {FDM_PORT}

The lift, drag, inertia distribution, control effectiveness, and propeller
parameters are engineering approximations derived from the pinned Skywalker X8
simulation model. They are not measured Shahed-136 aerodynamic data.
"""
    (output_dir / "PROVENANCE.md").write_text(notice, encoding="utf-8")


def generate(source_model: Path, output_dir: Path) -> None:
    tree = element_tree.parse(source_model)
    configure_model(tree)

    output_dir.mkdir(parents=True, exist_ok=True)
    output_model = output_dir / "model.sdf"
    element_tree.indent(tree, space="  ")
    tree.write(output_model, encoding="utf-8", xml_declaration=True)
    write_model_config(output_dir)
    write_provenance(output_dir, source_model)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    generate(args.source, args.output)


if __name__ == "__main__":
    main()
