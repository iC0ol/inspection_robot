# Copyright 2026 inspection_robot contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the inspection robot URDF/Xacro description."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


PACKAGE_DIR = Path(__file__).resolve().parents[1]
XACRO_FILE = PACKAGE_DIR / "urdf" / "inspection_robot.urdf.xacro"


def generate_urdf_root():
    """Expand the main Xacro file and return the parsed URDF XML root."""
    result = subprocess.run(
        ["xacro", str(XACRO_FILE)],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def test_xacro_can_be_expanded():
    """The main Xacro file must expand into a valid robot XML document."""
    root = generate_urdf_root()
    assert root.tag == "robot"
    assert root.attrib["name"] == "inspection_robot"


def test_required_links_exist():
    """All frames required by the current robot architecture must exist."""
    root = generate_urdf_root()
    link_names = {
        link.attrib["name"]
        for link in root.findall("link")
    }
    required_links = {
        "base_footprint",
        "base_link",
        "left_wheel_link",
        "right_wheel_link",
        "laser_link",
        "imu_link",
    }
    assert required_links.issubset(link_names)


def test_required_joints_exist():
    """All expected robot joints must exist."""
    root = generate_urdf_root()

    joint_names = {
        joint.attrib["name"]
        for joint in root.findall("joint")
    }

    required_joints = {
        "base_footprint_joint",
        "left_wheel_joint",
        "right_wheel_joint",
        "laser_joint",
        "imu_joint",
    }

    assert required_joints.issubset(joint_names)


def test_sensor_frames_are_attached_to_base_link():
    """Laser and IMU frames must both be fixed children of base_link."""
    root = generate_urdf_root()

    expected_children = {
        "laser_joint": "laser_link",
        "imu_joint": "imu_link",
    }

    for joint_name, expected_child in expected_children.items():
        joint = root.find(f"./joint[@name='{joint_name}']")

        assert joint is not None
        assert joint.attrib["type"] == "fixed"

        parent = joint.find("parent")
        child = joint.find("child")

        assert parent.attrib["link"] == "base_link"
        assert child.attrib["link"] == expected_child


def test_wheel_joint_types():
    """Drive wheels must use continuous joints."""
    root = generate_urdf_root()

    for joint_name in (
        "left_wheel_joint",
        "right_wheel_joint",
    ):
        joint = root.find(f"./joint[@name='{joint_name}']")

        assert joint is not None
        assert joint.attrib["type"] == "continuous"


def test_ros2_control_wheel_interfaces():
    """Drive wheels must expose the interfaces required by diff drive."""
    root = generate_urdf_root()

    ros2_control = root.find(
        "./ros2_control[@name='InspectionRobotSystem']"
    )

    assert ros2_control is not None
    assert ros2_control.attrib["type"] == "system"

    hardware_plugin = ros2_control.find("./hardware/plugin")
    assert hardware_plugin is not None
    assert hardware_plugin.text == "mock_components/GenericSystem"

    for joint_name in (
        "left_wheel_joint",
        "right_wheel_joint",
    ):
        joint = ros2_control.find(
            f"./joint[@name='{joint_name}']"
        )

        assert joint is not None

        command_interfaces = {
            interface.attrib["name"]
            for interface in joint.findall("command_interface")
        }

        state_interfaces = {
            interface.attrib["name"]
            for interface in joint.findall("state_interface")
        }

        assert command_interfaces == {"velocity"}
        assert {"position", "velocity"}.issubset(state_interfaces)
