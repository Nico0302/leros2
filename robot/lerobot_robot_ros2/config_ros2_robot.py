# Copyright 2025 Nicolas Gres
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass, field
from typing import Literal, Never
from typing_extensions import Generic, TypeVar

from lerobot.cameras import CameraConfig

from lerobot.robots.config import RobotConfig

from lerobot_common_ros2 import JointConfig


@dataclass
class JointTrajectoryComponentConfig:
    """Configuration for a joint level control component."""

    type: Literal["joint_trajectory"]

    # Joints configuration
    joints: dict[str, JointConfig] = field(default_factory=dict)

    # ROS 2 topic to read the current joint states
    joint_state_topic: str = "joint_state"

    # ROS 2 topic to send the goal joint trajectory commands
    joint_trajectory_topic: str = "joint_trajectory_controller/joint_trajectory"


@dataclass
class GripperComponentConfig:
    """Configuration for a gripper control component."""

    type: Literal["gripper"]

    # Gripper action command topic
    gripper_cmd_topic: str = "gripper_controller/gripper_cmd"

    # Gripper joint configuration
    joint: JointConfig = field(default_factory=JointConfig)

    # ROS 2 topic to read the current gripper joint state
    joint_state_topic: str = "joint_state"


C = TypeVar("C", default=Never)


@RobotConfig.register_subclass("ros2_robot")
@dataclass
class ROS2RobotConfig(RobotConfig, Generic[C]):
    """Base configuration for a ROS 2 robot arm."""

    components: dict[
        str, JointTrajectoryComponentConfig | GripperComponentConfig | C
    ] = field(default_factory=dict)

    # Cameras configuration
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # If the robot should not send actions (e.g. joints get controlled by an external ROS 2 node)
    passive: bool = False

    # ROS 2 node name
    node_name: str = "lerobot_robot"
