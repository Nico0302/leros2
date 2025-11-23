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

from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot_common_ros2 import JointConfig


@dataclass
class JointTrajectoryControllerStateConfig:
    """
    Configuration for the Joint Trajectory Controller State component.

    Reads the desired joint states in a MoveIt cartesian teleoperation setup.
    """

    type: Literal["joint_trajectory_controller_state"]

    # Joints configuration
    joints: dict[str, JointConfig] = field(default_factory=dict)

    # ROS 2 topic to read the joint controller state
    controller_state_topic: str = "joint_trajectory_controller/controller_state"


@dataclass
class JointStateConfig:
    """
    Configuration for the Joint State component.

    Reads the current joint state from the joint state broadcaster.
    """

    type: Literal["joint_state"]

    # Joints configuration
    joints: dict[str, JointConfig] = field(default_factory=dict)

    # ROS 2 topic to read the joint state
    joint_state_topic: str = "joint_state"


@dataclass
class PoseStampedConfig:
    """
    Configuration for the Pose Stamped component.

    Reads the desired end-effector pose from a ROS 2 topic.
    """

    type: Literal["pose_stamped"]

    name: str = "end_effector"

    # ROS 2 topic to read the desired end-effector pose
    pose_stamped_topic: str = "end_effector_pose"


C = TypeVar("C", default=Never)


@TeleoperatorConfig.register_subclass("ros2_teleoperator")
@dataclass
class ROS2TeleoperatorConfig(TeleoperatorConfig, Generic[C]):

    components: dict[
        str,
        JointTrajectoryControllerStateConfig | JointStateConfig | PoseStampedConfig | C,
    ] = field(default_factory=dict)

    node_name: str = "lerobot_teleoperator"
