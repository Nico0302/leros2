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
import math
from lerobot_robot_ros2 import JointConfig, ROS2RobotConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("ure")
@dataclass
class UReConfig(ROS2RobotConfig):
    """Configuration for the UR12e robot arm."""

    joint_trajectory_topic: str = "scaled_joint_trajectory_topic/joint_trajectory"

    # Joints configuration
    joints: dict[str, JointConfig] = field(
        default_factory=lambda: {
            "shoulder_pan_joint": JointConfig(
                range_min=-math.pi,
                range_max=math.pi,
            ),
            "shoulder_lift_joint": JointConfig(
                range_min=-math.pi / 2,
                range_max=math.pi / 2,
            ),
            "elbow_joint": JointConfig(
                range_min=-math.pi,
                range_max=math.pi,
            ),
            "wrist_1_joint": JointConfig(
                range_min=-math.pi,
                range_max=math.pi,
            ),
            "wrist_2_joint": JointConfig(
                range_min=-math.pi,
                range_max=math.pi,
            ),
            "wrist_3_joint": JointConfig(
                range_min=-math.pi,
                range_max=math.pi,
            ),
        }
    )
