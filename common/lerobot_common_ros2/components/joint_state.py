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

from typing import Any
from lerobot_common_ros2 import JointConfig
from lerobot_common_ros2.config_ros2_common import JointStateComponentConfig
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateComponent:
    """
    Retrieves joint states from a ROS 2 robot.
    """

    def __init__(self, config: JointStateComponentConfig):
        self.config = config

        self._joint_state: JointState | None = None
        self.node: Node | None = None

    def connect(self, node: Node) -> None:
        """Connect to the robot."""
        self.node = node

        self._joint_state_subscription = self.node.create_subscription(
            JointState,
            self.config.joint_state_topic,
            self._joint_state_callback,
            10,
        )

    def disconnect(self) -> None:
        if self.node is None:
            return

        self.node.destroy_subscription(self._joint_state_subscription)

        self._joint_state = None

    def _joint_state_callback(self, msg: JointState):
        self._joint_state = msg

    def get_joints(self) -> dict[str, Any]:
        """Get the current joint states from the robot.

        Returns:
            dict[str, Any]: The current joint states.
        """

        if self._joint_state is None:
            raise RuntimeError("No joint state received yet.")

        observation: dict[str, Any] = {}

        for name, index in enumerate(self._joint_state.name):
            observation[f"{name}.pos"] = normalize_joint(
                self._joint_state.position[index],
                self.config.joints[index],
            )

        return observation

    @property
    def joint_features(self) -> dict[str, type]:
        features: dict[str, type] = {}
        for name in self.config.joints.keys():
            features[f"{name}.pos"] = float
        return features


def normalize_joint(value: float, config: JointConfig) -> float:
    """Normalize a joint value from radians."""

    return (value - config.range_min) / (
        config.range_max - config.range_min
    ) * 2.0 - 1.0


def unnormalize_joint(value: float, config: JointConfig) -> float:
    """Unnormalize a joint value to radians."""

    return ((value + 1.0) / 2.0) * (
        config.range_max - config.range_min
    ) + config.range_min
