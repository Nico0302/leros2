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
from lerobot_teleoperator_ros2.config_ros2_teleoperator import (
    JointStateConfig,
)
from .common import BaseComponent
from rclpy.node import Node
from lerobot_common_ros2.components import (
    JointStateComponent as BaseJointStateComponent,
)


class JointStateComponent(BaseComponent):
    """
    Reads joint states from a ROS 2 topic.
    """

    def __init__(self, config: JointStateConfig):
        self.config = config

        self.node: Node | None = None

        self._joint_state_component = BaseJointStateComponent(self.config)

    def connect(self, node: Node) -> None:
        self.node = node

        self._joint_state_component.connect(node)

    def disconnect(self) -> None:
        if self.node is None:
            return

        self._joint_state_component.disconnect()
        self.node = None

    @property
    def action_features(self) -> dict:
        """Get the action features provided by this component.
        Returns:
            dict: The action features.
        """
        return self._joint_state_component.joint_features

    def get_action(self) -> dict[str, Any]:
        """Get the current joint states from the robot.

        Returns:
            dict[str, Any]: The current joint states.
        """
        return self._joint_state_component.get_joints()

    @property
    def feedback_features(self) -> dict:
        return {}

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass
