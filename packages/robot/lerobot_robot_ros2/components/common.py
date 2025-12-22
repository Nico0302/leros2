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
from abc import ABC
from rclpy.node import Node


class BaseComponent(ABC):
    """
    Base interface for LeROS2 robots.
    """

    node: Node | None = None

    def connect(self, node: Node) -> None:
        """Connect to the robot."""
        self.node = node

    def disconnect(self) -> None:
        """Disconnect from the robot."""
        pass

    @property
    def action_features(self) -> dict[str, type]:
        """Get the action types supported by the robot."""
        return {}

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Send an action to the robot.

        Args:
            action (dict[str, Any]): The action to send.
        Returns:
            dict[str, Any]: The response from the robot.
        """
        return {}

    @property
    def observation_features(self) -> dict[str, Any]:
        """Get the observation types provided by the robot."""
        return {}

    def get_observation(self) -> dict[str, Any]:
        """Get the current observation from the robot.

        Returns:
            dict[str, Any]: The current observation.
        """
        return {}
