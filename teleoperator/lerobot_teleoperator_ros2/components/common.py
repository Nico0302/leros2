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
from abc import ABC, abstractmethod
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
    @abstractmethod
    def action_features(self) -> dict:
        """
        A dictionary describing the structure and types of the actions produced by the teleoperator. Its
        structure (keys) should match the structure of what is returned by :pymeth:`get_action`. Values for
        the dict should be the type of the value if it's a simple value, e.g. `float` for single
        proprioceptive value (a joint's goal position/velocity)

        Note: this property should be able to be called regardless of whether the robot is connected or not.
        """
        pass

    @property
    @abstractmethod
    def feedback_features(self) -> dict:
        """
        A dictionary describing the structure and types of the feedback actions expected by the robot. Its
        structure (keys) should match the structure of what is passed to :pymeth:`send_feedback`. Values for
        the dict should be the type of the value if it's a simple value, e.g. `float` for single
        proprioceptive value (a joint's goal position/velocity)

        Note: this property should be able to be called regardless of whether the robot is connected or not.
        """
        pass

    @abstractmethod
    def get_action(self) -> dict[str, Any]:
        """
        Retrieve the current action from the teleoperator.

        Returns:
            dict[str, Any]: A flat dictionary representing the teleoperator's current actions. Its
                structure should match :pymeth:`observation_features`.
        """
        pass

    @abstractmethod
    def send_feedback(self, feedback: dict[str, Any]) -> None:
        """
        Send a feedback action command to the teleoperator.

        Args:
            feedback (dict[str, Any]): Dictionary representing the desired feedback. Its structure should match
                :pymeth:`feedback_features`.

        Returns:
            dict[str, Any]: The action actually sent to the motors potentially clipped or modified, e.g. by
                safety limits on velocity.
        """
        pass
