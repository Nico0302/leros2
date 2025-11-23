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
    JointTrajectoryControllerStateConfig,
)
from .common import BaseComponent
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState


class JointTrajectoryControllerStateComponent(BaseComponent):
    """
    Reads joint trajectory controller state from a ROS 2 topic.
    """

    def __init__(self, config: JointTrajectoryControllerStateConfig):
        self.config = config

        self.node: Node | None = None

        self._controller_state = None

    def connect(self, node: Node) -> None:
        self.node = node

        self._joint_controller_state_subscription = self.node.create_subscription(
            JointTrajectoryControllerState,
            self.config.controller_state_topic,
            self._joint_trajectory_controller_state_callback,
            10,
        )

    def disconnect(self) -> None:
        if self.node is None:
            return

        self.node.destroy_subscription(self._joint_controller_state_subscription)

        self.node = None
        self._controller_state = None

    def _joint_trajectory_controller_state_callback(
        self, msg: JointTrajectoryControllerState
    ):
        self._controller_state = msg

    @property
    def action_features(self) -> dict:
        """Get the action features provided by this component.
        Returns:
            dict: The action features.
        """
        features = {}
        for joint_name in self.config.joints.keys():
            features[f"{joint_name}.pos"] = float
        return features

    def get_action(self) -> dict[str, Any]:
        """Get the current joint states from the robot.

        Returns:
            dict[str, Any]: The current joint states.
        """
        if self._controller_state is None:
            return {}

        action = {}
        for idx, joint_name in enumerate(self._controller_state.joint_names):
            if joint_name in self.config.joints:
                action[f"{joint_name}.pos"] = self._controller_state.output.positions[
                    idx
                ]

        return action

    @property
    def feedback_features(self) -> dict:
        return {}

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass
