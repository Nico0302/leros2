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
from lerobot_common_ros2.components import JointStateComponent, unnormalize_joint
from lerobot_robot_ros2.config_ros2_robot import JointTrajectoryComponentConfig
from .common import BaseComponent
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointTrajectoryComponent(BaseComponent):
    """
    Sends joint trajectory commands to a ROS 2 control and retrieves joint states.
    """

    def __init__(self, config: JointTrajectoryComponentConfig):
        self.config = config

        self.node: Node | None = None

        self._joint_state_component = JointStateComponent(self.config)

    def connect(self, node: Node) -> None:
        """Connect to the robot."""
        self.node = node

        self._joint_state_component.connect(node)

        self._joint_trajectory_publisher = self.node.create_publisher(
            JointTrajectory,
            self.config.joint_trajectory_topic,
            10,
        )

    def disconnect(self) -> None:
        if self.node is None:
            return

        self._joint_state_component.disconnect()

        self.node.destroy_publisher(self._joint_trajectory_publisher)

        self.node = None

    @property
    def action_features(self) -> dict[str, type]:
        """Get the action types supported by the robot."""
        return self._joint_state_component.joint_features

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Send a joint trajectory action to the robot.

        Args:
            action (dict[str, JointTrajectory]): The joint trajectory action to send.

        Returns:
            dict[str, Any]: An empty response.
        """

        if self.node is None:
            raise RuntimeError("Node not connected.")

        msg = JointTrajectory()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.joint_names = []
        msg.points = [JointTrajectoryPoint()]

        for name, config in self.config.joints.items():
            value = action[f"{name}.pos"]
            if value is None:
                raise ValueError(f"Joint '{name}' not found in action.")
            msg.joint_names.append(name)
            msg.points[0].positions.append(unnormalize_joint(value, config))

        self._joint_trajectory_publisher.publish(msg)

        return action

    @property
    def observation_features(self) -> dict[str, Any]:
        """Get the observation types provided by the robot."""
        return self._joint_state_component.joint_features

    def get_observation(self) -> dict[str, Any]:
        """Get the current joint states from the robot.

        Returns:
            dict[str, Any]: The current joint states.
        """
        return self._joint_state_component.get_joints()
