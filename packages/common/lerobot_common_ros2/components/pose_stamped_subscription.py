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
from lerobot_common_ros2.config_ros2_common import (
    PoseStampedSubscriptionComponentConfig,
)
from .common import BaseComponent
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from lerobot.utils.rotation import Rotation
import numpy as np


class PoseStampedSubscriptionComponent(BaseComponent):
    """
    Retrieves pose stamped information from a ROS 2 topic.
    """

    def __init__(self, config: PoseStampedSubscriptionComponentConfig):
        self.config = config

        self.node: Node | None = None

        self._pose_stamped: PoseStamped | None = None

    def connect(self, node: Node) -> None:
        self.node = node

        self._pose_stamped_subscription = self.node.create_subscription(
            PoseStamped,
            self.config.pose_stamped_topic,
            self._pose_stamped_callback,
            10,
        )

    def disconnect(self) -> None:
        if self.node is None:
            return

        self.node.destroy_subscription(self._pose_stamped_subscription)

        self.node = None
        self._pose_stamped = None

    def _pose_stamped_callback(self, msg: PoseStamped):
        self._pose_stamped = msg

    @property
    def get_pose_features(self) -> dict:
        """Get the pose features provided by this component.
        Returns:
            dict: The pose features.
        """
        return {
            f"{self.config.name}.pos": np.ndarray,  # shape (3,)
            f"{self.config.name}.rot": Rotation,  # quaternion
        }

    def get_pose(self) -> dict[str, Any]:
        """Get the current pose from the robot.

        Returns:
            dict[str, Any]: The current pose.
        """
        if self._pose_stamped is None:
            return {}

        return {
            f"{self.config.name}.pos": np.array(
                [
                    self._pose_stamped.pose.position.x,
                    self._pose_stamped.pose.position.y,
                    self._pose_stamped.pose.position.z,
                ]
            ),
            f"{self.config.name}.rot": Rotation.from_quat(
                np.array(
                    [
                        self._pose_stamped.pose.orientation.x,
                        self._pose_stamped.pose.orientation.y,
                        self._pose_stamped.pose.orientation.z,
                        self._pose_stamped.pose.orientation.w,
                    ]
                )
            ),
        }