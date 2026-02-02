# Copyright 2026 Nicolas Gres
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
from leros2.components.common.action import ActionComponent
from leros2.components.pose_action import PoseActionComponentConfig, PoseActionComponent
from leros2.components.pose_state import PoseStateComponentConfig, PoseStateComponent
from dataclasses import dataclass

from typing import Any
import numpy as np
from lerobot.utils.rotation import Rotation
from leros2.components.common import ActionComponentConfig, ActionTopicComponent
from leros2.components.common.base import BaseComponentConfig
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

@dataclass
@BaseComponentConfig.register_subclass('relative_pose_action')
class RelativePoseActionComponentConfig(ActionComponentConfig):
    state: PoseStateComponentConfig
    action: PoseActionComponentConfig

class RelativePoseActionComponent(ActionComponent[RelativePoseActionComponentConfig]):
    def __init__(self, config: RelativePoseActionComponentConfig):
        super().__init__(config)

        self._state = PoseStateComponent(config.state)
        self._action = PoseActionComponent(config.action)

    @property
    def features(self) -> dict[str, type]:
        self._action.features

    def connect(self, node: Node) -> None:
        super().connect(node)

        self._state.connect(Node)
        self._action.connect(Node)

    def disconnect(self) -> None:
        self._state.disconnect()
        self._action.disconnect()

        super().disconnect()

    def send_action(self, action: dict[str, Any]) -> None:
        """Send an action to the ROS 2 message.

        Args:
            action: The action features to send.
        """
        msg = self.to_message(action)
        self._publisher.publish(msg)

    
