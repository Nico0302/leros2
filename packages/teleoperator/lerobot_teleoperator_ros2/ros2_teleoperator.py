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

import logging
from typing import Any

from lerobot_common_ros2 import ROS2Common

from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.teleoperators.teleoperator import Teleoperator

from .components.common import BaseComponent
from .config_ros2_teleoperator import (
    JointStateConfig,
    PoseStampedConfig,
    ROS2TeleoperatorConfig,
    JointTrajectoryControllerStateConfig,
)


logger = logging.getLogger(__name__)


class ROS2Teleoperator(ROS2Common[ROS2TeleoperatorConfig, BaseComponent], Teleoperator):
    config_class = ROS2TeleoperatorConfig
    name = "ros2_teleoperator"

    def __init__(self, config: ROS2TeleoperatorConfig):
        Teleoperator.__init__(self, config)
        ROS2Common.__init__(self, config)

    def _init_components(self):
        for comp_name, comp_cfg in self.config.components.items():
            if isinstance(comp_cfg, JointTrajectoryControllerStateConfig):
                from .components.joint_trajectory_controller_state import (
                    JointTrajectoryControllerStateComponent,
                )

                self._components[comp_name] = JointTrajectoryControllerStateComponent(
                    comp_cfg
                )
            elif isinstance(comp_cfg, JointStateConfig):
                from .components.joint_state import (
                    JointStateComponent,
                )

                self._components[comp_name] = JointStateComponent(comp_cfg)
            elif isinstance(comp_cfg, PoseStampedConfig):
                from .components.pose_stamped import (
                    PoseStampedComponent,
                )

                self._components[comp_name] = PoseStampedComponent(comp_cfg)
            else:
                raise ValueError(f"Unknown component config type: {type(comp_cfg)}")

    def connect(self, calibrate: bool = True) -> None:
        ROS2Common.connect(self, calibrate)

    def disconnect(self) -> None:
        ROS2Common.disconnect(self)

    @property
    def action_features(self) -> dict[str, type]:
        """Get the action features provided by this teleoperator.
        Returns:
            dict[str, type]: The action features provided by this teleoperator.
        """
        action_features = {}
        for comp in self._components.values():
            action_features.update(comp.action_features)
        return action_features

    def get_action(self) -> dict[str, Any]:
        """Get the current teleoperator action.

        Returns:
            dict[str, Any]: The current teleoperator action.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} not connected")

        action: dict[str, Any] = {}

        for comp_name, component in self._components.items():
            action.update(component.get_action())

        return action
