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
import copy
from typing import Any

from lerobot_common_ros2 import ROS2Common

from lerobot.cameras import make_cameras_from_configs
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.robots.robot import Robot

from .components.common import BaseComponent
from .config_ros2_robot import (
    GripperComponentConfig,
    JointTrajectoryComponentConfig,
    ROS2RobotConfig,
)

logger = logging.getLogger(__name__)


class ROS2Robot(ROS2Common[ROS2RobotConfig, BaseComponent], Robot):
    config_class = ROS2RobotConfig
    name = "ros2_robot"

    def __init__(self, config: ROS2RobotConfig):
        Robot.__init__(self, config)
        ROS2Common.__init__(self, config)

        self.cameras = make_cameras_from_configs(config.cameras)

    def _init_components(self):
        for comp_name, comp_cfg in self.config.components.items():
            if isinstance(comp_cfg, JointTrajectoryComponentConfig):
                from lerobot_robot_ros2.components.joint_trajectory import (
                    JointTrajectoryComponent,
                )

                self._components[comp_name] = JointTrajectoryComponent(comp_cfg)
            elif isinstance(comp_cfg, GripperComponentConfig):
                from lerobot_robot_ros2.components.gripper import GripperComponent

                self._components[comp_name] = GripperComponent(comp_cfg)

    def connect(self, calibrate: bool = True) -> None:
        ROS2Common.connect(self, calibrate)

        for cam in self.cameras.values():
            cam.connect(self._node)

    def disconnect(self) -> None:
        if not self.is_connected:
            return

        for cam in self.cameras.values():
            cam.disconnect()

        ROS2Common.disconnect(self)

    @property
    def action_features(self) -> dict[str, type]:
        action_features = {}
        for comp in self._components.values():
            action_features.update(comp.action_features)
        return action_features

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self.config.passive:
            return action

        action = copy.deepcopy(action)

        for comp in self._components.values():
            comp_action = {
                k: action.pop(k) for k in comp.action_features.keys() if k in action
            }
            comp.send_action(comp_action)

        return action

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected.")

        # Read arm position
        obs_dict = {}
        for comp in self._components.values():
            obs_dict.update(comp.get_observation())

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        return obs_dict

    def reset(self):
        pass

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @property
    def observation_features(self) -> dict[str, Any]:
        obs_features = {}
        for comp in self._components.values():
            obs_features.update(comp.observation_features)
        for cam_key, cam in self.cameras.items():
            obs_features[cam_key] = (cam.config.height, cam.config.width, 3)
        return obs_features

    @property
    def cameras(self):
        return self._cameras

    @cameras.setter
    def cameras(self, value):
        self._cameras = value
