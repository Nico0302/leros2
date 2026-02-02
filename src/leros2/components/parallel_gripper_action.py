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
from leros2.components.joint_state import JointConfig

from dataclasses import dataclass

from control_msgs.action import ParallelGripperCommand
from typing import Any
import numpy as np
from lerobot.utils.rotation import Rotation
from leros2.components.common import ActionComponentConfig, ActionClientComponent
from leros2.components.common.base import BaseComponentConfig


@dataclass
@BaseComponentConfig.register_subclass('parallel_gripper_action')
class ParallelGripperActionComponentConfig(ActionComponentConfig):
    joints: list[JointConfig]


class ParallelGripperActionComponent(ActionClientComponent[ParallelGripperActionComponentConfig, ParallelGripperCommand]):
    def __init__(self, config: ParallelGripperActionComponentConfig):
        super().__init__(config, ParallelGripperCommand)

    @property
    def features(self) -> dict[str, type]:
        features: dict[str, type] = {}

        for joint in self._config.joints:
            features[f"{joint.name}.pos"] = float

        return features

    def to_goal(self, action: dict[str, Any]) -> Any:
        msg = ParallelGripperCommand.Goal()

        # [TODO] implement

        return msg
