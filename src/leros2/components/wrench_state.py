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

from leros2.components.common import StateComponent

from typing import Any
import numpy as np
from leros2.components.common import StateComponentConfig
from leros2.components.common.base import BaseComponentConfig
from dataclasses import dataclass
from geometry_msgs.msg import WrenchStamped


@dataclass
@BaseComponentConfig.register_subclass('wrench_state')
class WrenchStateComponentConfig(StateComponentConfig):
    name: str


class WrenchStateComponent(StateComponent[WrenchStateComponentConfig, WrenchStamped]):
    def __init__(self, config: WrenchStateComponentConfig):
        super().__init__(config, WrenchStamped)

    @property
    def features(self) -> dict[str, type]:
        return {
            f"{self._config.name}_x.force": float,
            f"{self._config.name}_y.force": float,
            f"{self._config.name}_z.force": float,
            f"{self._config.name}_x.torque": float,
            f"{self._config.name}_y.torque": float,
            f"{self._config.name}_z.torque": float,
        }

    def to_value(self, msg: WrenchStamped) -> dict[str, Any]:
        return {
            f"{self._config.name}_x.force": msg.wrench.force.x,
            f"{self._config.name}_y.force": msg.wrench.force.y,
            f"{self._config.name}_z.force": msg.wrench.force.z,
            f"{self._config.name}_x.torque": msg.wrench.torque.x,
            f"{self._config.name}_y.torque": msg.wrench.torque.y,
            f"{self._config.name}_z.torque": msg.wrench.torque.z,
        }
