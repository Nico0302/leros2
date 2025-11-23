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

from .config_ure import UReConfig
from lerobot_robot_ros2 import ROS2Robot


class URe(ROS2Robot):
    """Robot class for an URe series robot arm."""

    name = "ure"

    def __init__(self, config: UReConfig):
        super().__init__(config)
