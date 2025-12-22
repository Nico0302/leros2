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

import math
from typing import Protocol, TypeVar

from dataclasses import dataclass, field

T = TypeVar("T")


@dataclass
class JointConfig:
    """Configuration for a single joint."""

    # Minimum joint range in radians
    range_min: float = -math.pi

    # Maximum joint range in radians
    range_max: float = math.pi


class JointStateComponentConfig(Protocol):
    """Configuration for a joint level control component."""

    # Joints configuration
    joints: dict[str, JointConfig] = field(default_factory=dict)

    # ROS 2 topic to read the current joint states
    joint_state_topic: str = "joint_state"


class PoseStampedSubscriptionComponentConfig(Protocol):
    """Configuration for a pose level control component."""

    # Name of the component
    name: str = "end_effector"

    # ROS 2 topic to read the current pose
    pose_stamped_topic: str = "pose_stamped"

class ROS2CommonConfig(Protocol[T]):
    """Configuration for ROS 2 common features."""

    node_name: str

    components: dict[str, T]
