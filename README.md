# ROS 2 LeRobot

Map [ROS 2](https://www.ros.org/) topics and actions to [LeRobot](https://github.com/huggingface/lerobot) robots and teleoperators.

## Robot

A [LeRobot robot](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/robot.py) outputs observations such as joint states and publishes actions in the form of joint trajectories.

A ROS 2 LeRobot robot consists of *components* that subscribe to topics and publish ROS 2 messages.

```yaml
# file: ur12e_robot_config.yaml

id: ur12e
components:
    arm:  # you can choose your own component name e.g. arm, gripper, left_arm
        type: joint_trajectory
        joint_trajectory_topic: "scaled_joint_trajectory_topic/joint_trajectory"
        joints:
            shoulder_pan_joint: {}
            shoulder_lift_joint:
                range_min: -1,5707963268 # -PI/2
                range_max: 1,5707963268 # PI/2
            elbow_joint: {}
            wrist_1_joint: {}
            wrist_2_joint: {}
            wrist_3_joint: {}
    gripper: ...
```

## Teleoperator

A [LeRobot teleoperator](https://github.com/huggingface/lerobot/blob/main/src/lerobot/teleoperators/teleoperator.py) retrieves actions from a teleportation device such as a leader arm or a VR controller.

This package provides two modes of teleportation based on a modular component approach:

*Active* teleportation utilizes the standard LeRobot pipeline in which a teleportation device sends joint actions which are then sent to the robot by LeRobot. \
You can use the `JointStateComponent` to read joints from a leader arm or the `PoseStampedComponent` to read a cartesian pose from an end-effector or a VR controller. The policy either needs to support end-effector pose actions or a [`teleop_action_processor`](https://github.com/huggingface/lerobot/blob/main/examples/phone_to_so100/record.py) needs to be configured.

*Passive* teleportation decouples LeRobot from the teleportation loop. The robot can be directly controlled by a ROS 2 node which also publishes the control actions to a ROS 2 topic. This allows the use of a low latency leader follower setup with force feedback or a VR controlled end-effector teleportation with MoveIt and the `JointTrajectoryControllerStateComponent`. \
The *passive* mode can be enabled by configuring the robot during dataset recording:
```bash
lerobot-record --robot.passive=true
```


## Custom Components

You can implement your own ROS 2 component adapters by extending the `ROS2Robot` class:

```python
# file: <robot_name>.py

from .config_ure import UReConfig, MyCustomComponentConfig
from .components import MyCustomComponent
from lerobot_robot_ros2 import ROS2Robot

class URe(ROS2Robot):
    """Robot class for an URe series robot arm."""

    name = "ure"

    def __init__(self, config: UReConfig):
        super().__init__(config)

    def _init_components(self):
        super().init_components(self) # make sure to initialize existing components

        for comp_name, comp_cfg in self.config.components.items():
            elif isinstance(comp_cfg, MyCustomComponentConfig):
                from .components import MyCustomComponent import MyCustomComponent

                self._components[comp_name] = MyCustomComponent(comp_cfg)
```

You need to create your own config class:

```python
# file: config_<robot_name>.py

from dataclasses import dataclass
from lerobot_robot_ros2 import ROS2RobotConfig
from lerobot.robots.config import RobotConfig

@dataclass
class MyCustomComponentConfig:
    pass

@RobotConfig.register_subclass("ure")
@dataclass
class UReConfig(ROS2RobotConfig[MyCustomComponentConfig]):
    """Configuration for the UR12e robot arm."""
    pass
```

The custom robot and teleoperator need to be added to a custom python package prefixed with `lerobot_robot_` and `lerobot_teleoperator_` respectively. Read more about creating custom LeRobot devices [here](https://huggingface.co/docs/lerobot/en/integrate_hardware#using-your-own-lerobot-devices-).