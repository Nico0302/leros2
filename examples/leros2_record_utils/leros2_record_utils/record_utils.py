import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Boolean

class LeROS2RecordUtils(Node):

    def __init__(self):
        super().__init__("leros2_record_utils")
        
        self._event_publisher = self.create_publisher(String, "lerobot_event", 10)
        self._task_publisher = self.create_publisher(String, "lerobot_task", 10)
        self._is_recording_publisher = self.create_publisher(Boolean, "lerobot_is_recording", 10)