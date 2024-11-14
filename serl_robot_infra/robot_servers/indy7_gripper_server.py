import rclpy
from std_msgs.msg import UInt8
from sensor_msgs.msg import JointState
import numpy as np

from robot_servers.gripper_server import GripperServer


class Indy7GripperServer(GripperServer):
    def __init__(self, parent_node: rclpy.Node):
        super().__init__()
        self.parent_node = parent_node
        self.grippermovepub = self.parent_node.create_publisher(
            "/indy7_gripper/move/goal", UInt8, queue_size=1
        )
        self.grippergrasppub = self.parent_node.create_publisher(
            "/indy7_gripper/grasp/goal", UInt8, queue_size=1
        )
        self.gripper_sub = self.parent_node.create_subscription(
            "/indy7_gripper/joint_states", JointState, self._update_gripper
        )

    def open(self):
        msg = UInt8()
        msg.data = 0
        self.grippermovepub.publish(msg)

    def close(self):
        msg = UInt8()
        msg.data = 1
        self.grippergrasppub.publish(msg)

    def move(self, position: int):
        """Move the gripper to a specific position in range [0, 255]"""
        msg = UInt8()
        msg.data = position
        self.grippermovepub.publish(msg)

    def _update_gripper(self, msg):
        """internal callback to get the latest gripper position."""
        self.gripper_pos = np.sum(msg.position)
