#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time

class FingerMotionNode(Node):
    def __init__(self):
        super().__init__("finger_motion_node")

        self.publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10
        )

        self.joint_names = [
            "thumb_joint0", "thumb_joint1", "thumb_joint2", "thumb_joint3", "thumb_joint4", "thumb_joint5",
            "index_joint0", "index_joint1", "index_joint2", "index_joint3", "index_joint4",
            "middle_joint0", "middle_joint1", "middle_joint2", "middle_joint3", "middle_joint4",
            "ring_joint0", "ring_joint1", "ring_joint2", "ring_joint3", "ring_joint4",
            "little_joint0", "little_joint1", "little_joint2", "little_joint3", "little_joint4",
        ]

        self.n = len(self.joint_names)
        self.positions = np.zeros(self.n)
        self.delta_t = 0.02

        self.velocity_cmd = np.array([
            0, np.pi/2, np.pi/6, np.pi/6, 0, 0,        # thumb
            0, 0, np.pi/6, np.pi/6, 0,           # index
            0, 0, np.pi/6, np.pi/6, 0,           # middle
            0, 0, np.pi/6, np.pi/6, 0,           # ring
            0, 0, np.pi/6, np.pi/6, 0,           # little
        ])

        self.motion_duration = 1.0  # seconds
        self.elapsed = 0.0

        self.timer = self.create_timer(self.delta_t, self.update)  # 50 Hz
        self.start_time = self.get_clock().now()
        
        # MUST MATCH URDF JOINT NAMES EXACTLY
        

        self.get_logger().info("Finger motion node started")

    def update(self):
        msg = JointState()
        now = self.get_clock().now()
        msg.header.stamp = Time(
            sec=now.seconds_nanoseconds()[0],
            nanosec=now.seconds_nanoseconds()[1]
        )

        msg.name = self.joint_names

        if self.elapsed < self.motion_duration:
            self.positions += self.velocity_cmd * self.delta_t
            msg.velocity = self.velocity_cmd.tolist()
            self.elapsed += self.delta_t
        else:
            msg.velocity = np.zeros(self.n).tolist()

        msg.position = self.positions.tolist()
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = FingerMotionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
