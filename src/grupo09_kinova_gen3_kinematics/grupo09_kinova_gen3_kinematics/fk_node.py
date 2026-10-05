import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from grupo09_kinova_gen3_kinematics.kinematics import (
    JOINT_NAMES,
    forward_kinematics,
    rotation_to_quaternion,
)


class FKNode(Node):

    def __init__(self):
        super().__init__('fk_node')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/fk_pose',
            10
        )

        self.last_q = None

        self.get_logger().info('FK node ready. Listening to /joint_states')

    def joint_states_callback(self, msg):

        positions = dict(zip(msg.name, msg.position))

        if not all(name in positions for name in JOINT_NAMES):
            return

        q = np.array([positions[name] for name in JOINT_NAMES])

        T = forward_kinematics(q)
        p = T[:3, 3]
        R = T[:3, :3]
        quat = rotation_to_quaternion(R)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = float(p[0])
        pose.pose.position.y = float(p[1])
        pose.pose.position.z = float(p[2])
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        self.pose_publisher.publish(pose)

        if self.last_q is not None and np.allclose(q, self.last_q, atol=1e-4):
            return

        self.last_q = q.copy()

        np.set_printoptions(precision=4, suppress=True)
        self.get_logger().info(
            '\n--- FK ---'
            f'\nq [rad]           = {q}'
            f'\np [m] (x, y, z)   = {p}'
            f'\nquat (x, y, z, w) = {quat}'
            f'\nR =\n{R}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
