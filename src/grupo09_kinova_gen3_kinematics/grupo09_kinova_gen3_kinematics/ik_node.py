import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

from grupo09_kinova_gen3_kinematics.kinematics import (
    JOINT_NAMES,
    JOINT_LIMITS,
    forward_kinematics,
    position_jacobian,
)

Q_HOME = np.radians([0.0, 15.0, -130.0, 0.0, 55.0, 90.0])


class IKNode(Node):

    def __init__(self):
        super().__init__('ik_node')

        self.alpha = 0.5
        self.tolerance = 1e-4
        self.max_iterations = 500
        self.max_step = 0.2

        self.q_current = Q_HOME.copy()

        self.subscription = self.create_subscription(
            Point,
            '/target',
            self.target_callback,
            10
        )

        self.joint_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_joint_states)

        np.set_printoptions(precision=4, suppress=True)
        self.get_logger().info(
            'IK node ready. Waiting for /target (geometry_msgs/Point)\n'
            f'q_home [rad] = {Q_HOME}'
        )

    def solve_ik(self, p_d, q0):

        q = q0.copy()

        for k in range(1, self.max_iterations + 1):

            p = forward_kinematics(q)[:3, 3]
            e = p_d - p
            error = np.linalg.norm(e)

            if error < self.tolerance:
                return q, k, error, True

            J = position_jacobian(q)
            dq = self.alpha * np.linalg.pinv(J) @ e

            step = np.linalg.norm(dq)
            if step > self.max_step:
                dq = dq * (self.max_step / step)

            q = np.clip(q + dq, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])

        p = forward_kinematics(q)[:3, 3]
        return q, self.max_iterations, np.linalg.norm(p_d - p), False

    def target_callback(self, msg):

        p_d = np.array([msg.x, msg.y, msg.z])
        q0 = self.q_current.copy()

        q_sol, iterations, error, converged = self.solve_ik(p_d, q0)

        p_f = forward_kinematics(q_sol)[:3, 3]

        status = 'CONVERGED' if converged else 'NOT CONVERGED'

        self.get_logger().info(
            '\n--- IK ---'
            f'\ntarget p_d [m]   = {p_d}'
            f'\nq0 [rad]         = {q0}'
            f'\nq* [rad]         = {q_sol}'
            f'\niterations       = {iterations}'
            f'\np(q*) [m]        = {p_f}'
            f'\nerror ||e|| [m]  = {error:.6f}'
            f'\nstatus           = {status}'
        )

        if converged:
            self.q_current = q_sol
        else:
            self.get_logger().warn(
                'Target not reached (check reachability). '
                'Keeping previous configuration.'
            )

    def publish_joint_states(self):

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(v) for v in self.q_current]
        self.joint_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
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
