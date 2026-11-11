#!/usr/bin/env python3
"""Moves joint 1 of the Panda by 0.2 rad in 4 s, streaming positions at 100 Hz."""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from uclv_panda_interfaces.msg import JointsCommandedPositions, PandaRobotState

DURATION = 4.0  # s


class JointClient(Node):
    def __init__(self):
        super().__init__('my_joint_client')
        self.pub = self.create_publisher(JointsCommandedPositions, '/panda/joint_position_command', 10)
        self.create_subscription(PandaRobotState, '/panda/robot_state', self.on_state, 10)
        self.start = None

    def on_state(self, msg):
        # The stream starts from the position commanded now (q_d), then a timer publishes at 100 Hz
        if self.start is None:
            self.start = list(msg.q_d)
            self.t0 = self.get_clock().now()
            self.create_timer(0.01, self.publish)

    def publish(self):
        t = min((self.get_clock().now() - self.t0).nanoseconds * 1e-9, DURATION)
        cmd = JointsCommandedPositions()
        cmd.header.stamp = (self.t0 + Duration(nanoseconds=int(t * 1e9))).to_msg()  # instant of the sample
        cmd.positions = list(self.start)
        s = t / DURATION
        cmd.positions[0] += 0.2 * (10 * s**3 - 15 * s**4 + 6 * s**5)  # quintic: starts and ends at rest
        self.pub.publish(cmd)
        if t == DURATION:
            raise SystemExit  # last sample, at rest: the bridge holds this position


def main():
    rclpy.init()
    node = JointClient()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
