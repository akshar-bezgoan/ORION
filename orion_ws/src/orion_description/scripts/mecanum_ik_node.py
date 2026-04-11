#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class MecanumIKNode(Node):

    def __init__(self):
        super().__init__('mecanum_ik_node')

        self.r  = 0.05   
        self.lx = 0.095   
        self.ly = 0.13    

        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        self.pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocity_controller/commands', 10)

        self.get_logger().info(
            f'mecanum_ik_node ready — r={self.r} lx={self.lx} ly={self.ly}')

    def cmd_vel_cb(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        k = self.lx + self.ly
        r = self.r

        fl = (vx - vy - k * wz) / r
        fr = (vx + vy + k * wz) / r
        rl = (vx + vy - k * wz) / r
        rr = (vx - vy + k * wz) / r

        msg_out = Float64MultiArray()
        msg_out.data = [fl, fr, rl, rr]
        self.pub.publish(msg_out)

        self.get_logger().debug(
            f'vx={vx:.2f} vy={vy:.2f} wz={wz:.2f} → '
            f'fl={fl:.2f} fr={fr:.2f} rl={rl:.2f} rr={rr:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = MecanumIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
