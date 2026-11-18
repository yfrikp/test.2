#!/usr/bin/env python3
"""把车体系 cmd_vel 转成世界系速度，供 gz-sim 的 VelocityControl 使用。

全向底盘用两套机制：
  * MecanumDrive：按 vx/vy/wz 驱动四个麦轮转动、发布 /odom 与 TF（保证可视化和里程计真实）；
  * VelocityControl：把 base_link 的平面速度直接置为指令值（运动学全向，避免圆柱轮无法产生侧向力）。
本节点负责 world = R(yaw) * body 的坐标变换，这样按 W 永远是"车头方向"，不会随朝向漂。
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OmniBridge(Node):
    def __init__(self):
        super().__init__('omni_bridge')
        self.declare_parameter('in_topic', '/cmd_vel')
        self.declare_parameter('out_topic', '/omni_cmd_vel_world')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('timeout', 0.4)
        self.yaw = 0.0
        self.last_cmd = None
        self.last_stamp = 0.0
        self.pub = self.create_publisher(Twist, self.get_parameter('out_topic').value, 10)
        self.create_subscription(Twist, self.get_parameter('in_topic').value, self.on_cmd, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom, 20)
        self.create_timer(0.05, self.on_timer)
        self.get_logger().info('全向底盘坐标变换节点已启动（车体系 -> 世界系）')

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_cmd(self, msg):
        self.last_cmd = msg
        self.last_stamp = self.now_s()
        self.publish_world(msg)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def publish_world(self, msg):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        out = Twist()
        out.linear.x = c * msg.linear.x - s * msg.linear.y
        out.linear.y = s * msg.linear.x + c * msg.linear.y
        out.angular.z = msg.angular.z
        self.pub.publish(out)

    def on_timer(self):
        # 指令超时后主动归零，避免 VelocityControl 一直保持上一次速度
        if self.last_cmd is None:
            return
        if self.now_s() - self.last_stamp > float(self.get_parameter('timeout').value):
            if any(abs(v) > 1e-6 for v in (self.last_cmd.linear.x, self.last_cmd.linear.y,
                                           self.last_cmd.angular.z)):
                zero = Twist()
                self.publish_world(zero)
                self.last_cmd = zero


def main(args=None):
    rclpy.init(args=args)
    node = OmniBridge()
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
