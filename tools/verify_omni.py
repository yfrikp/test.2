#!/usr/bin/env python3
"""实测 omni_bot 是否真的能全向移动（横移 / 前后 / 原地旋转）。

用 Gazebo 的世界真值位姿（/gz_ground_truth）判断，而不是用 /odom（odom 由轮速推算，会自证）。
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class OmniCheck(Node):
    def __init__(self):
        super().__init__('omni_check')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(TFMessage, '/gz_ground_truth', self._on_pose, 10)
        self.x = self.y = self.yaw = None

    def _on_pose(self, msg):
        for t in msg.transforms:
            if 'omni_bot' in t.child_frame_id:
                self.x = t.transform.translation.x
                self.y = t.transform.translation.y
                q = t.transform.rotation
                self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def wait_pose(self, timeout=25.0):
        t0 = time.time()
        while self.x is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.x is not None

    def drive(self, vx, vy, wz, seconds):
        t0 = time.time()
        rate = 1.0 / 20.0
        while time.time() - t0 < seconds:
            self.pub.publish(self._twist(vx, vy, wz))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(rate)
        for _ in range(10):
            self.pub.publish(self._twist(0.0, 0.0, 0.0))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)
        t_end = time.time() + 0.6
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)

    @staticmethod
    def _twist(vx, vy, wz):
        m = Twist()
        m.linear.x = vx
        m.linear.y = vy
        m.angular.z = wz
        return m

    def pose(self):
        return (self.x, self.y, self.yaw)


def main():
    rclpy.init()
    node = OmniCheck()
    if not node.wait_pose():
        print('[FAIL] 收不到 /gz_ground_truth，仿真没起来')
        return 2

    cases = [
        ('前进  linear.x=+0.25', 0.25, 0.0, 0.0, 3.0),
        ('后退  linear.x=-0.25', -0.25, 0.0, 0.0, 3.0),
        ('左横移 linear.y=+0.25', 0.0, 0.25, 0.0, 3.0),
        ('右横移 linear.y=-0.25', 0.0, -0.25, 0.0, 3.0),
        ('原地左转 angular.z=+0.8', 0.0, 0.0, 0.8, 1.5),
        ('原地右转 angular.z=-0.8', 0.0, 0.0, -0.8, 1.5),
    ]
    print('=' * 74)
    print(f'{"指令":<26}{"起始位姿":<26}{"位移":<22}判定')
    print('=' * 74)
    ok = 0
    for name, vx, vy, wz, dur in cases:
        x0, y0, yaw0 = node.pose()
        node.drive(vx, vy, wz, dur)
        x1, y1, yaw1 = node.pose()
        dx, dy = x1 - x0, y1 - y0
        dyaw = math.degrees(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)))
        if vx:
            good = abs(dx) > 0.30 and abs(dy) < 0.12
        elif vy:
            good = abs(dy) > 0.30 and abs(dx) < 0.12
        else:
            good = abs(dyaw) > 25.0
        ok += good
        print(f'{name:<22}({x0:+.3f},{y0:+.3f},{math.degrees(yaw0):+6.1f}°)  '
              f'Δ=({dx:+.3f},{dy:+.3f},{dyaw:+6.1f}°)  {"✅ 通过" if good else "❌ 未通过"}')
    print('=' * 74)
    print(f'结果：{ok}/{len(cases)} 个方向通过')
    rclpy.shutdown()
    return 0 if ok == len(cases) else 1


if __name__ == '__main__':
    sys.exit(main())
