import select
import sys
import termios
import time
import tty
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
BANNER = '\n╔══════════════════════════════════════════════════════════════════╗\n║           WASDQE 小车遥控  (题目 1)                              ║\n╠══════════════════════════════════════════════════════════════════╣\n║   W 前进      S 后退                                             ║\n║   A 左转      D 右转          （边走边转，像汽车打方向）          ║\n║   Q 原地左转  E 原地右转      （只转不走）                        ║\n║                                                                  ║\n║   空格 急停          + / - 加减速        1 2 3 速度档位          ║\n║   H 帮助             Ctrl+C 退出                                 ║\n╚══════════════════════════════════════════════════════════════════╝\n\n  ※ 终端不会告诉你"按键松开了"，所以速度是"锁定"的：\n    按 W 之后会一直前进，直到你按空格或 S 或 Q/E。\n    这是所有 ROS2 遥控程序的通用做法（包括官方的 teleop_twist_keyboard）。\n'

class WasdqeTeleop(Node):
    SPEED_PRESETS = [(0.12, 0.5, 0.7), (0.25, 1.0, 1.4), (0.45, 1.8, 2.4)]

    def __init__(self) -> None:
        super().__init__('wasdqe_teleop')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('preset', 2)
        topic = self.get_parameter('cmd_topic').value
        self.rate_hz = float(self.get_parameter('publish_rate').value)
        self.scale = float(self.get_parameter('speed_scale').value)
        self.preset = int(self.get_parameter('preset').value)
        self.publisher = self.create_publisher(Twist, topic, 10)
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_cmd)
        self.get_logger().info(f'键盘遥控已启动，发布到话题 {topic} @ {self.rate_hz:.0f} Hz')
        self.print_status()

    def speeds(self):
        fwd, turn, spin = self.SPEED_PRESETS[self.preset - 1]
        return (fwd * self.scale, turn * self.scale, spin * self.scale)

    def print_status(self) -> None:
        fwd, turn, spin = self.speeds()
        self.get_logger().info(f'档位 {self.preset} | 前进 {fwd:.2f} m/s | 转向 {turn:.2f} rad/s | 原地转 {spin:.2f} rad/s | 当前 v={self.linear_x:+.2f} w={self.angular_z:+.2f}')

    def publish_cmd(self) -> None:
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular_z
        self.publisher.publish(msg)

    def publish_cmd_now(self) -> None:
        self.publish_cmd()

    def handle_key(self, key: str) -> None:
        fwd, turn, spin = self.speeds()
        changed = True
        if key == 'w':
            self.linear_x = +fwd
        elif key == 's':
            self.linear_x = -fwd
        elif key == 'a':
            self.angular_z = +turn
        elif key == 'd':
            self.angular_z = -turn
        elif key == 'q':
            self.linear_x = 0.0
            self.angular_z = +spin
        elif key == 'e':
            self.linear_x = 0.0
            self.angular_z = -spin
        elif key == ' ':
            self.linear_x = 0.0
            self.angular_z = 0.0
        elif key in ('+', '='):
            self.preset = min(3, self.preset + 1)
            self.get_logger().info(f'加速 -> 档位 {self.preset}')
        elif key == '-':
            self.preset = max(1, self.preset - 1)
            self.get_logger().info(f'减速 -> 档位 {self.preset}')
        elif key in ('1', '2', '3'):
            self.preset = int(key)
            self.get_logger().info(f'档位 -> {self.preset}')
        elif key in ('h', 'H', '?'):
            print(BANNER, flush=True)
        else:
            changed = False
        if changed:
            self.publish_cmd_now()
            self.print_status()

    def stop(self) -> None:
        self.linear_x = 0.0
        self.angular_z = 0.0
        for _ in range(5):
            self.publish_cmd()
            time.sleep(0.02)

def read_key(timeout: float):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if ready:
        return sys.stdin.read(1)
    return None

def main(args=None) -> None:
    rclpy.init(args=args)
    node = WasdqeTeleop()
    if not sys.stdin.isatty():
        node.get_logger().error('当前输入不是交互式终端，读不到按键。\n  请用桌面「① Ubuntu 终端」直接运行，不要通过管道或重定向。')
        node.destroy_node()
        rclpy.shutdown()
        return
    print(BANNER, flush=True)
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        period = 1.0 / node.rate_hz
        while rclpy.ok():
            key = read_key(period * 0.8)
            if key is not None:
                if key == '\x03':
                    break
                node.handle_key(key.lower())
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print('\n已恢复终端设置，正在停车…', flush=True)
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('已退出。', flush=True)
if __name__ == '__main__':
    main()
