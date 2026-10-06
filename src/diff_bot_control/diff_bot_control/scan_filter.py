import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

class ScanFilter(Node):

    def __init__(self):
        super().__init__('scan_filter')
        self.declare_parameter('in_topic', '/scan_raw')
        self.declare_parameter('out_topic', '/scan')
        self.declare_parameter('min_valid_range', 0.6)
        self.declare_parameter('frame_id', 'laser_link')
        gp = self.get_parameter
        self.min_r = float(gp('min_valid_range').value)
        self.frame_id = str(gp('frame_id').value)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, gp('in_topic').value, self.cb, qos)
        self.pub = self.create_publisher(LaserScan, gp('out_topic').value, 10)
        self.n_filtered = 0
        self.n_total = 0
        self._last_log = 0.0
        self._src_frame = None
        self.get_logger().info(f"雷达过滤器已启动: {gp('in_topic').value} → {gp('out_topic').value}，滤除 < {self.min_r} m 的读数")
        if self.frame_id:
            self.get_logger().info(f'同时把 frame_id 改写为 "{self.frame_id}"')

    def cb(self, msg: LaserScan) -> None:
        if self.frame_id:
            if self._src_frame is None:
                self._src_frame = msg.header.frame_id
                self.get_logger().info(f'原始 frame_id = "{self._src_frame}" → 改写为 "{self.frame_id}"')
            msg.header.frame_id = self.frame_id
        r = list(msg.ranges)
        n = 0
        for i, v in enumerate(r):
            if math.isfinite(v) and v < self.min_r:
                r[i] = float('inf')
                n += 1
        self.n_filtered += n
        self.n_total += 1
        msg.ranges = r
        self.pub.publish(msg)
        now = self.get_clock().now().nanoseconds / 1000000000.0
        if now - self._last_log > 10.0:
            self._last_log = now
            if self.n_filtered:
                self.get_logger().info(f'已滤除 {self.n_filtered} 个自反射点（共 {self.n_total} 帧，本次 {n} 个）')
            else:
                self.get_logger().info(f'暂未发现自反射点（共 {self.n_total} 帧）')

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanFilter()
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
