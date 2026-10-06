import math
import os
import rclpy
import rclpy.time
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a
DEFAULT_ROUTE = [-2.6, -2.6, -2.6, 2.6, 2.6, 2.6, 2.6, -2.6, -2.6, -2.6]

class MappingPatrol(Node):

    def __init__(self) -> None:
        super().__init__('mapping_patrol')
        self.declare_parameter('route', DEFAULT_ROUTE)
        self.declare_parameter('odom_is_world', True)
        self.declare_parameter('spawn_x', -2.0)
        self.declare_parameter('spawn_y', -2.0)
        self.declare_parameter('spawn_yaw', 0.785)
        self.declare_parameter('v_max', 0.28)
        self.declare_parameter('w_max', 1.2)
        self.declare_parameter('wp_tolerance', 0.2)
        self.declare_parameter('turn_threshold_deg', 20.0)
        self.declare_parameter('k_yaw', 1.8)
        self.declare_parameter('k_dist', 1.5)
        self.declare_parameter('enable_safety', False)
        self.declare_parameter('safety_distance', 0.7)
        self.declare_parameter('safety_half_angle_deg', 25.0)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('brake_seconds', 3.0)
        self.declare_parameter('track_file', '')
        self.declare_parameter('track_ground_truth', False)
        gp = self.get_parameter
        flat = list(gp('route').value)
        self.route = [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat) - 1, 2)]
        self.odom_is_world = bool(gp('odom_is_world').value)
        self.spawn = (float(gp('spawn_x').value), float(gp('spawn_y').value), float(gp('spawn_yaw').value))
        self.v_max = float(gp('v_max').value)
        self.w_max = float(gp('w_max').value)
        self.wp_tol = float(gp('wp_tolerance').value)
        self.turn_thresh = math.radians(float(gp('turn_threshold_deg').value))
        self.k_yaw = float(gp('k_yaw').value)
        self.k_dist = float(gp('k_dist').value)
        self.safety_d = float(gp('safety_distance').value)
        self.enable_safety = bool(gp('enable_safety').value)
        self.safety_half = math.radians(float(gp('safety_half_angle_deg').value))
        self.rate = float(gp('control_rate').value)
        self.brake_seconds = float(gp('brake_seconds').value)
        self.pose = None
        self.wp_idx = 0
        self.done = False
        self.started = False
        self.stop_deadline = 0.0
        self.front_min = float('inf')
        self.front_angle = 0.0
        self._last_log = 0.0
        self._t0 = None
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.route_pub = self.create_publisher(Path, 'patrol_route', path_qos)
        self.publish_route()
        self.timer = self.create_timer(1.0 / self.rate, self.control_loop)
        self.track_file = str(gp('track_file').value)
        self.track_gt = bool(gp('track_ground_truth').value)
        self.gt_pose = None
        self._track_f = None
        self._track_buf = None
        self._track_lis = None
        self._n_track = 0
        self._last_track = None
        self._last_still = None
        if self.track_file:
            self._track_buf = Buffer()
            self._track_lis = TransformListener(self._track_buf, self)
            if self.track_gt:
                from tf2_msgs.msg import TFMessage
                self.create_subscription(TFMessage, 'gz_ground_truth', self._on_gt, 50)
            try:
                self._track_f = open(self.track_file, 'w')
                self._track_f.write('t,x_map,y_map,yaw_map,x_world,y_world\n' if self.track_gt else 't,x_map,y_map,yaw_map\n')
                self._track_f.flush()
                self.get_logger().info(f'地图坐标轨迹将记录到 {self.track_file}' + ('（含世界坐标真值）' if self.track_gt else ''))
            except OSError as e:
                self.get_logger().warn(f'打不开轨迹文件 {self.track_file}: {e}')
                self._track_f = None
        self.get_logger().info(f'巡逻路线共 {len(self.route)} 个路点: ' + ' → '.join((f'({x:+.1f},{y:+.1f})' for x, y in self.route)))
        self.get_logger().info(f'巡逻速度 {self.v_max} m/s，前向急停阈值 {self.safety_d} m')
        self.get_logger().info('等待 /odom ...')

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        ox, oy = (float(p.x), float(p.y))
        oyaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.odom_is_world:
            self.pose = (ox, oy, oyaw)
        else:
            sx, sy, syaw = self.spawn
            c, s = (math.cos(syaw), math.sin(syaw))
            self.pose = (sx + c * ox - s * oy, sy + s * ox + c * oy, normalize_angle(syaw + oyaw))
        if not self.started:
            self.started = True
            self.get_logger().info(f"开始巡逻。起点 ({self.pose[0]:+.2f}, {self.pose[1]:+.2f}) [{('odom 即世界坐标' if self.odom_is_world else 'odom 相对出生点，已换算')}]")

    def on_scan(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        if n == 0:
            return
        best = float('inf')
        best_a = 0.0
        for i in range(n):
            a = msg.angle_min + i * msg.angle_increment
            if abs(normalize_angle(a)) > self.safety_half:
                continue
            r = msg.ranges[i]
            if msg.range_min < r < msg.range_max and r < best:
                best = r
                best_a = math.degrees(normalize_angle(a))
        self.front_min = best
        self.front_angle = best_a

    def _on_gt(self, msg) -> None:
        for t in msg.transforms:
            if t.child_frame_id == 'diff_bot':
                p = t.transform.translation
                self.gt_pose = (p.x, p.y)
                return

    def _record_track(self, now: float) -> None:
        if self._track_f is None or self._track_buf is None:
            return
        try:
            tr = self._track_buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception as e:
            self._track_err = getattr(self, '_track_err', 0) + 1
            if self._track_err in (1, 100):
                self.get_logger().warn(f'第 {self._track_err} 次取 map→base_footprint 失败: {type(e).__name__}: {e}')
            return
        p = tr.transform.translation
        q = tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self._last_track is not None:
            if math.hypot(p.x - self._last_track[0], p.y - self._last_track[1]) < 0.03:
                return
        if self.track_gt:
            if self._last_still is None:
                self._last_still = (p.x, p.y, now)
            else:
                lx, ly, lt = self._last_still
                if math.hypot(p.x - lx, p.y - ly) > 0.03:
                    self._last_still = (p.x, p.y, now)
                    return
                if now - lt < 0.6:
                    return
        self._last_track = (p.x, p.y)
        self._n_track += 1
        try:
            if self.track_gt:
                if self.gt_pose is None:
                    return
                self._track_f.write(f'{now:.3f},{p.x:.4f},{p.y:.4f},{yaw:.4f},{self.gt_pose[0]:.4f},{self.gt_pose[1]:.4f}\n')
            else:
                self._track_f.write(f'{now:.3f},{p.x:.4f},{p.y:.4f},{yaw:.4f}\n')
            self._track_f.flush()
        except OSError:
            pass

    def control_loop(self) -> None:
        if self.pose is None:
            return
        now = self.get_clock().now().nanoseconds / 1000000000.0
        self._record_track(now)
        if self.done:
            if self._track_f is not None:
                self._track_f.close()
                self._track_f = None
            if now < self.stop_deadline:
                self.cmd_pub.publish(Twist())
            return
        if self._t0 is None:
            self._t0 = now
        x, y, yaw = self.pose
        tx, ty = self.route[self.wp_idx]
        dx, dy = (tx - x, ty - y)
        dist = math.hypot(dx, dy)
        if dist < self.wp_tol:
            self.get_logger().info(f'✔ 到达路点 {self.wp_idx + 1}/{len(self.route)} ({tx:+.2f}, {ty:+.2f})，用时 {now - self._t0:.1f}s')
            self.wp_idx += 1
            if self.wp_idx >= len(self.route):
                self.finish()
            return
        target_yaw = math.atan2(dy, dx)
        err = normalize_angle(target_yaw - yaw)
        cmd = Twist()
        cmd.angular.z = max(-self.w_max, min(self.w_max, self.k_yaw * err))
        if abs(err) > self.turn_thresh:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = min(self.v_max, self.k_dist * dist)
            if self.enable_safety and self.front_min < self.safety_d:
                cmd.linear.x = 0.0
                if now - self._last_log > 2.0:
                    self.get_logger().warn(f'⚠ 前方 {self.front_min:.2f} m 有障碍（角度 {self.front_angle:+.0f}°，0°=正前方），暂停前进（仍在转向）')
        self.cmd_pub.publish(cmd)
        if now - self._last_log > 3.0:
            self._last_log = now
            pct = 100.0 * self.wp_idx / len(self.route)
            self.get_logger().info(f'巡逻中 {pct:5.1f}% | 位置 ({x:+.2f},{y:+.2f}) 朝向 {math.degrees(yaw):+6.1f}° | 去路点 {self.wp_idx + 1} 还差 {dist:.2f} m | 前方 {self.front_min:.2f} m')

    def finish(self) -> None:
        if self.done:
            return
        self.done = True
        now = self.get_clock().now().nanoseconds / 1000000000.0
        self.stop_deadline = now + self.brake_seconds
        self.cmd_pub.publish(Twist())
        x, y, _ = self.pose
        self.get_logger().info('=' * 62)
        self.get_logger().info('✅ 巡逻完成，全场覆盖结束')
        self.get_logger().info(f'   回到起点附近 ({x:+.3f}, {y:+.3f})')
        self.get_logger().info(f'   总用时约 {now - (self._t0 or now):.1f} 秒')
        self.get_logger().info('   现在可以去终端存档地图：')
        self.get_logger().info('     ros2 run nav2_map_server map_saver_cli -f ~/vision_ws/models/map')
        self.get_logger().info('=' * 62)

    def publish_route(self) -> None:
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.route:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.route_pub.publish(msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()
