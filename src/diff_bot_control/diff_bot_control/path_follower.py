import math
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from diff_bot_control.astar import astar, catmull_rom, max_curvature, path_length, shorten_from_obstacles, simplify_collinear
from diff_bot_control.world_map import build_map, min_clearance, parse_world

def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

class PathFollower(Node):

    def __init__(self) -> None:
        super().__init__('path_follower')
        self.declare_parameter('world_file', '')
        self.declare_parameter('spawn_x', -2.0)
        self.declare_parameter('spawn_y', -2.0)
        self.declare_parameter('spawn_yaw', 0.785)
        self.declare_parameter('odom_is_world', True)
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 2.0)
        self.declare_parameter('lookahead', 0.45)
        self.declare_parameter('v_max', 0.22)
        self.declare_parameter('v_min', 0.06)
        self.declare_parameter('w_max', 1.5)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('slow_radius', 0.6)
        self.declare_parameter('brake_seconds', 3.0)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('inflate_radius', 0.25)
        self.declare_parameter('map_size', 7.0)
        gp = self.get_parameter
        self.spawn = (gp('spawn_x').value, gp('spawn_y').value, gp('spawn_yaw').value)
        self.odom_is_world = bool(gp('odom_is_world').value)
        self.goal = (gp('goal_x').value, gp('goal_y').value)
        self.L = float(gp('lookahead').value)
        self.v_max = float(gp('v_max').value)
        self.v_min = float(gp('v_min').value)
        self.w_max = float(gp('w_max').value)
        self.goal_tol = float(gp('goal_tolerance').value)
        self.slow_radius = float(gp('slow_radius').value)
        self.brake_seconds = float(gp('brake_seconds').value)
        self.rate = float(gp('control_rate').value)
        world_file = self._locate_world(gp('world_file').value)
        self.get_logger().info(f'加载地图: {world_file}')
        obstacles, info = parse_world(world_file)
        self.obstacles = obstacles
        self.get_logger().info(f"解析到 {info['obstacles']} 个障碍 (模型 {info['models']} 个，其中 {info['with_collision']} 个碰撞体)")
        size = float(gp('map_size').value)
        res = float(gp('resolution').value)
        inflate_r = float(gp('inflate_radius').value)
        raw, inflated = build_map(obstacles, width_m=size, height_m=size, resolution=res, origin=(-size / 2.0, -size / 2.0), inflate_radius=inflate_r)
        self.get_logger().info(f'栅格地图: {raw.width}x{raw.height} 格 @ {res} m/格，障碍膨胀 {inflate_r} m')
        path = astar(inflated, self.spawn[:2], self.goal)
        if path is None:
            self.get_logger().error('A* 找不到从 A 到 B 的路径！检查起终点是否被障碍堵死')
            raise RuntimeError('规划失败')
        self.get_logger().info(f'A* 搜到 {len(path)} 个路点')
        ctrl = simplify_collinear(path)
        curve = catmull_rom(ctrl, samples_per_seg=25, alpha=0.5)
        self.get_logger().info(f'简化到 {len(ctrl)} 个控制点，平滑成 {len(curve)} 个点')
        curve, removed = shorten_from_obstacles(obstacles, curve, required_clearance=0.0)
        clearance = min((min_clearance(obstacles, x, y) for x, y in curve))
        self.path = curve
        self.get_logger().info(f'路径长度 {path_length(curve):.3f} m，最大曲率 {max_curvature(curve):.3f} 1/m')
        self.get_logger().info(f'★ 安全校验：全程距最近障碍最小 {clearance:.3f} m (>0 即不碰撞)，剔除越界点 {removed} 个')
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.path_pub = self.create_publisher(Path, 'planned_path', path_qos)
        self.publish_path()
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose = None
        self.idx = 0
        self.done = False
        self.started = False
        self.stop_deadline = 0.0
        self._last_log = 0.0
        self.timer = self.create_timer(1.0 / self.rate, self.control_loop)
        self.get_logger().info(f'路径跟踪已启动: A{self.spawn[:2]} → B{self.goal}, 前视距离 {self.L} m, 速度 {self.v_max} m/s, 控制频率 {self.rate:.0f} Hz')
        self.get_logger().info('等待 /odom ...（Gazebo 里看到小车后就会开始走）')

    def _locate_world(self, explicit: str) -> str:
        if explicit:
            return explicit
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('diff_bot_description')
            return f'{share}/worlds/obstacles.world'
        except Exception:
            pass
        return '/mnt/c/Users/Lenovo/dsh-workspace/vision_task2/src/diff_bot_description/worlds/obstacles.world'

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        ox, oy = (float(p.x), float(p.y))
        oyaw = yaw_from_quaternion(msg.pose.pose.orientation)
        sx, sy, syaw = self.spawn
        if self.odom_is_world:
            x, y, yaw = (ox, oy, oyaw)
        else:
            c, s = (math.cos(syaw), math.sin(syaw))
            x = sx + c * ox - s * oy
            y = sy + s * ox + c * oy
            yaw = normalize_angle(syaw + oyaw)
        self.pose = (x, y, yaw)
        if not self.started:
            self.started = True
            d = math.hypot(x - sx, y - sy)
            mode = 'odom 即世界坐标' if self.odom_is_world else 'odom 相对出生点（已换算）'
            if d > 0.3:
                self.get_logger().warn(f'⚠ 坐标系校验不通过：换算后起点 ({x:+.2f},{y:+.2f}) 与预期出生点 ({sx:+.2f},{sy:+.2f}) 相差 {d:.2f} m。当前模式是「{mode}」—— 很可能是 odom_is_world 填反了！')
            else:
                self.get_logger().info(f'✔ 坐标系校验通过（模式：{mode}）：起点 ({x:+.2f},{y:+.2f}) ≈ 出生点 ({sx:+.2f},{sy:+.2f})，偏差仅 {d:.3f} m')
            self.get_logger().info(f'开始行驶。当前位置 ({x:+.3f}, {y:+.3f})，朝向 {math.degrees(yaw):.1f}°')
            self.publish_path()

    def control_loop(self) -> None:
        if self.pose is None:
            return
        if self.done:
            now = self.get_clock().now().nanoseconds / 1000000000.0
            if now < self.stop_deadline:
                self.cmd_pub.publish(Twist())
            return
        x, y, yaw = self.pose
        gx, gy = self.goal
        remain = math.hypot(gx - x, gy - y)
        if remain < self.goal_tol:
            self.finish('已到达 B 点')
            return
        target = None
        i = self.idx
        while i < len(self.path):
            px, py = self.path[i]
            if math.hypot(px - x, py - y) >= self.L:
                target = (px, py)
                self.idx = i
                break
            i += 1
        if target is None:
            target = self.path[-1]
        dx = target[0] - x
        dy = target[1] - y
        c, s = (math.cos(yaw), math.sin(yaw))
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        dist2 = local_x * local_x + local_y * local_y
        if dist2 < 1e-06:
            return
        curvature = 2.0 * local_y / dist2
        omega = self.v_max * curvature
        bend = min(1.0, abs(local_y) / max(self.L, 1e-06))
        v = self.v_max - (self.v_max - self.v_min) * bend
        if remain < self.slow_radius:
            v = min(v, max(0.04, self.v_max * remain / self.slow_radius))
        omega = max(-self.w_max, min(self.w_max, omega))
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)
        now = self.get_clock().now().nanoseconds / 1000000000.0
        if now - self._last_log > 2.0:
            self._last_log = now
            remain = math.hypot(gx - x, gy - y)
            clear = min_clearance(self.obstacles, x, y)
            self.get_logger().info(f'位置 ({x:+.2f},{y:+.2f}) 朝向 {math.degrees(yaw):+6.1f}° | v={v:.2f} ω={omega:+.2f} | 距B {remain:.2f} m | 距最近障碍 {clear:+.3f} m')

    def finish(self, reason: str) -> None:
        if self.done:
            return
        self.done = True
        now = self.get_clock().now().nanoseconds / 1000000000.0
        self.stop_deadline = now + self.brake_seconds
        self.cmd_pub.publish(Twist())
        x, y, _ = self.pose
        clear = min_clearance(self.obstacles, x, y)
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'✅ {reason}')
        self.get_logger().info(f'   最终位置 ({x:+.3f}, {y:+.3f})，目标 ({self.goal[0]:+.3f}, {self.goal[1]:+.3f})')
        self.get_logger().info(f'   距最近障碍 {clear:+.3f} m（>0 表示全程无碰撞）')
        self.get_logger().info('=' * 60)

    def publish_path(self) -> None:
        sx, sy, syaw = self.spawn
        c, s = (math.cos(-syaw), math.sin(-syaw))
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for wx, wy in self.path:
            if self.odom_is_world:
                px, py = (wx, wy)
            else:
                dx, dy = (wx - sx, wy - sy)
                px, py = (c * dx - s * dy, s * dx + c * dy)
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(px)
            ps.pose.position.y = float(py)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = PathFollower()
    except Exception as e:
        print(f'启动失败: {e}')
        rclpy.shutdown()
        return
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
