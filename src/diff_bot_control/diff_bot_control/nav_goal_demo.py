import math
import os
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
DEFAULT_GOALS = [2.0, 2.0, 45.0, 2.3, -2.3, -90.0, -2.3, 2.3, 135.0, 0.0, -2.4, 0.0, 2.4, 0.0, 0.0, -2.0, -2.0, 45.0]

class NavGoalDemo(BasicNavigator):

    def __init__(self):
        super().__init__(node_name='nav_goal_demo')
        self.track = []
        self.create_subscription(Odometry, 'odom', self._on_odom, 20)
        self.declare_parameter('goals', DEFAULT_GOALS)
        self.declare_parameter('goal_timeout', 120.0)
        self.declare_parameter('track_file', '/tmp/nav_track.csv')

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.track.append((p.x, p.y))

    def make_pose(self, x: float, y: float, yaw_deg: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        yaw = math.radians(float(yaw_deg))
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def run(self) -> int:
        goals_flat = list(self.get_parameter('goals').value)
        goals = [(float(goals_flat[i]), float(goals_flat[i + 1]), float(goals_flat[i + 2])) for i in range(0, len(goals_flat) - 2, 3)]
        timeout = float(self.get_parameter('goal_timeout').value)
        track_file = self.get_parameter('track_file').value
        self.get_logger().info(f'共 {len(goals)} 个目标点，等待 Nav2 激活...')
        self.waitUntilNav2Active()
        self.get_logger().info('Nav2 已激活，开始依次导航')
        results = []
        for idx, (x, y, yaw_deg) in enumerate(goals, 1):
            self.get_logger().info('=' * 62)
            self.get_logger().info(f'▶ 目标 {idx}/{len(goals)}: ({x:+.2f}, {y:+.2f}) 朝向 {yaw_deg:+.0f}°')
            t0 = time.time()
            self.goToPose(self.make_pose(x, y, yaw_deg))
            timed_out = False
            while not self.isTaskComplete():
                if time.time() - t0 > timeout:
                    self.get_logger().warn(f'  ⏱ 超过 {timeout:.0f}s，取消本目标')
                    self.cancelTask()
                    timed_out = True
                    break
                fb = self.getFeedback()
                if fb is not None and idx == 1:
                    self.get_logger().info(f'    剩余 {fb.distance_remaining:.2f} m', throttle_duration_sec=5.0)
                time.sleep(0.2)
            dt = time.time() - t0
            res = self.getResult()
            if timed_out:
                self.get_logger().error(f'  ❌ 超时未到达，用时 {dt:.1f}s')
                results.append(('超时', x, y, dt))
            elif res == TaskResult.SUCCEEDED:
                self.get_logger().info(f'  ✅ 到达，用时 {dt:.1f}s')
                results.append(('到达', x, y, dt))
            elif res == TaskResult.CANCELED:
                self.get_logger().warn(f'  ⚠ 已取消，用时 {dt:.1f}s')
                results.append(('取消', x, y, dt))
            else:
                self.get_logger().error(f'  ❌ 失败，用时 {dt:.1f}s')
                results.append(('失败', x, y, dt))
        self.get_logger().info('=' * 62)
        self.get_logger().info('导航结果汇总:')
        ok = 0
        for i, (st, x, y, dt) in enumerate(results, 1):
            self.get_logger().info(f'  {i}. ({x:+.2f},{y:+.2f})  {st}  {dt:.1f}s')
            ok += st == '到达'
        self.get_logger().info(f'  成功 {ok}/{len(results)}')
        try:
            with open(track_file, 'w') as f:
                f.write('x,y\n')
                for x, y in self.track:
                    f.write(f'{x:.4f},{y:.4f}\n')
            self.get_logger().info(f'  轨迹已保存: {track_file}（{len(self.track)} 个点）')
        except OSError as e:
            self.get_logger().warn(f'  轨迹保存失败: {e}')
        return 0 if ok == len(results) else 1

def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavGoalDemo()
    try:
        rc = node.run()
    except KeyboardInterrupt:
        rc = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(rc)
if __name__ == '__main__':
    main()
