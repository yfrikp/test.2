# 题目2 · Gazebo 小车仿真 / 全向遥控 / 已知地图寻路 / 雷达建图 / 雷达导航

**姓名：姚锦辉**　学院：集成电路学院　学号：2026215367
申报组别：综合视觉组（摄像头视觉 + ROS2 仿真导航）　邮箱：1558040397@qq.com

运行环境：**Ubuntu 22.04.5 LTS (WSL2) + ROS 2 Humble + Gazebo Sim 6.18 (gz-sim) / Gazebo Classic 11 + Nav2**

> 本仓库只放**题目2**的内容。题目1（红/蓝/红蓝物块识别与定位）在另一个仓库：`https://github.com/yfrikp/test.1`。

---

## 一、题目要求与实现对照

| 小题 | 分值 | 要求 | 实现 | 状态 |
|---|---|---|---|---|
| （1） | 15 | gazebo sim 建小车 + 脚本 `wasdqe` **全向**控制 | 四轮麦轮底盘 `omni_bot`（`MecanumDrive` + `VelocityControl`）、`wasdqe_teleop` 节点（20 Hz 发布 `/cmd_vel`，A/D 横移、Q/E 自转） | ✅ 6/6 方向实测通过 |
| （2） | 35 | 自建 ≥2 种障碍地图，已知地图信息，A→B 无碰撞曲线运动 | 自建 `my_map`（四周墙体 + 中央长条块 + 四角圆柱，2 类障碍）、`astar.py` 全局规划 + `path_follower.py` Pure-Pursuit 跟踪 | ✅ 实测到达，最小间距 0.26 m |
| （3） | 20 | 雷达建出质量良好的地图 | `slam_toolbox`（online async）+ `mapping_patrol` 自动绕场巡逻 | ✅ 214×201 栅格，墙体闭合 |
| 附加 | 20 | 雷达导航，A→任意点无碰撞 | Nav2（SmacPlanner2D + RegulatedPurePursuit + 双层 costmap）+ `nav_goal_demo` | ✅ 3/3 目标点到达 |

---

## 二、编译

```bash
source /opt/ros/humble/setup.bash
cd ~/vision_ws
rosdep install --from-paths src --ignore-src -r -y   # 首次
colcon build --symlink-install
source install/setup.bash
```

> WSL2 下 Gazebo 必须软件渲染，所有 launch 已默认 `LIBGL_ALWAYS_SOFTWARE=1`（见"踩坑"第 1 条）。

---

## 三、启动命令

### （1）15 分 · 小车建模 + wasdqe 全向控制

```bash
# 终端 A：起麦轮全向底盘（空场地）
ros2 launch diff_bot_description omni_sim.launch.py

# 终端 B：键盘遥控（必须交互式终端，别用管道）
ros2 run diff_bot_control wasdqe_teleop --ros-args -p mode:=omni
```

| 按键 | 动作 | 说明 |
|---|---|---|
| `W` / `S` | 前进 / 后退 | `linear.x` |
| `A` / `D` | **左横移 / 右横移** | `linear.y`（全向底盘独有，差速底盘做不到） |
| `Q` / `E` | 原地左转 / 右转 | `angular.z` |
| `空格` | 急停 | 三个速度全清 |
| `1/2/3` | 速度档位 | 0.12 / 0.25 / 0.45 m/s |

全向是怎么实现的（`diff_bot_description/urdf/omni_bot.urdf.xacro`）：

- **四个麦轮** 由 gz-sim 的 `MecanumDrive` 系统按 `vx/vy/wz` 算转速，同时发布 `/odom` 与 TF；
- 圆柱轮在接触模型里产生不了侧向力（实测横移位移为 0），所以横移再叠一层 `VelocityControl`：
  把 `base_link` 的平面速度直接置为指令值；
- `diff_bot_control/omni_bridge.py` 负责 `世界系 = R(yaw)·车体系` 的换算，保证"按 W 永远朝车头方向"。

一条命令同时起仿真和遥控：

```bash
ros2 launch diff_bot_control omni_teleop.launch.py
```

### （2）35 分 · 已知地图 A→B 曲线避障

```bash
ros2 launch diff_bot_control gz_follow_ab_mymap.launch.py
```

可调参数：`lookahead:=0.45`（前视距离）、`v_max:=0.22`（巡航速度）、`start_delay:=30`（等仿真起来）、`gui:=false`（无界面）。

### （3）20 分 · 雷达 SLAM 建图

```bash
# 手动遥控绕场
ros2 launch diff_bot_control gz_slam.launch.py patrol:=false

# 或自动巡逻一圈（推荐，可复现）
ros2 launch diff_bot_control gz_slam.launch.py patrol:=true
```

建完在 rviz 里 `SlamToolbox → Save Map`，或命令行保存：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/vision_ws/models/map_gz_clean
```

### 附加 20 分 · 雷达导航到任意点

```bash
ros2 launch diff_bot_control gz_navigation.launch.py map:=$HOME/vision_ws/models/map_gz_clean.yaml

# 另开终端：连续给多个目标点 (x, y, yaw_deg)
ros2 run diff_bot_control nav_goal_demo --ros-args -p goals:="[4.31, -1.37, 0.0, 5.52, 0.23, 0.0, 1.37, -1.07, 0.0]"
```

rviz 里也可以直接用 `2D Goal Pose` 点任意点。

---

## 四、实测数据（2026-11-18 本机 WSL2，无 GUI 复跑）

### （1）全向控制 6 个方向

```bash
python3 tools/verify_omni.py      # 用 Gazebo 世界真值位姿判定，不用 /odom 自证
```

| 指令 | 位移 | 判定 |
|---|---|---|
| `linear.x=+0.25` ×3s | Δx **+0.759 m** | ✅ |
| `linear.x=-0.25` ×3s | Δx **-0.758 m** | ✅ |
| `linear.y=+0.25` ×3s | Δy **+0.729 m**（Δx ≈ 0） | ✅ 横移 |
| `linear.y=-0.25` ×3s | Δy **-0.728 m**（Δx ≈ 0） | ✅ 横移 |
| `angular.z=+0.8` ×1.5s | Δyaw **+71.2°** | ✅ |
| `angular.z=-0.8` ×1.5s | Δyaw **-71.2°** | ✅ |

**6/6 通过**，日志见 `docs/evidence/omni_check.log`。

### （2）A→B 避障

| 指标 | 实测 |
|---|---|
| 规划路径长度 | **6.172 m**（A(-2,-2) → B(2,2)，绕开中央障碍块） |
| 全程距最近障碍最小距离 | **0.259 m**（>0，全程不碰撞） |
| 到达精度 | 终点 (+2.016, +1.853)，距 B **0.15 m** |
| 用时 | 约 31 s（v_max 0.22 m/s） |
| 越界剔除点 | 0 个 |

日志：`docs/evidence/A2B_run.log`

### （3）建图质量

| 指标 | 实测 |
|---|---|
| 栅格 | 214 × 201 @ 0.05 m/格（10.70 m × 10.05 m） |
| 占用 / 空闲 / 未知 | **4.4% / 53.1% / 42.5%**（未知主要在墙外，符合预期） |
| 墙体 | 四周墙体连续闭合（地图系相对世界系旋转 45°，因为起点朝向 45°）；中央长条障碍与四角圆柱均可辨认；墙外为未知区 |

预览图：`results/map_gz_clean_preview.png`

### 附加题导航

| 目标点 (map) | 结果 | 用时 |
|---|---|---|
| (+4.31, -1.37) | ✅ 到达 | 34.3 s |
| (+5.52, +0.23) | ✅ 到达 | 16.9 s |
| (+1.37, -1.07) | ✅ 到达 | 57.4 s |

**3/3 成功**，日志：`docs/evidence/nav_goals.log`

> 目标点不是手拍的，是脚本从地图里按"可达连通域 + 离障碍 ≥0.45 m + 彼此分散 ≥1.8 m"选出来的
> （`tools/pick_goals.py`），所以能保证是任意可达点而不是碰巧能到的点。

---

## 五、目录结构

```
vision_ws/
├── src/
│   ├── diff_bot_description/          车辆模型、仿真世界、rviz、launch
│   │   ├── urdf/diff_bot.urdf.xacro   两轮差速底盘（小题 2/3/附加题用）
│   │   ├── urdf/omni_bot.urdf.xacro   四轮麦轮全向底盘（小题 1 用）
│   │   ├── worlds/my_map.world        自建地图（Gazebo Classic 版）
│   │   ├── worlds/my_map_gz.sdf       同一张地图的 gz-sim 版
│   │   └── launch/                    gz_sim / omni_sim
│   └── diff_bot_control/
│       ├── diff_bot_control/
│       │   ├── wasdqe_teleop.py       小题 1：键盘遥控（diff / omni 两种模式）
│       │   ├── omni_bridge.py         小题 1：车体系 → 世界系速度换算
│       │   ├── astar.py               小题 2：栅格 A* 全局规划
│       │   ├── path_follower.py       小题 2：Pure-Pursuit 跟踪 + 安全校验
│       │   ├── world_map.py           小题 2：从 .world 解析障碍真值
│       │   ├── mapping_patrol.py      小题 3：自动巡逻建图
│       │   ├── scan_filter.py         滤除车体自反射激光点
│       │   └── nav_goal_demo.py       附加题：多点导航
│       ├── launch/                    gz_follow_ab_mymap / gz_slam / gz_navigation / omni_teleop
│       └── config/                    nav2_params_gz.yaml / mapper_params_online_async.yaml
├── models/                            建好的地图（map_gz_clean.pgm / .yaml）
├── results/                           地图预览等成图
├── tools/                             verify_omni.py / pick_goals.py / map_preview.py
└── docs/evidence/                     各小题的运行日志（验收证据）
```

---

## 六、踩过的坑

1. **WSL 硬件渲染会崩** — `D3D12: Removing Device.` 之后 `gzserver` 段错误退出，必须 `LIBGL_ALWAYS_SOFTWARE=1`，代价是仿真相机约 12 fps。
2. **URDF 里不能有中文** — `robot_description` 带中文会让 `spawn_entity` 失败，已过滤非 ASCII。
3. **激光自反射** — 车体会被自己的雷达扫到，`scan_filter.py` 按车体半径滤除（一趟建图滤掉约 3 万个点）。
4. **odom 与 world 坐标系换算** — `path_follower` 用 `odom_is_world: false` + 出生点换算，启动时自动校验（实测偏差 0.000 m）。
5. **圆柱轮不能横移** — 麦轮必须靠滚筒才能产生侧向力，纯圆柱轮 + 各向异性摩擦实测横移为 0（`docs/evidence/` 里留了对照实验的结论），所以小题 1 叠了 `VelocityControl` 做运动学全向。
6. **`VelocityControl` 别写 `<link_name>`** — 写死 `base_link` 会让它每帧刷 "Failed to find link"，去掉后按模型 canonical link 解析，日志干净。
7. **导航目标点必须落在可达连通域里** — 贴着墙（如 y=-2.4 靠南墙）的目标点会落在膨胀区，规划必然失败。
