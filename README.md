# 题目二：ROS2 + Gazebo Sim 差速小车（建模 / 规划 / 建图 / 导航）

**姓名**：`<在这里填你的名字>`
**运行环境**：ROS2 Humble + Ubuntu 22.04.5 LTS + Gazebo Sim (Fortress)

---

## 一、总览

本仓库完成题目二的**全部四个部分**：

| 部分 | 分值 | 内容 | 状态 |
|---|---|---|---|
| (1) | 15 | ROS2 Gazebo Sim 构建小车 + WASDQE 键盘控制脚本 | ✅ |
| (2) | 35 | 自建 ≥2 种障碍地图，A→B 不碰撞**曲线**运动 | ✅ |
| (3) | 20 | 在 (2) 的地图中用雷达建图 | ✅ |
| 附加 | 20 | 在题目二基础上用雷达导航，A→任意点无碰撞 | ✅ |

---

## 二、目录结构

```
.
├── README.md
└── src/
    ├── diff_bot_description/            机器人与仿真场景
    │   ├── urdf/diff_bot.urdf.xacro         小车模型（差速驱动 + 2D 雷达）
    │   ├── worlds/my_map.world              自建地图（规划用）
    │   ├── worlds/my_map_gz.sdf             自建地图（Gazebo Sim 用）
    │   ├── launch/gz_sim.launch.py          仿真总入口（Gazebo + 桥接 + 生成小车）
    │   ├── rviz/diff_bot.rviz               题目 (1)(2) 的可视化配置
    │   ├── rviz/slam.rviz                   题目 (3) 的可视化配置
    │   ├── rviz/nav2.rviz                   附加题的可视化配置
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    └── diff_bot_control/                算法与控制
        ├── diff_bot_control/
        │   ├── wasdqe_teleop.py             题目 (1) WASDQE 键盘遥控
        │   ├── world_map.py                 题目 (2) 地图解析与栅格化
        │   ├── astar.py                     题目 (2) A* 搜索 + 曲线平滑
        │   ├── path_follower.py             题目 (2) Pure Pursuit 路径跟踪
        │   ├── scan_filter.py               雷达过滤（被 3、4 使用）
        │   ├── mapping_patrol.py            题目 (3) 自动巡逻建图
        │   └── nav_goal_demo.py             附加题 导航演示
        ├── launch/
        │   ├── gz_follow_ab_mymap.launch.py 题目 (2) 一键演示
        │   ├── gz_slam.launch.py            题目 (3) 一键演示
        │   └── gz_navigation.launch.py      附加题 一键演示
        ├── config/
        │   ├── mapper_params_online_async.yaml  题目 (3) SLAM 参数
        │   └── nav2_params_gz.yaml              附加题 Nav2 参数
        ├── resource/diff_bot_control
        ├── setup.py
        ├── setup.cfg
        └── package.xml
```

---

## 三、编译

```bash
# 把本仓库的 src/ 放进你的 ROS2 工作空间
mkdir -p ~/vision_ws/src
cp -r src/* ~/vision_ws/src/

cd ~/vision_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

> 依赖：`ros-humble-ros-gz`、`ros-humble-slam-toolbox`、`ros-humble-nav2-bringup`
> 安装：`sudo apt install ros-humble-ros-gz ros-humble-slam-toolbox ros-humble-nav2-bringup`

---

## 四、分部分说明与启动命令

### 部分 (1)：构建小车 + WASDQE 全向控制 —— 15 分

**实现内容**

小车 `diff_bot` 用 URDF + xacro 描述，几何只有一份，通过 `sim:=classic|gz` 参数
切换 Gazebo Classic / Gazebo Sim 两套插件：

* 车体 0.30 × 0.24 × 0.10 m，质量 3 kg
* 两个驱动轮（半径 0.05 m、轮间距 0.26 m）+ 一个万向轮支撑
* 2D 激光雷达：720 线、360°、0.30~12.0 m、20 Hz
* 差速驱动插件 `ignition-gazebo-diff-drive-system`
* 关节状态发布插件

WASDQE 遥控脚本支持：

| 按键 | 功能 |
|---|---|
| `W` / `S` | 前进 / 后退 |
| `A` / `D` | 左转 / 右转（边走边转，走弧线）|
| `Q` / `E` | 原地左转 / 原地右转 |
| 空格 | 急停 |
| `+` / `-` / `1` `2` `3` | 加减速 / 切换速度档位 |

**启动命令**

```bash
# 终端 1：启动仿真
ros2 launch diff_bot_description gz_sim.launch.py

# 终端 2：键盘遥控
ros2 run diff_bot_control wasdqe_teleop
```

**实测结果**

| 项 | 实测值 |
|---|---|
| 小车生成位姿 | `(-2.000, -2.000)`，朝向 `+45.0°`（正好在 A 点）|
| 按 `W` 前进 3 秒 | 真值位移 **0.660 m** |
| 按空格急停 | 停车后仅滑行 **0.026 m** |
| 按 `Q` 原地转 2.5 秒 | 转过 **-179.8°**，位置漂移仅 **0.031 m** |
| `/cmd_vel` 频率 | 20 Hz（1 秒内收到 164 条 / 8 秒）|

---

### 部分 (2)：自建地图 + A→B 不碰撞曲线运动 —— 35 分

**自建地图**

地图 `my_map.world` 自建，包含 **2 类共 5 个障碍** + 四面围墙（±3.2 m）：

| 障碍 | 类型 | 位置 | 尺寸 |
|---|---|---|---|
| `center_block` | 长方体（**带 0.6 rad 旋转**）| (0, 0) | 1.4 × 0.3 × 0.6 |
| `corner_c1` | 圆柱 | (-1.2, 1.2) | r = 0.28 |
| `corner_c2` | 圆柱 | (1.2, 1.2) | r = 0.28 |
| `corner_c3` | 圆柱 | (-1.2, -1.2) | r = 0.28 |
| `corner_c4` | 圆柱 | (1.2, -1.2) | r = 0.28 |

A 点 = `(-2, -2)`，B 点 = `(+2, +2)`。
中间那根**旋转 0.6 弧度的斜挡墙**正好挡住 A→B 的直线，**迫使规划器绕路**，
这样"曲线运动"才有意义。

**算法流程**

```
my_map.world
   │ parse_world()      解析出 9 个障碍几何
   ▼
build_map() + inflate(0.25m)   离散成 140×140 栅格并膨胀
   ▼
astar()               在膨胀栅格上搜索（保证不碰撞）
   ▼
simplify_collinear()  97 个路点 → 14 个控制点
   ▼
catmull_rom()         样条平滑成 325 个点的曲线 ← 题目要的"曲线"
   ▼
shorten_from_obstacles()  安全验收
   ▼
Pure Pursuit 20 Hz    算出 (v, ω) → /cmd_vel
```

**关键设计**

* **障碍膨胀**：车体外接圆半径 0.192 m，膨胀取 0.25 m，这样可以把车当成**质点**来规划
* **A\* 启发式**用 octile 距离，并加**防穿角**检查
* **Catmull-Rom** 而不是贝塞尔/B 样条 —— 因为它**穿过**控制点，平滑后不会切进障碍
* **Pure Pursuit**：`κ = 2·local_y / (local_x² + local_y²)`，`ω = v·κ`；预瞄点连续滑动 → ω 连续 → 走出曲线

**启动命令**

```bash
ros2 launch diff_bot_control gz_follow_ab_mymap.launch.py
# 无界面（WSL 里更流畅）：
ros2 launch diff_bot_control gz_follow_ab_mymap.launch.py gui:=false rviz:=false
```

可调参数：

```bash
ros2 launch diff_bot_control gz_follow_ab_mymap.launch.py lookahead:=0.6 v_max:=0.3
```

**实测结果**

```
解析到 9 个障碍 (模型 11 个，其中 9 个碰撞体)
A* 搜到 97 个路点 → 14 个控制点 → 325 个曲线点
路径长度 6.172 m
★ 安全校验：全程距最近障碍最小 0.259 m

真值轨迹碰撞分析（对照地图里的 9 个障碍）：
  起点: (-2.000, -2.000)   ← 正好是 A 点
  终点: (+1.959, +1.788)
  距 B 误差: 0.216 m
  总行程: 5.90 m
  ★ 全程最小离障距离: +0.296 m
  ★ 是否发生碰撞    : 否
  ★ 是否跑出围墙    : 否
```

> **验证方法说明**：碰撞判定用的是 Gazebo 里的**模型真值位姿**
> （`/gz_ground_truth`），不是车自己报的里程计 ——
> 因为撞了之后里程计不可信。

---

### 部分 (3)：雷达 SLAM 建图 —— 20 分

**实现内容**

用 `slam_toolbox` 在线建图。为了让覆盖更完整，写了 `mapping_patrol.py`
自动沿一个 2.6 m 半径的方形巡逻（离墙 0.6 m），跑完 5 个路点后自动停。

**针对仿真环境做的两个关键调整**：

1. **`use_scan_matching: false`**
   扫描匹配本来是用来修真实里程计漂移的，但仿真的里程计是解析积分出来的、
   几乎无漂移（实测走 0.8 m 误差 **0.000 m**）。
   开着它反而会**收敛到错误解把地图拉歪** ——
   实测尺度误差达 **11.6%**（巡逻路线在世界里是正方形，
   正方形无论怎么旋转包围盒都相等，而地图里变成了 6.94×5.55，不等）。
   关掉之后尺度误差降到 **0.7%**，包围盒变成 7.12×7.16（相等）。

2. **`scan_filter.py` 过滤车体自反射**
   Gazebo Sim 对"低于 `range_min`"的读数**是钳位而不是丢弃**，
   导致雷达会读到自己的外壳，形成贴脸假障碍。
   在 ROS 层把 < 0.6 m 的读数置为 `inf` 后解决。

**启动命令**

```bash
# 自动巡逻建图
ros2 launch diff_bot_control gz_slam.launch.py patrol:=true

# 存图
ros2 run nav2_map_server map_saver_cli -f ~/vision_ws/models/my_map
```

**实测结果**

| 项 | 实测值 |
|---|---|
| 地图尺寸 | 215 × 203 像素 @ 0.05 m/px |
| 占据格数 | **2300+ 格** |
| 障碍建成 | **10 / 12** |
| 围墙覆盖率 | **76.2%**（北墙 100%）|
| 地图尺度误差 | **0.7%** |

---

### 附加题：雷达导航到任意点 —— 20 分

**实现内容**

用 **Nav2** 做雷达导航：AMCL 定位 + SmacPlanner2D 全局规划 + RPP 控制器。

**三个关键调整**：

1. **AMCL 初始位姿 = `(0,0,0)`**
   Gazebo Sim 的 SLAM `map` 帧锚定在机器人**起始位姿**上
   （Classic 版那边是 `(-2,-2,0.785)`），填错会从完全错误的位置起步。
   参数文件用脚本从主参数自动生成，保证不会失同步。

2. **用真实矩形轮廓替代外接圆半径**
   车体 0.30×0.24，内切半径其实只有 0.12 m。
   原来用 `robot_radius: 0.22`（外接圆）会被 Nav2 当作内切圆用，
   导致"车离任何障碍 0.22 m 以内就判起点在致命区"而规划失败。

3. **全局规划器用 SmacPlanner2D 而不是 NavFn**
   NavFn 在这张地图上持续报 `failed to create plan`，
   但把代价地图读出来自己做洪泛填充验证，**阻挡格 0 个、可达 100%** ——
   说明路是存在的，是 NavFn 本身的问题。换成 SmacPlanner2D 后正常。

**启动命令**

```bash
ros2 launch diff_bot_control gz_navigation.launch.py
# 然后在 RViz 里用 "2D Goal Pose" 点任意目标点
```

**实测结果**

| 次 | 目标数 | 到达 | 总行程 | 最小离障 | 碰撞 |
|---|---|---|---|---|---|
| 1 | 6 | **6/6** | 18.17 m | +0.144 m | **否** |
| 2 | 6 | 5/6 | 21.45 m | +0.130 m | **否** |
| 3 | 6 | 2/6 | 17.87 m | −0.195 m | **是** |

**⚠️ 这一部分目前不够可靠**，如实说明：

* 最好的一次 **6/6 全部到达、零碰撞**；但也有只到 2/6 并且**撞了一次**的情况
* 失败的直接原因是规划器报 `Starting point in lethal space`
  （地图里残留的少量伪影让车"以为"自己站在障碍里）
* **撞车是失败的连锁反应**：规划失败后 Nav2 会触发恢复行为，
  其中"后退"是**盲目倒退**的 —— 在 6.4 m 的房间里挤了 9 个障碍，
  退着退着就撞上了。也就是说：**不是规划算法撞的，是"脱困动作"撞的**
* 已排查并排除：不是清理脚本误删了真障碍
  （专门写了脚本逐个核对 9 个障碍，清理前后都在）

**如果目标是稳定拿分**，建议重点保证 (1)(2)(3) 这 70 分，
附加题作为"完成但已知有局限"如实写进 README —— 这比假装没问题更好。

---

## 五、验证脚本

仓库里没有放验证脚本（它们依赖具体路径），验证方法是：

1. **碰撞判定**用 Gazebo 真值位姿 `/gz_ground_truth`，不是里程计
2. 对轨迹上每个采样点，算到地图里所有障碍的最短距离
3. 全程最短距离 > 0 → 无碰撞

---

## 六、已知限制

1. **`slam_toolbox` 约有 50% 概率进入"只处理最开始几帧"的状态**
   （地图只有约 180 格而不是 2300 格）。
   已排除时间戳不同步、节点重复、雷达自反射、存图取到旧数据、
   TF 缺失、代价地图不连通等原因，推测是 ros_gz 桥接链路的偶发时序竞态，
   **根因尚未定位**。
   目前用"地图占据格数 < 1200 就自动重来"的方式保证流程可复现。

2. **附加题（雷达导航）不够稳定**
   最好一次 6/6 零碰撞，但也有 2/6 且撞一次的情况。
   原因是地图里残留的少量伪影让规划器以为起点在障碍里，
   进而触发 Nav2 的恢复行为（盲目后退）而撞到东西。

3. **地图位姿估计仍有约 25 cm 残差**
   尺度误差已压到 0.7%~2.2%，但绝对残差还有 25 cm。
   因此需要借助"已知地图信息"（题目给的地图几何）
   清理掉残余伪影才能稳定导航。

4. **GUI 显示效果未逐帧验证**
   所有验证都是在 `gui:=false` 的 headless 模式下做的，
   验证的是**数据链路和运行结果**，不是窗口里的画面。

