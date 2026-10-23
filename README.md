# Jeff Follow Bot

基于 LEAP 小车和 ROS 2 Humble 的 ArUco 标记跟随项目。在宿舍这类空间有限的环境中，用户手动开启跟随后，小车持续跟随缓慢移动的标记，在约 25 cm 的车头间隔处停止前进；用户关闭后保持停止。

**当前进展：2026-10-22 完成首轮宿舍场景人工实车验收。** 本次二次开发的主要入口是 [`leap_marker_follow`](leap_marker_follow/README.md)，仓库同时保留了原工程的提交历史和基础代码。

## 已实现的行为

| 场景 | 小车行为 |
| --- | --- |
| 启动节点 | 默认关闭跟随，等待用户开启 |
| 标记向远处缓慢移动 | 连续前进并纠正朝向，可以同时输出前进和转向速度 |
| 接近教定的标记大小 | 停止前进，用回差减少临界位置的频繁启停 |
| 少量帧漏检 | 降低前进速度；超过目标超时后输出零速度 |
| 标记或输入恢复 | 保持开启时，满足条件后自动继续跟随 |
| 雷达停止区域被占用 | 停车等待，当前不执行绕障 |
| 用户关闭 | 发送零速度，保持关闭，不因标记重新出现而自动启动 |

当前配置的前进速度上限为 **0.10 m/s**，转向上限为 **0.60 rad/s**。这些是命令限值，不是实测峰值速度。目标为 `DICT_4X4_50` 字典中的 **ID0** 标记，移动时保持标记大致正对相机、位于视野内。

## 实现思路

相机检测标记四角后，计算归一化边长和水平偏移：水平偏移用于纠正朝向，标记大小与教定基准的比例用于控制前进。滤波、启停回差和速度变化率限制用于减少检测波动带来的频繁启停。

这里采用视觉大小基准：第一次使用时，将标记面放到车头最前缘前约 25 cm 并保存基准。程序不依赖完整相机标定，也不把标记大小换算成精确距离。标记大小、相机视角或分辨率改变后需要重新采集基准。

```text
/camera/image_raw/compressed → ArUco 检测 → 连续跟随控制 ─┐
/scan + /odom ────────────────────────→ 输入与停障检查 ─┤→ /cmd_vel
用户启停 + 视觉基准 ────────────────────────────────────┘
```

## 代码结构

```text
leap_marker_follow/
├── leap_marker_follow/
│   ├── node.py          # ROS 话题、服务与控制循环
│   ├── controller.py    # 不依赖 ROS 的跟随控制逻辑
│   ├── reference.py     # 视觉基准采集与保存
│   └── safety.py        # 输入有效性与雷达停止区域检查
├── config/follow.yaml   # 本轮实车使用的启动参数
├── launch/             # ROS 2 launch 入口
├── resource/           # ament 功能包注册标记
└── test/               # 控制逻辑、输入检查和基准读写测试
```

根目录的 `xuegecar_*`、`slam_gmapping`、`openslam_gmapping` 等目录来自继承的工程。它们的存在不代表本项目已经完成了这些功能的整车集成与验证。

## 在已有 LEAP 环境中运行

已使用的环境为 Ubuntu 22.04、ROS 2 Humble，以及带 `cv2.aruco` 的 OpenCV。需要 NumPy；本轮相机环境中的 OpenCV 版本为 4.13.0。

**当前仓库尚未包含完整整车运行环境。** 相机桥接、车端 micro-ROS Agent、当前使用的雷达和底盘接口等仍需沿用已有 LEAP 环境。将本仓库中的 `leap_marker_follow` 目录放入已有工作空间的 `src/`，只构建这个包。

### 1. 启动基础数据，再启动跟随节点

先按设备原有流程启动 Agent、相机及车端数据，确保存在以下输入：

| 话题 | 消息类型 |
| --- | --- |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |
| `/scan` | `sensor_msgs/msg/LaserScan` |
| `/odom` | `nav_msgs/msg/Odometry` |

跟随节点启动入口不会自动启动这些基础节点。运行跟随时，关闭其他会发布 `/cmd_vel` 的遥控、导航或旧跟随节点。

```bash
cd ~/xuegeros_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
colcon build --packages-select leap_marker_follow --symlink-install
source install/setup.bash
ros2 launch leap_marker_follow follow.launch.py
```

以上使用当前实车的 ROS domain；换用其他基础环境时，各终端应采用相同的 domain 设置。

### 2. 首次采集约 25 cm 的视觉基准

保持小车静止、跟随关闭。将 ID0 标记大致正对相机、放在画面中央，**从车头最前缘量到标记面约 25 cm**，静置约一秒。

另开 Ubuntu 终端执行；后续启停命令也在这个已加载环境的终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
ros2 service call /marker_follow/capture_reference std_srvs/srv/Trigger '{}'
```

返回 `success=True` 后，基准保存到 `~/.ros/leap_marker_follow/reference.json`，后续启动可以复用。不同相机或标记不应直接复用别人的基准文件。

### 3. 开启、跟随和关闭

让标记保持可见、完成确认后开启：

```bash
ros2 service call /marker_follow/enable std_srvs/srv/SetBool '{data: true}'
```

之后缓慢移动标记，小车持续跟随，无需在移动过程中反复输入命令。结束时关闭：

```bash
ros2 service call /marker_follow/enable std_srvs/srv/SetBool '{data: false}'
```

需要观察状态或图像时：

```bash
ros2 topic echo /marker_follow/status --once --full-length
ros2 run rqt_image_view rqt_image_view
```

在 rqt 中选择 **`/marker_follow/debug`**。这个话题直接发布原始 `Image`；无需选择 `/compressed`。

参数说明、暂停原因及更多运行细节见[功能包说明](leap_marker_follow/README.md)。

## 已有验证与适用范围

- 软件侧：已在 ROS 2 Humble 中完成该包构建，18 项逻辑测试通过。
- 实车侧：2026-10-22 完成一次人工验收流程，包括开启、缓慢直行与左右移动、短时遮挡恢复、关闭后保持停止。现场反馈整体符合预期，最终车头到标记面粗量约 **24.5 cm**。
- `24.5 cm` 是一次人工粗量读数，不能据此声称厘米级定位精度、重复停车精度或长期稳定性。

本版本支持前进、转向和停车，没有自动倒车、主动绕障或丢失目标后的搜索导航。二维雷达检查覆盖其扫描平面；低矮障碍和图传中断仍是实车使用时需要注意的条件。

纯逻辑测试不需要启动小车，也不依赖 ROS：

```bash
cd ~/xuegeros_ws/src/leap_marker_follow
python3 -m unittest discover -s test -v
```

## Nav2 多点巡航仿真

新增 [`leap_patrol`](leap_patrol/README.md)，在 TurtleBot3、Gazebo 和固定地图中练习 A → B → C 的任务调度，支持人工开始、途中取消和失败后结束整轮。导航算法、定位、控制和恢复复用 Nav2；巡航节点只调用 `NavigateToPose`，不发布速度。

**2026-10-22 已完成三轮基础仿真演示。** 用户亲自补全 `Mission.on_result`，执行 A → B → C、途中取消，以及改为 B → A → C 后完成。Ubuntu 中 16 项框架与路线解析检查、5 项调度检查通过。三轮终端结果与人工观察见[验证记录](leap_patrol/VALIDATION.md)；这些结果不代表长期稳定性或实车导航已验证。

[`leap_nav_demo`](leap_nav_demo/README.md) 保存本次从 Ubuntu 实际工作空间核对的现有仿真后端，使用入口为 `run_manual --gui`，沿用 RPP 配置。该包还保留历史实验工具；当前巡航不调用其中的行为克隆工具。实车跟随版本独立保留，本轮不需要小车、相机或热点。

学习入口保留 `patrol_lesson` 的完整小例子和 `patrol_check` 的即时反馈。默认路线保持 A → B → C，B → A → C 另存示例配置。第三轮观察到一次额外原地旋转，原因未确认，本轮没有为此调整参数。构建与命令见[巡航说明](leap_patrol/README.md)。

## 手动 SLAM 建图

[`leap_mapping`](leap_mapping/README.md) 提供独立建图入口，复用 TurtleBot3 Gazebo 和已安装的 SLAM Toolbox。用户通过键盘探索、保存自己的 PGM/YAML 地图；另有新地图导航入口，启动 AMCL + Nav2，等待用户在 RViz 设置初始位姿、选择目标。建图和定位导航分别启动。

2026-10-23 已完成用户手动建图、保存并重新加载地图、人工初始位姿与 AMCL 定位、单点导航，以及自建地图上的 A → B → C 巡航。用户反馈运动符合预期，巡航终态为 `COMPLETED`、3 / 3、`last_nav_status=SUCCEEDED`、`goal_unresolved=false`。地图为 112×103 格、分辨率 0.05m；导航使用单独 YAML 保留未知区域，原图未改动。本轮是同一 Gazebo 场景的一次人工演示，未量化建图或定位误差。详见[建图记录](docs/mapping-status.md)。现有跟随及巡航版本保持独立。

## 来源与许可

本项目从虚拟机中已有的 LEAP 工程继续开发，原仓库地址记录为 [czulcl/leap1](https://gitee.com/czulcl/leap1)。本轮新增的跟随逻辑、配置、ROS 接口和测试位于 `leap_marker_follow/`，不将继承的底盘、SLAM 或导航代码作为新增实现。

继承工程保留根目录的 [LICENSE](LICENSE)；新跟随、巡航和建图包分别保留自己的许可：[leap_marker_follow/LICENSE](leap_marker_follow/LICENSE)、[leap_patrol/LICENSE](leap_patrol/LICENSE)、[leap_mapping/LICENSE](leap_mapping/LICENSE)。上游组件保留各自的来源和许可。
