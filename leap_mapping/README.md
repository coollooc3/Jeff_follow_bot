# leap_mapping：亲手建图与加载新图导航

本包提供两个独立入口：用 SLAM Toolbox 手动建图，以及用保存的地图启动 AMCL + Nav2。复用已有 TurtleBot3 Gazebo 场景和 RPP 配置，遥控、保存地图、设置初始位姿和选择目标均由用户操作。

2026-10-23 用户已亲手遥控探索并保存首张地图，原始 PGM/YAML 保存在 `maps/`。地图文件和元数据已检查，外观上墙体与九个柱状障碍可辨认，没有明显大范围重影。新地图导航入口已构建并通过静止启动检查；用户随后完成了人工定位、单点导航与新图 A → B → C 巡航，反馈符合预期，并提供巡航成功终态。现有 `leap_patrol`、`leap_nav_demo` 和实车跟随代码保持独立。

## 数据怎么流动

```text
Gazebo → /scan + odom→base_footprint TF → SLAM Toolbox → /map + map→odom TF
键盘遥控 → /cmd_vel → Gazebo 底盘
```

SLAM 同时估计位姿并建立地图。`mapping.launch.py` 不加载现成地图，不启动 AMCL、Nav2 或巡航。退出上一轮仿真后再使用；所有终端使用域 178 和本机通信。

## 构建与启动

```bash
cd ~/xuegeros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select leap_mapping --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1
ros2 launch leap_mapping mapping.launch.py
```

一个入口打开 Gazebo 和专用 RViz。开始时只会建立当前位置能看见的局部地图；移动后逐渐扩展。`gui:=false rviz:=false` 可只运行后台仿真和 SLAM。

## 用键盘探索

另开终端 B，加载相同环境后运行：

```bash
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.15 -p turn:=0.5
```

键盘焦点放在终端 B：`i` 前进，`,` 后退，`j/l` 原地左/右转，`k` 停车。安装版遥控按键后保持该方向；松开按键不会自动发停车命令。切换窗口、保存地图或停止观察前先按 `k`。先沿一侧通路探索，再绕过场景中间的障碍，最后回到熟悉区域，观察墙体是否基本对齐。

目标是让空闲通路与主要墙体能在 RViz 中辨认，不要求首次把所有灰色未知区域填满。穿墙、墙体严重重影或断裂时先停车，再检查定位与 TF。此轮不设置自动绕障，遥控时由用户避开障碍。

## 停车并保存地图

在 B 按 `k`，保持建图终端 A 运行。终端 C 加载相同环境，选择一个新的输出文件名：

```bash
mkdir -p ~/leap_nav_data/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/leap_nav_data/maps/my_first_map \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=5.0
ls -lh ~/leap_nav_data/maps/my_first_map.*
```

预期生成 `my_first_map.pgm` 和 `my_first_map.yaml`。PGM 保存占用栅格，YAML 保存图像路径、分辨率、原点等信息，供后续地图服务器使用。再次采集时换文件名，保留前一份结果。

这两个文件用于后续静态地图定位导航；若要下次恢复 SLAM 的完整建图状态，还需另外序列化位姿图，本轮先不扩展。新图的坐标基准以实际建图结果为准，巡航点要在新地图上重新选择。

结束时先退出键盘遥控，再在建图终端 A 按 Ctrl+C；等待 Gazebo 和 SLAM 退出。

## 用保存的地图定位导航

关闭建图会话后，终端 A 执行：

```bash
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1
ros2 launch leap_mapping navigation.launch.py
```

该入口启动同一 Gazebo 场景、地图服务器、AMCL、Nav2 和专用 RViz；不启动 SLAM、巡航或键盘遥控。仿真出生位置沿用 `leap_nav_demo`，世界坐标约 `(-2.0, -0.5)`，朝向世界 +x。地图中的位置要通过雷达与地图匹配确认。

1. 等待 RViz 显示已保存的地图。初始时没有 `map→odom`，机器人或雷达暂时不显示、出现 TF 等待提示是预期现象。
2. 点击工具栏 **2D Pose Estimate**。根据 Gazebo 中的实际位置，在地图左侧通道的相应位置按下鼠标，向右拖出朝向箭头并松开。调整后，红色雷达点应基本贴合墙体和柱子；明显错位时重新设置位姿，暂不发目标。
3. 定位后等待 Nav2 激活。点击 **Nav2 Goal**，在同一通道约一米外的白色空闲区域按下并拖出目标朝向，松开后发送。不要点柱子、墙边或灰色未知区。
4. 观察规划路径、机器人沿路径移动并在目标附近停车。需要取消时点击 **Navigation 2** 面板中的 **Cancel**。结束在终端 A 按 Ctrl+C。

若要查看终态，先在终端 B 执行下面的观察命令，再回到 RViz 发目标：

```bash
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1
ros2 topic echo /navigate_to_pose/_action/status
```

关注本轮目标 UUID：`status: 2` 为执行中、`4` 为成功、`5` 为取消、`6` 为失败。目标受理不等于到达；同时核对实际运动和停车。终端 B 的 Ctrl+C 只停止观察，不取消目标。

```text
PGM/YAML → Map Server → /map ────────────┐
/scan + odom TF + 人工初始位姿 → AMCL → map→odom TF
RViz 目标 → Nav2 规划和控制 → /cmd_vel → Gazebo
```

自定义地图可用 `map:=/绝对路径/地图.yaml`。默认使用 `maps/my_first_map_20261023_nav.yaml`，它与原始 YAML 引用同一 PGM，仅将 `free_thresh` 从 0.25 改为 0.196。实际加载对照确认：原始阈值会将灰度 205 的 2,937 个未知格读成空闲，修正后保留未知格；原始 PGM/YAML 均未改动。导航参数沿用已有 RPP，速度参数 0.22m/s 是命令配置，不是本轮实测速度。

静止检查已确认 112×103 地图、0.05m 分辨率、360 束雷达、地图服务器与 AMCL active、RPP 插件与参数正确；没有发送初始位姿、目标或速度。随后用户反馈上述人工定位与单点导航符合预期；没有提供该目标的完整 Action 终态输出或精确停车误差，不将其作为定量精度证据。

## 在新地图上运行多点巡航

保持上一步已定位的 Gazebo / Nav2 / RViz 会话运行，等待手动目标结束。另开终端 B：

```bash
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1
ros2 launch leap_patrol patrol.launch.py \
  route_file:=$(ros2 pkg prefix --share leap_mapping)/config/patrol_route.yaml
```

这条命令只启动巡航调度，读取本包中与新图配套的 A → B → C 路线；不重启仿真或 AMCL。节点启动后为 IDLE，等待人工开始。`$(...)` 会先取得已安装 `leap_mapping` 的资源目录，再拼成路线文件路径。

终端 C 加载相同环境后操作：

```bash
ros2 service call /patrol/start std_srvs/srv/Trigger '{}'
ros2 topic echo /patrol/status
```

观察顺序 A → B → C；成功时最终应为 `state=COMPLETED`、`completed=3`、`current_waypoint=C`、`goal_unresolved=false`，并确认 Gazebo 中停车。开始服务的成功答复只代表任务启动，不代表到达。

需要途中取消时，在另一个已加载相同环境的终端执行：

```bash
ros2 service call /patrol/cancel std_srvs/srv/Trigger '{}'
```

巡航期间只由巡航节点发送目标，避免再用 RViz 点击目标抢占。结束先退出巡航终端 B，再关闭导航终端 A。保留原 `leap_patrol/config/route.yaml`，本次仅新增新地图配套路线路径，未修改调度实现。

路线点已在保存的地图上检查为空闲且有车体空间；这是静态点位检查。用户随后运行一轮 A → B → C 并反馈符合预期，提供完整终态 `COMPLETED`、`completed=3`、`last_nav_status=SUCCEEDED`、`goal_unresolved=false`、`active_goal_uuid=null`。这是本次同一 Gazebo 场景的单轮演示，没有重复成功率或精确停车误差测量；保留的 `distance_remaining` 是最后一次导航反馈，不作为最终位置误差。

## 用 VS Code 扩展路线

优先在 Ubuntu 的 VS Code 编辑 `~/xuegeros_ws/src/leap_mapping/config/` 中的 YAML。Windows 仓库副本不会自动改变 Ubuntu 中运行的路线。原三点配置保留，四点示例为 `config/patrol_route_abcd.yaml`；D 由用户通过 RViz Publish Point 选取，采用 `x=2.012, y=-0.818, yaw=0`。

保存 YAML 后退出旧巡航节点，保留已经定位的导航会话，再加载新文件：

```bash
ros2 launch leap_patrol patrol.launch.py \
  route_file:=$(ros2 pkg prefix --share leap_mapping)/config/patrol_route_abcd.yaml
```

上面的路径用于构建安装后的资源。刚在 VS Code 新建文件且尚未重新构建时，也可以直接指定源码路径：

```bash
ros2 launch leap_patrol patrol.launch.py \
  route_file:=/home/chuyun/xuegeros_ws/src/leap_mapping/config/patrol_route_abcd.yaml
```

直接读取源码 YAML 不需要重新编译。节点在启动时读取路线，保存文件后需重启巡航节点加载；开始任务仍调用 `/patrol/start`。本次用户提供的四点终态为 `COMPLETED`、`completed=4`、`current_waypoint=D`、`last_nav_status=SUCCEEDED`、`goal_unresolved=false`。调度实现未修改，本轮没有额外运动轨迹或精度测量。

## 验证地图外目标失败

`config/patrol_route_failure.yaml` 保存 A → B_outside → C。第二点 `(10,10)` 位于本次保存地图之外，用于验证真实 Nav2 失败结果能否结束整轮并报告失败点。

在 VS Code 查看或修改 YAML；退出旧巡航节点，保留已定位的导航会话，再加载：

```bash
ros2 launch leap_patrol patrol.launch.py \
  route_file:=$(ros2 pkg prefix --share leap_mapping)/config/patrol_route_failure.yaml
```

新文件尚未构建安装时，`route_file` 也可使用源码文件的绝对路径。调用 `/patrol/start` 后，Nav2 默认恢复树可能先清图、旋转、等待或后退，再报告最终结果。

本次用户提供真实终态 `FAILED`、`current_waypoint=B_outside`、`completed=1`、`last_nav_status=ABORTED`、`reason="Failed at B_outside with status ABORTED"`、`goal_unresolved=false`，并反馈符合预期，未继续执行 C。调度实现、规划容差和默认恢复树均未改动。此例验证地图外无效目标，不作为地图内道路封闭或实车绕障的结果；`distance_remaining=0` 不表示抵达目标。

## 包结构与配置

```text
launch/mapping.launch.py   # 复用仿真，连接 SLAM 和 RViz
launch/navigation.launch.py # 加载自己的地图，等待人工定位和发目标
config/mapping.yaml       # 对已安装 SLAM 默认配置的小幅覆盖
config/patrol_route.yaml  # 与本次保存地图配套的三点路线
config/patrol_route_abcd.yaml # 用户选取 D 后扩展的四点路线
config/patrol_route_failure.yaml # 第二点在图外，用于验证失败停止
rviz/mapping.rviz         # 只显示地图、雷达和机器人
rviz/navigation.rviz      # 地图、雷达、粒子、路径与定位/目标工具
maps/                     # 原始地图及保留未知区域的导航 YAML
CMakeLists.txt            # 安装 launch/config/rviz 等资源
package.xml               # 包名、依赖和构建方式
```

这是以启动文件和配置为主的 `ament_cmake` 包，不需要 C++ 源码或额外 Python 业务节点。CMake 在此负责安装资源。SLAM 的求解器等参数沿用安装版 `mapper_params_online_async.yaml`；本包明确设置 TF、模拟时钟、雷达范围和小车尺度下的更新门槛。配置不是自研 SLAM 算法。

来源：[SLAM Toolbox Humble](https://github.com/SteveMacenski/slam_toolbox/tree/humble)、[Nav2 Humble bringup](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/bringup_launch.py)、[Nav2 地图读写实现](https://github.com/ros-navigation/navigation2/blob/humble/nav2_map_server/src/map_io.cpp)及既有 `leap_nav_demo` 仿真入口。新集成文件使用 MIT 许可，上游组件保留各自许可。
