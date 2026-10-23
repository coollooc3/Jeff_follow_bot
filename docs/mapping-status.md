# SLAM 建图记录：2026-10-23

目标是用户亲手遥控仿真机器人建立地图并保存，随后用新图定位和导航。SLAM 算法复用已安装的 SLAM Toolbox；用户已完成首次探索、保存、人工定位、单点导航与自建地图上的三点巡航。

已核对 Ubuntu 中安装的 SLAM Toolbox、Gazebo、`teleop_twist_keyboard`、`nav2_map_server` 与 RViz，无需新增依赖。新 `leap_mapping` 包构建成功，YAML/RViz 配置可解析；模拟时钟、坐标系及隔离域检查通过。内存反转域检查条件后，检查能发现错误，正式源码未改变。

一次无 GUI、无遥控的静止仿真检查收到 360 束雷达，生成 80×102 栅格的局部地图，分辨率约 0.05m，`map→base_footprint` TF 连通。检查时 `/cmd_vel` 发布者数量为 0；`map_saver_cli` 成功写出临时地图及元数据，检查进程退出 0 并完成清理。这只验证启动、数据和保存通路，不能作为完整地图质量或运动建图结果。

准备中发现安装版遥控不支持 `key_timeout/repeat_rate`，操作命令已改为其支持的 `speed/turn`；按 `k` 显式停车。用户退出旧导航后仍有该轮孤立子进程，经核对进程命令、用户和域 178 后仅清理该会话，未修改旧导航运行代码。

## 首次人工建图与保存

用户提供 `map_saver_cli` 成功输出及文件列表；原始文件已取回检查并保存在 `leap_mapping/maps/my_first_map_20261023.pgm` 与同名 YAML。图像为 112×103 格，分辨率 0.05m，栅格图覆盖范围为 5.60×5.15m；图像左下角在 `map` 中的位置为 `[-2.96, -2.58, 0]`。这不是机器人起点，也不是定位精度。

PGM 共 11,536 格：已知空闲 7,858 格、占用 741 格、未知 2,937 格；文件长度 11,551 字节，YAML 为 139 字节。保存使用 trinary 模式及默认 free/occupied 阈值 0.25/0.65。图像外观上外围墙体与九个柱状障碍清楚，未见明显大范围重影，部分未观测区域仍为未知；没有与 Gazebo 真值做定量配准或误差测量。

PGM SHA-256：`474e091c3ece11abfa65fd08067a0f39a2935d3d3facf3f917fa27feeb2c80fe`。

YAML SHA-256：`440ac6d7d1589ba207cb04f910d2031f0faafe9a62f05703c4948ddddc55e0e9`。

## 新地图导航入口：静止启动检查

新增 `leap_mapping/launch/navigation.launch.py` 和导航 RViz 配置，直接复用原仿真入口、Nav2 官方 bringup 和现有 RPP 配置。旧 `leap_nav_demo` 运行代码未改动。启动后由用户在 RViz 设置初始位姿、发送目标。

Ubuntu 中构建与启动参数检查通过。一轮无 GUI、无初始位姿、无目标及无速度命令的检查确认：地图 112×103、0.05m；收到 360 束雷达和里程计；地图服务器和 AMCL 的生命周期为 active；`amcl.set_initial_pose=false`；RPP 插件正确，`desired_linear_vel=0.22`。这不证明机器人已完成定位或导航。检查会话以 SIGINT 正常退出，没有残留进程；域保护及内存变异检查通过。

实际 Map Server 加载对照发现：原始 YAML 的 `free_thresh=0.25` 会把灰度 205 的未知区域读为空闲，加载得到空闲 10,795、占用 741、未知 0。另存 `my_first_map_20261023_nav.yaml`，仅改 `free_thresh=0.196`，仍引用同一 PGM；加载恢复空闲 7,858、占用 741、未知 2,937。默认导航入口使用这份 YAML，原始文件及其哈希保持不变。依据是 [Humble 地图读写实现](https://github.com/ros-navigation/navigation2/blob/humble/nav2_map_server/src/map_io.cpp)与本地加载结果。

## 人工定位与单点导航反馈

用户执行新图导航步骤后反馈“没问题符合预期”，据此记录人工初始位姿设置与单点导航流程通过。没有该次完整 Action 终态输出、定位误差或停车误差测量，不扩大为定量精度或长期稳定性结论。

## 自建地图上的多点巡航

新增 `leap_mapping/config/patrol_route.yaml`，按 A → B → C 排列，坐标沿用旧路线，调度仍使用用户补全的 `Mission.on_result`。保存地图中三个点均为灰度 254 的空闲格；至最近非空闲栅格的保守距离下界约为 0.560m、0.647m、0.560m，大于当前 0.13m 的机器人半径。这只是保存栅格上的目标点检查，不能代替在线代价地图检查、路径规划或真实巡航结果。

新图巡航连接当前已定位的 Nav2 会话，不启动旧 `run_manual`。用户亲自启动巡航并提供完整 `/patrol/status` 输出，同时反馈运动符合预期。原样保存的状态字段见[本轮终态](evidence/saved-map-patrol-20261023.json)：

```text
state=COMPLETED, current_waypoint=C, current_index=2
completed=3, total=3, last_nav_status=SUCCEEDED
goal_unresolved=false, active_goal_uuid=null, reason=""
```

据此完成“手动 SLAM 建图 → 保存并重新加载地图 → 人工初始位姿与 AMCL 定位 → 单点导航 → A/B/C 巡航”的一次仿真演示。终态中的 `distance_remaining=0.22499999403953552` 是保留的最后一次导航反馈，不能当成独立测量的最终位置误差。本轮沿用同一 TurtleBot3 Gazebo 场景，没有新增长期成功率、不同场景或实车导航结论。

现有导航、巡航调度与实车跟随代码保持独立。操作见[建图和导航说明](../leap_mapping/README.md)。地图分辨率不能当成定位精度。

## 用户扩展四点路线

用户通过 RViz Publish Point 选取 D，消息使用 `frame_id=map`，位置为 `(2.0116257667541504, -0.8176941275596619, 0.002471923828125)`。二维路线取 `x=2.012, y=-0.818`，另设到达朝向 `yaw=0`，不使用点击位置的 z。保存地图上的 D 点为空闲格，至非空闲栅格的保守距离下界约为 0.321m；该静态检查不等于在线路径可达性保证。

用户在 Ubuntu 编辑 `leap_mapping/config/patrol_route_abcd.yaml`，通过 `route_file` 参数加载它，调度代码和原三点路线均未修改。实际编辑文件已从 Ubuntu 取回，随本记录保存。用户随后提供[完整四点终态](evidence/saved-map-patrol-abcd-20261023.json)：

```text
state=COMPLETED, current_waypoint=D, current_index=3
completed=4, total=4, last_nav_status=SUCCEEDED
goal_unresolved=false, active_goal_uuid=null, reason=""
```

这验证了同一调度实现可通过配置从三点扩展为四点。本轮提供的是终态输出，没有另行提供完整轨迹、逐点原生结果、运动平滑度观察或精确最终位姿测量。`distance_remaining=0.21003150939941406` 仍是保留的最后一次导航反馈，不作为实测停车误差。

## 真实 Nav2 失败结果：地图外目标

用户在 Ubuntu 编辑并运行 `config/patrol_route_failure.yaml`，路线为 A → B_outside → C，第二点坐标为 `(10,10)`。运行前只读检查确认当前全局代价地图边界约为 `[-2.96,2.64) × [-2.58,2.57)`，该点在图外；规划器为 NavFn，目标替代容差 0.5m，使用安装版默认重规划与恢复树。图外目标会在规划器进入替代目标搜索前失败，本轮未修改容差、行为树或调度逻辑。

用户提供两次相同的 FAILED 状态输出，并反馈全部符合预期。保存的一份[终态字段](evidence/saved-map-patrol-failure-20261023.json)为：

```text
state=FAILED, current_waypoint=B_outside, current_index=1
completed=1, total=3, last_nav_status=ABORTED
reason="Failed at B_outside with status ABORTED"
goal_unresolved=false, active_goal_uuid=null
```

按终态与用户观察，本轮 A 成功后，第二点收到 Nav2 ABORTED，整轮失败且没有继续前往 C。两次周期状态不是两轮独立实验。`distance_remaining=0` 不表示抵达图外目标，任务结论依据最终状态。该场景验证真实 Nav2 的无效目标失败处理，没有验证地图内通道被障碍封死或实车绕障；没有单独采集完整 Action/轨迹日志或碰撞记录。
