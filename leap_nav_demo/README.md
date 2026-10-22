# leap_nav_demo

ROS2 Humble 导航主线的独立仿真入口。复用安装版 TurtleBot3 Burger、世界、Gazebo 插件及 Nav2，通过 Simple Commander 完成单目标任务并记录结果。这里验证的是仿真导航，不是 LEAP 的实车动力学。

## 自动导航演示

依赖和本包已经在本机 Ubuntu 中安装、编译。开一个 Ubuntu 终端即可运行：

```bash
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1
ros2 run leap_nav_demo run_demo
```

程序检查端口和隔离域内的已有节点，然后启动 Gazebo、地图服务器、AMCL 和 Nav2。固定初始位姿为 `(-2.0, -0.5, 0)`，目标为 `(-2.0, 1.0, π/2)`，单位为米、弧度。抵达或失败后自动停用 Nav2 并清理本次创建的进程组。墙钟会话上限通常165秒，双任务`blocked_route`为240秒，含启动与任务等待；退出清理最多额外等待约20秒/进程组。不得同时另开本包仿真。

`--gui` 可显示 Gazebo；无障碍、固定障碍及优化版突发障碍演示已由用户反馈确认。突发障碍仍在同参数回归中出现过长恢复，尚不能称为稳定顺畅。导航任务启动后会自动运动，不再等待键盘确认；这是隔离域内的仿真运动。终端显示 `SAVED:` 的目录含：

- `result.json`：任务状态、最终位姿误差、里程计路径长度和停止检查。
- `trajectory.csv`：采样后的里程计位置、地图 TF 位姿及时间。
- `events.jsonl`：任务阶段、全局规划路径。
- `session.json`：软件版本、上游参数哈希、进程退出和清理结果。
- `nav2_params_used.yaml`：本次传给 bringup 的完整参数（之后 bringup 会填入地图路径等启动参数）。
- `behavior_tree_used.xml`：使用本包自定义行为树时保存的副本，其哈希记录在 `result.json`。
- `launch.log`、`task.log`：完整诊断日志。

到达类场景的通过条件包含 Nav2 成功、停车两秒后的地图位姿误差不超过 `0.25m/0.25rad`、确有规划与运动、最终零速命令及新鲜的低速里程计数据。DWB 内部到达容差沿用上游 `xy=0.25m`、`yaw=0.25rad`；RPP 内部 xy 收紧为 `0.20m`，给最终转向期间的定位更新留出余量，外部验收没有放宽。取消与目标占用使用下述独立规则。通过本任务不代表厘米级精度或无碰撞证明。普通单Action场景的任务耗时不含启动与退出，路径长度来自仿真里程计；双Action场景的计时范围见下文。

数据流程：固定地图与雷达供 AMCL 定位，形成 `map → odom → base_footprint`；任务通过 `NavigateToPose` action 发目标；Nav2 规划器产生路径，控制器跟踪路径，速度平滑器向仿真底盘发布 `/cmd_vel`，底盘返回 `/odom` 与 `/scan`。

`navigate_once` 是由 `run_demo` 管理的内部工作进程。上游 Commander 的部分启动/服务调用会阻塞，由父进程提供总时限；日常使用运行 `run_demo`。

## 人工下发、查询与取消

这一入口让同一个仿真和Nav2会话连续接受目标，只在开始时初始化定位。使用安装版RPP控制器，默认READY后保留30分钟，或按Ctrl+C退出。先关闭上一轮仿真，然后在Ubuntu终端A执行：

```bash
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1
ros2 run leap_nav_demo run_manual --gui
```

看到`READY:`后，在终端B也执行同样的两条source和export，再按需要运行：

```bash
# 目标是map坐标，x/y单位米，yaw单位弧度。
ros2 run leap_nav_demo nav_task goal -2 1 1.570796
ros2 run leap_nav_demo nav_task status
ros2 run leap_nav_demo nav_task cancel
# 取消后可以重新下发，定位和Nav2均不重启。
ros2 run leap_nav_demo nav_task goal -2 1 1.570796
```

`goal`返回`accepted: true`及UUID仅表示受理，随后用`status`查看该UUID的`EXECUTING`、`SUCCEEDED`、`CANCELED`或`ABORTED`。这是原生Action状态；Commander把`ABORTED`映射成`FAILED`，不要把两种显示误认为不同故障。持续节点把观察到的状态保留在`/leap_nav_demo/task_history`（标准`GoalStatusArray`，transient-local），让新打开的查询命令也能读取已结束任务；会话结束后查日志。`status`可能包含此前的终止任务；没有观察到UUID也不能证明空闲或停车。

`cancel`取消该隔离域内所有活动的`NavigateToPose`目标，等待被确认取消的UUID进入`CANCELED`，并检查请求之后收到的新零速命令和新鲜低速里程计；超时会报错。没有活动任务时返回`nothing_to_cancel`，不宣称验证了停车。其他类型Action不受这个命令管理。当前没有任务队列；行驶中直接重发新目标会交给Nav2原生抢占机制，本轮只验收取消后再发、到达后再发。

在终端A按Ctrl+C会先尝试取消活动目标、停用Nav2，再清理本次进程组。`--seconds 120`可把READY后的会话预算改为120秒，支持30～3600；启动另有90秒上限。输出保存在`~/leap_nav_data/nav-manual-*`，包括Action UUID/状态变化、完整会话轨迹、规划事件、实际参数、退出和清理结果。它是日常操作入口；固定场景的到达误差和重复评估仍使用`run_demo`/`run_suite`。

主要代码：`nav_task.py`封装原生Action与取消服务，`run_manual.py`管理会话及一次定位初始化，`session_process.py`与自动演示共用隔离检查及进程清理。没有增加训练、网页后台或自定义任务消息。

## 出口封闭后失败，再开门重试

```bash
ros2 run leap_nav_demo run_demo --scenario blocked_route --controller rpp
```

在已完成上述source和隔离环境变量设置的终端运行；可加`--gui`观察。夹具是围住起点的三面固定墙和一面可移除出口，尺寸固定，当前不接受自定义起终点。目标位于封闭区域外，目标本身未被致命占用。先保存原地图和关闭后的实时代价地图，以8邻接、允许未知区域的点网格连通检查确认路径断开，再提交第一次`NavigateToPose`。该次复用`fail_fast.xml`，规划失败直接结束，要求实际`FAILED`、存在规划器失败证据、无非零速度命令且里程计保持静止。

程序随后通过Gazebo删除出口，显式调用`clearAllCostmaps`，等待三面墙仍被感知且出口连通的实时代价地图，再在同一Nav2会话中发送相同目标，使用安装版Nav2默认重规划/恢复行为树。两次Action UUID和结果分别保留，第二次到达仍需正常的最终位置/朝向、零速及新鲜里程计验收。这里验证应用层失败处理和条件恢复后的重新执行，不是自主移除障碍，也不代表行驶中封路后会自动等待恢复。

新增证据为`route-original.json`、`route-closed.json`、`route-opened.json`（完整网格与连通性记录）、两个SDF副本及`retry_behavior_tree_used.xml`。批量入口可运行`run_suite --scenarios blocked_route cancel --controller rpp`，会复算网格证据并显示`FAILED -> SUCCEEDED`，按两棵行为树哈希分别分组。`blocked_route`不进入专家数据采集；点网格连通性不是车体外廓或全场景无碰撞证明。

该场景每个`closed_attempt`/`retry_attempt.task_wall_seconds`仅统计该Action从发送到终止，不含之后两秒停稳观察；顶层`task_wall_seconds`统计从工作进程等待定位到完成两次任务与中间环境处理的整段工作流，含夹具准备和停稳观察，不能直接与普通单目标任务耗时比较。

## 新增固定障碍

已在无 GUI 模式验证的绕行入口：

```bash
ros2 run leap_nav_demo run_demo --scenario obstacle --controller rpp --gui
```

在前述完成 source 和环境变量设置的 Ubuntu 终端运行；去掉 `--gui` 即为无图形模式。用户已反馈固定障碍 RPP 图形演示正常。

障碍是导航开始前生成的静止红色箱体，中心 `(-2.0, 0.25)m`，尺寸 `0.35×0.35×0.50m`；原地图保持不变。程序要求原地图该区域为空闲、实时代价地图出现障碍、箱体接触传感器已输出数据，才发送导航目标。它不是行驶中突然插入障碍的测试。

`--controller dwb` 可复现原 DWB 对照；不带参数仍为无障碍 DWB。两种控制器的地图、目标、外廓半径和期望前进速度上限相同，控制器专属参数不同。RPP 使用官方插件，参数在 `config/rpp.yaml`，未自研规划或控制算法。固定障碍单次对照中，DWB 约 54.5 秒、RPP 约 18.8 秒抵达；DWB 有起步停滞和恢复，RPP 未观察到恢复。不能由此推断一般性能或成功率。

新增验收检查会锁存与箱体的机器人接触；监测缺失、超时或出现接触不能通过。接触监测只覆盖注入箱体，其他场景碰撞未监测；TF 圆形外廓到箱体的最小距离只是采样估计量。接触插件已通过单独的仿真接触正例验证。

## 行驶中插入静止障碍

```bash
ros2 run leap_nav_demo run_demo --scenario sudden_obstacle --controller rpp --gui
```

原地图与初始代价地图中的箱体区域均为空闲。发送目标后，里程计累计路径达到 `0.15m`，且新鲜命令与里程计都显示正在前进，才在同一固定位置生成箱体；触发时估计车体到箱体间隙须至少 `0.25m`，否则中止。箱体生成后保持静止。

额外要求首次规划穿过箱体区域、后续实时代价地图收到障碍，以及在该证据之后收到绕开箱体的规划；接触、停止与进程清理检查继续生效。日志记录从生成请求开始到收到代价地图/新路径的墙钟延迟，不是独立测量的传感器反应时间。

当前RPP配置使用 `lookahead_dist=0.25m`、`max_allowed_time_to_collision_up_to_carrot=2.0s` 和 `rotate_to_heading_min_angle=0.35rad`。较短前视点配合更早的碰撞预测，减少贴近箱角后反复进入恢复；预测仍会被前视点距离截断，2秒不代表固定的前向检查距离。

旧配置约46～48秒抵达并后退约30厘米；最新配置同一场景三轮无GUI运行约24～26秒，恢复反馈最大值为0或1，均未触发等待/后退恢复，未记录箱体接触。障碍、触发条件、速度上限和恢复树相同；具体触发位姿存在差异，少量固定场景运行不能作为通用成功率。失败尝试、轨迹估计与检查定义见接入文档。

后续一次相同参数回归再次耗时约46.9秒、恢复反馈最大9。`ok` 检查到达、停车与场景证据，不表示连续性一定通过；完整记录包含这一反例。

## 取消与目标占用

```bash
ros2 run leap_nav_demo run_demo --scenario cancel --controller rpp --gui
ros2 run leap_nav_demo run_demo --scenario blocked_goal --controller rpp --gui
```

依次单独运行。`cancel` 在累计路径至少0.15m，且新鲜命令与里程计都确认前进后，自动通过Commander发出取消请求。无需键盘输入。要求Nav2返回 `CANCELED`，取消结果与首次停止观测均在请求后2秒内，随后两秒观测中的里程计最大位移不超过5厘米，最终零速命令及新鲜静止里程计也须通过。速度归零同时检查线速度与角速度；异常样本会使取消测量验收失败。坐标基准是请求前最近收到的里程计，并记录其接收年龄，不是独立物理真值。

`blocked_goal` 生成中心 `(-2.0,1.02)m`、尺寸 `0.35×0.10×0.50m` 的薄箱体，使目标点 `(-2.0,1.0)m` 被实时地图占据。要求原地图箱体周围为空闲、目标格代价值至少99、接触监测有效后才发目标。此场景把NavFn目标替代容差设为0，并使用本包 `fail_fast.xml`：一次规划失败就返回。其他场景继续使用原默认恢复树与参数。

目标占用场景预期Nav2返回 `FAILED`；还须收到规划action的 `ABORTED` 状态、10秒内结束、没有非零运动指令且里程计保持静止。若从未接收到命令，报告如实记录 `command_observed=false`，不把未观察到的零速命令写成已收到。缺少占用证据、其他异常或超时都不能算通过。这验证的是已占用目标的失败分支，全道路封闭和替代路线判断尚未验证。

`ok=true` 表示符合该场景的预期；`result` 保留Nav2实际结果，`expected_result` 给出预期，`check_failures` 列出未通过项。因此取消/占用目标正确返回 `CANCELED/FAILED` 时，测试会话可以通过，导航没有被改写成 `SUCCEEDED`。

## 批量评估

起终点也纳入任务配置哈希，不同路线单独统计。旧报告缺少该哈希时标为`legacy-default`，不与新配置混合。

在完成前述source和隔离域设置的终端中运行：

```bash
ros2 run leap_nav_demo run_suite
```

依次自动运行无障碍、固定障碍、突发障碍、取消、目标占用五种场景，无GUI，不需要逐次确认。每次会话仍使用原有时限和退出清理。输出根目录下新增`nav-suite-*`，其中每个`attempt-*`保留原始会话及`runner.log`；`suite.json`记录计划、已尝试次数、验收与失败原因，`attempts.csv`逐次列出结果，`summary.md`用于阅读。

限定场景或重复运行，例如：

```bash
ros2 run leap_nav_demo run_suite --scenarios baseline sudden_obstacle --repetitions 3
```

重复次数可设为1～10，同一场景不要重复列名。批次重新计算场景验收，核对配置快照哈希和清理记录，不只相信原报告的`ok`字段。导航失败但已完成清理时继续收集下一次；缺少会话或清理未确认时停止批次。Ctrl+C请求当前`run_demo`清理，并等待其退出，未确认退出不启动后续会话。批次中断或未完成全部计划不能报告整体通过。

统计按场景、控制器、完整参数哈希、自定义行为树哈希及软件版本分组。正确返回CANCELED或FAILED仍保留实际状态；失败或缺失数据也计入尝试次数。耗时统计仅包含验收通过的运行，报告给出该样本数；恢复反馈最大值另列，不以到达通过代替连续性评价。当前都是固定场景重复，不是随机场景成功率。原始失败记录应和通过结果一起保留。

## 自定义任务与专家数据采集

在已完成source和隔离域设置的终端中，例如运行反向路线：

```bash
ros2 run leap_nav_demo run_demo --scenario baseline --controller rpp \
  --start -2 0.8 -1.5707963267948966 \
  --goal -2 -0.8 -1.5707963267948966 --record-expert
```

`--start/--goal`均为地图坐标`x y yaw`，单位米和弧度。起点同步传入Gazebo实体生成及AMCL初始位姿，包含朝向；最终位置/朝向误差按所给目标计算。`task_config_used.json`保存实际使用的任务配置及哈希；`first_odom_pose`记录最先收到的实际仿真里程计位姿，用于核对生成位置。初始地图TF尚未到达时`localized_initial_map_pose`可能为null，不能把它当作已观察到定位。

自定义配置当前支持`baseline`和`cancel`，其余三种障碍夹具保持固定几何。批量运行使用同名参数，例如`run_suite --scenarios baseline --start ... --goal ... --record-expert`。起终点须至少相隔0.5m；到达类运动证据门槛为`min(0.5m, 0.4×直线间距)`，默认路线仍要求路径超过0.5m。外部位置/朝向验收仍为0.25m/0.25rad。有限坐标不等于空闲或可达，当前仍只验证了有限的左侧走廊路线。

`--record-expert`只增加订阅与文件记录，不发布速度。采集文件包括：

- `expert_inputs.jsonl`：雷达原始束、里程计速度、全局路径，按`kind/id`关联，保留源时间与回调接收墙钟时间。无穷值写作`+inf/-inf`，NaN写作null。
- `expert_samples.jsonl`：每条`/cmd_vel_nav`命令、输入记录ID、源/接收年龄、特征、候选标志与排除原因；另存此前最近收到的执行速度命令，便于区分平滑前后的速度。
- `expert_metadata.json`：特征坐标系、分区中心、时间门槛、路径参考距离与局限。格式2使用每种输入最多64条历史，在已接收到的记录中选源时间最接近但不晚于动作回调ROS时间的一条；雷达还检查末束时间。仿真时钟约0.1秒更新粒度造成的“最新消息时间较新”不会通过放宽时间门槛处理。
- `expert_summary.json`：全部/候选动作数、逐原因排除次数、整段候选标志。多种排除原因可能重叠，不应简单相加当作丢弃条数。

特征包括36个雷达角区最小距离及有限/正无穷/无效束计数、机器人速度、机器人坐标系下目标向量与目标朝向误差、全局路径上约0.35m参考点。雷达分区保留在激光坐标系，并记录激光相对车体变换；正无穷按量程上界作截断记录，与未知束区分，不是无障碍真值。地图到车体变换查询使用被选里程计的源时刻。

仅任务进行中、FollowPath状态执行中、输入时间/坐标/数值均满足检查的记录标为候选。雷达/里程计源年龄上限0.2秒、接收年龄0.4秒；路径对应为2秒/3秒。整段候选还要求任务到达与停车验收通过、未观察到恢复、至少20条候选；20条只是采集检查门槛，不说明训练数据足够。取消、目标拒绝和低覆盖记录仍保存，不强行当成到达示范。

`/cmd_vel_nav`没有源时间戳，回调时间只近似决策时刻；该话题也可由行为节点输出，FollowPath状态仅作来源辅助判断。短历史选取是有界过去观测，不是精确的Nav2内部同步。策略仍需要全局路径输入，目标是局部控制模仿。训练显式选择观测字段，`map_pose_at_odom_stamp`及激光外参仅用于核验，不能自动把整个JSON扁平化成网络输入。训练/验证按整段与任务配置划分，不能随机拆相邻帧。全场景接触真值尚未覆盖，候选数据不等于已认证的无碰撞专家数据集。

## 最小行为克隆实验

`train_bc.py`使用CPU PyTorch训练80→64→32→2的MLP。输入为36个雷达扇区距离、36个有限回波比例、相对目标xy与朝向、里程计速度和前方0.35m路径参考。拟合训练集的均值/标准差，按验证集选择轮次，最后评估测试集；相同任务配置的所有重复记录必须处于同一分区。出现恢复或未完成验收的整段记录保留审计，但不加入学习。

动作输出可用`--output-transform linear`（默认，兼容首轮训练）或`--output-transform sigmoid_tanh`。后者在训练与推理中都将线速度映射到0～0.22m/s、角速度映射到±0.6rad/s，外层限幅和观测保护仍保留。它可以消除负线速度预测，但仍可能输出接近零的速度，不能据此保证任务成功。新模型保存显式输出方式的格式2；当前读取器继续支持原格式1的线性模型，旧读取器会拒绝新格式以免忽略激活函数。

对照实验须固定示范、分区、隐藏层、优化器、随机种子、训练轮数及运行规则，分别按同一验证集选择checkpoint。已经用于诊断、影响模型改进的历史测试任务属于开发评估；后续正式泛化评价需要新的留出任务，不能继续称作盲测。

独立训练环境使用`torch==2.5.1+cpu`；ROS运行时只需NumPy，读取JSON权重，不加载PyTorch或pickle。输出含checkpoint、JSON策略、训练曲线、数据哈希和常量动作基线。示例（manifest需指向完整原始记录）：

```bash
~/leap_bc_venv/bin/python ~/xuegeros_ws/src/leap_nav_demo/scripts/train_bc.py \
  --manifest ~/leap_nav_data/bc-experiment-20261017/manifest.json \
  --output ~/leap_nav_data/bc-new-training

# 有界输出对照：输出目录必须不存在，避免覆盖已有模型。
~/leap_bc_venv/bin/python ~/xuegeros_ws/src/leap_nav_demo/scripts/train_bc.py \
  --manifest ~/leap_nav_data/bc-experiment-20261017/manifest.json \
  --output ~/leap_nav_data/bc-new-bounded-training --output-transform sigmoid_tanh

source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1
ros2 run leap_nav_demo run_demo --controller bc \
  --policy ~/leap_nav_data/bc-experiment-20261017/model-v1-complete/policy.json \
  --start -2 -1.0 -0.2 --goal -2 0.8 1.57079632679
```

BC入口仅接受baseline场景，不接受专家采集标记。它调用Nav2全局规划API，每秒异步更新路径；不提交NavigateToPose/FollowPath任务，也不调用专家恢复。模型以20Hz输出`/cmd_vel_nav`，保留原速度平滑器。动作限制为0～0.22m/s、±0.6rad/s；输入失效或有限雷达回波小于0.20m会停车并终止，所有限幅与保护原因写入`policy_commands.jsonl`。这不是完整碰撞检测，不能据此宣称无碰撞。

BC结果使用独立的`GOAL_REACHED`/`TIMEOUT`/`GUARD_STOP`状态，不能混为Nav2 Action的`SUCCEEDED`；现有`run_suite`继续只评估DWB/RPP。到达还需两秒停车后的位姿误差与新鲜里程计通过检查。离线动作误差小不等于闭环导航成功，同一地图走廊的任务划分不代表未见地图泛化。

共享观测记录`transform_diagnostics`及`odom_selection`。若最新过去Odometry唯一失败原因是对应时刻TF缺失，会在最多64条历史中选择仍满足原0.2s源年龄/0.4s接收年龄限制且精确TF可用的记录；位姿、速度和输入ID同步选取。latest TF仅作为诊断，不替代该时刻位姿；其他无效输入和过期数据仍触发原保护。已有示范/模型不改写，新旧观测实现应与源码快照一起保留。

## 启动

在 Ubuntu 中运行；无需打开小车、相机或手机热点。安装依赖：

```bash
sudo apt-get update
sudo apt-get install --no-install-recommends ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-navigation2
source /opt/ros/humble/setup.bash
cd ~/xuegeros_ws
colcon build --packages-select leap_nav_demo --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1
ros2 launch leap_nav_demo simulation.launch.py
```

以下为仅启动传感器仿真的诊断方式。默认无图形界面；需要观察时，在 Ubuntu 桌面终端给 launch 增加 `gui:=true`。图形界面性能未单独量化。同一虚拟机一次只启动一个本入口。

另开终端，只读检查：

```bash
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1
ros2 run leap_nav_demo check_sim --seconds 15 \
  --output "$HOME/leap_nav_data/check-$(date -u +%Y%m%dT%H%M%SZ).json"
```

检查包括 `/clock`、`/scan`、`/odom`、`odom → base_scan` TF、最近消息到达情况和有效雷达回波。报告记录墙钟接收频率及仿真时间/墙钟时间；不检验导航成功率或碰撞。检查器不发布 `/cmd_vel`。退出检查器不会停止仿真；在 launch 终端按 Ctrl+C 停止。

启动器和检查器都要求 ROS 域 178、仅本机发现，未满足时拒绝运行。这用于避免与本项目默认域的实车节点混用；域号是配置隔离，不能替代实车使用规范。

## 与上游的关系

复用 [ROBOTIS TurtleBot3 Humble 仿真资源](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/humble/turtlebot3_gazebo) 与 [gazebo_ros](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_ros)，上游各自保留其许可证。本包只组合安装版组件，没有复制机器人模型。

本包补充了本机 Gazebo 系统模型路径，避免无地面场景；直接向 `robot_state_publisher` 加载上游 URDF，避免所装 TB3 启动文件为空前缀追加 `/`。使用 Gazebo 11 本地路径，其他发行版需另行适配。

导航复用 [Nav2 Humble bringup](https://github.com/ros-navigation/navigation2/tree/humble/nav2_bringup) 和 [Simple Commander](https://github.com/ros-navigation/navigation2/tree/humble/nav2_simple_commander)。使用当前安装版 Nav2 默认参数，修改圆形外廓半径为 `0.13m`、控制器线速度上限为 `0.22m/s`。半径仅用于当前 Burger 演示。当前安装版 TB3 导航 YAML 含旧节点配置，未直接使用。

当前使用独立节点启动 Nav2，方便诊断与退出；组合容器模式在本机试验中退出挂起。清理代码只管理本次创建的进程组，记录信号升级和残留进程，存在残留则会话判失败。RPP 复用来源见 [Nav2 Humble RPP](https://github.com/ros-navigation/navigation2/tree/humble/nav2_regulated_pure_pursuit_controller)。

开发检查：`python3 -m unittest discover -s src/leap_nav_demo/test -v`（在已 source ROS 的 Ubuntu 工作区根目录）。包含任务配置/哈希、短路径证据、观测时间边界、历史选取、雷达未知值、坐标转换、数据分组泄漏、模型输入与限幅，以及既有导航/取消/占用/批量清理检查；不能替代实际导航和数据核验。

修改Python可执行脚本后须重新运行 `colcon build --packages-select leap_nav_demo --symlink-install`；本机 `install(PROGRAMS ... RENAME ...)` 的安装结果是副本，单改源码不会更新实际入口。目录资源与可执行脚本的安装行为不同，部署后应核对安装文件。

实测记录与下一步范围见项目 `docs/Nav2仿真接入记录.md`。
