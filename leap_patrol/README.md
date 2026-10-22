# leap_patrol：Nav2 多点巡航练习

**当前状态：调度方法已由用户补全，并在同一 Gazebo 会话完成三项基础演示：A → B → C、途中取消、B → A → C。** 详见 [验证记录](VALIDATION.md)。现有实车跟随和旧 Nav2 实验包保持独立。

目标：手动启动后按 A → B → C 导航一轮，最后完成；途中可取消。Nav2 最终返回失败时停止整轮，保留失败点。再次启动从 A 开始。本包只发送 Nav2 Action，不发布 `/cmd_vel`；规划、控制、绕障和恢复仍由 Nav2 提供。

## 先看完整流程

在 Ubuntu 中，安装后运行：

```bash
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
ros2 run leap_patrol patrol_lesson
```

这个小例子不使用 ROS 通信，也不启动仿真。完整源码为 `leap_patrol/lesson.py`，流程是：

```text
发送 A → 等目标受理 → 取得 handle → 等最终结果 → 成功才发送 B
发送 B → 等目标受理 → 取得 handle → 等最终结果 → 成功才发送 C
发送 C → 等目标受理 → 取得 handle → 等最终结果 → 整轮完成
```

`response_future.done()` 表示受理答复到了，`result_future.done()` 表示最终结果到了，两者不是同一件事。例子中的 `spin_once` 只是从 Python 队列取出一个工作执行，用于展示事件顺序，不是 rclpy 的内部实现。真实节点通过注册回调推进，不在服务回调里阻塞等待导航结束。

## 调度核心与学习入口

文件：`~/xuegeros_ws/src/leap_patrol/leap_patrol/mission.py`，方法：`Mission.on_result(self, status)`。

本轮由用户亲自补全 `Mission.on_result`。框架负责启动、复位、目标通信、取消和回调交错；这个方法处理普通的最终结果，使用这些成员：

| 成员 | 含义 |
| --- | --- |
| `self.names` | 按顺序保存的点名，例如 `('A', 'B', 'C')` |
| `self.index` | 当前点下标，从 0 开始；完成后停留在最后一点 |
| `self.completed` | 成功完成的点数，开始时为 0 |
| `self.state` | 当前任务状态 |
| `self.reason` | 失败或取消的说明文字 |

调度方法实现的行为：

1. `SUCCEEDED`：成功数量加一；还有点时，下标前移，保持 `RUNNING`；最后一点完成则设为 `COMPLETED`。
2. `ABORTED` 或 `REJECTED`：设为 `FAILED`，说明中包含当前点名和结果字符串，不前移、不增加成功数量。
3. `CANCELED`：设为 `CANCELED`，不前移。

修改后可立即检查，不用打开 Gazebo：

```bash
ros2 run leap_patrol patrol_check
```

当前代码已通过 5 项调度检查。以后若修改导致检查失败，会显示 `EXERCISE NOT READY` 和具体反馈，退出码为 1，节点也会拒绝开始巡航。逻辑检查不代替导航验收。改 Python 方法后重启巡航节点，以加载新代码。

无 ROS 的电脑也能在本包目录运行：

```bash
python -m leap_patrol.lesson
python -m leap_patrol.exercise_check
```

## 功能包结构

```text
leap_patrol/
├── mission.py          # 用户完成的任务调度，纯 Python
├── client.py           # 异步目标受理、结果和取消协调，纯 Python
├── node.py             # ROS Action、服务和状态话题
├── route.py            # 读取路线文件
├── lesson.py           # 完整小例子，无 ROS
└── exercise_check.py   # 给你的即时反馈，无 ROS
config/route.yaml       # map 坐标下的三个点
launch/patrol.launch.py # 只启动巡航节点，不启动仿真
test/                   # 框架与路线解析测试
```

## 构建

需要 Ubuntu 22.04 / ROS 2 Humble，以及已有 `leap_nav_demo`、TurtleBot3 和 Nav2 环境。首次构建：

```bash
cd ~/xuegeros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select leap_patrol --symlink-install
source install/setup.bash
```

从全新源码工作空间构建时，应先安装依赖，并用 `colcon build --packages-up-to leap_patrol --symlink-install` 一起构建仓库中的 `leap_nav_demo`。本包不导入旧实验的内部脚本，也不启动行为克隆工具。

## 运行仿真

无需开启实物小车、相机或手机热点。所有仿真终端使用：

```bash
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1
```

终端 A 启动一次已有的仿真会话，等 `READY:`，定位与 Nav2 在三个目标之间不重启：

```bash
ros2 run leap_nav_demo run_manual --gui
```

终端 B 启动巡航节点，默认保持 `IDLE`：

```bash
ros2 launch leap_patrol patrol.launch.py
```

终端 C 用标准服务操作：

```bash
# 开始整轮；成功答复只表示启动请求被接受，不代表导航已完成。
ros2 service call /patrol/start std_srvs/srv/Trigger '{}'

# 查询当前点、成功数量、最后的 Nav2 结果。
ros2 topic echo /patrol/status --once --full-length

# 途中取消；等状态从 CANCELING 到 CANCELED，再确认 Gazebo 中已经停下。
ros2 service call /patrol/cancel std_srvs/srv/Trigger '{}'
```

服务不会逐点询问确认，运行中也不需要反复输入。`start` 在运行或取消中会被拒绝；取消、完成、失败后可再启动，从 A 开始。它只取消本节点持有的目标，不取消其他客户端的全部任务。不要同时用 `nav_task goal` 另发目标抢占本轮巡航。

结束时先退出终端 B，再退出终端 A。B 会尝试取消当前目标并最多等待 3 秒；退出进程本身不等于 Nav2 已停。A 沿用原会话的取消和清理流程。若通信异常导致 `FAILED` 且 `goal_unresolved=true`，不会再发新目标，应先用取消接口或关闭整个仿真会话处理，不将这个状态当成已停稳。

## 路线与接口

`config/route.yaml` 中每个点包含 `name/x/y/yaw`，单位为米和弧度，坐标系固定为 `map`。它规定目的地，不规定途经路径。调换列表顺序就是调换巡航顺序，修改后重启巡航节点。

默认配置保持 A → B → C。用户验证过的 B → A → C 另存为 `config/route_bac.yaml`，可通过 launch 的 `route_file` 参数指定其绝对路径；无需重启 Gazebo。点名和坐标应作为完整记录一起移动。

| 接口 | 类型 | 作用 |
| --- | --- | --- |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 调用已有 Nav2 |
| `/patrol/start` | `std_srvs/srv/Trigger` | 从首点开始一轮 |
| `/patrol/cancel` | `std_srvs/srv/Trigger` | 请求取消当前任务 |
| `/patrol/status` | `std_msgs/msg/String`，JSON | 状态、当前点、成功数量、原因和原生结果 |

状态为 `IDLE / RUNNING / CANCELING / COMPLETED / CANCELED / FAILED`。未受理目标、失败的 Action 和成功的 Action 分开处理；取消答复本身不会触发下一个点。

## 已完成的三次基础演示

1. 默认 A → B → C：用户反馈运动符合预期，终端最终为 `COMPLETED`、成功数量 3、当前点 C。
2. 前往 A 时取消：用户反馈停车正常，终端最终为 `CANCELED`、成功数量 0，目标已结束。
3. 用户修改为 B → A → C：反馈顺序符合预期，终端最终为 `COMPLETED`、成功数量 3、当前点 C。

以上依据用户提供的终端输出与人工观察；未做重复成功率或精确停车误差测量。失败分支使用受控结果检查，尚未执行真实 Gazebo 无路可达场景。第三轮 A 到 C 时观察到一次额外原地旋转，原因未确认，本轮保留现象，不调整导航参数。

## 当前验证记录（2026-10-22）

| 检查 | 结果与边界 |
| --- | --- |
| Ubuntu 22.04 / ROS 2 Humble | 新包构建成功；16 项框架与路线解析测试通过 |
| Windows 纯 Python | 14 项通信框架测试及完整小例子通过；未在 Windows 安装 ROS |
| 取消与成功回调交错 | 受控 Future 检查通过；临时移除取消优先逻辑后，回归测试能检出错误，源码未修改 |
| 实际 ROS 节点 | 初始未完成练习时验证过启动拒绝；补全后由用户运行三轮仿真 |
| 默认三个点 | 已检查固定地图上的位置均为空闲栅格；这不证明路线可达或导航成功 |
| 调度与仿真 | 用户完成 `Mission.on_result`，5 项调度检查通过；三轮基础演示的输出与观察见 [验证记录](VALIDATION.md) |

部署前核对了 Ubuntu 的 `leap_nav_demo` 源码与安装入口；本轮未修改现有导航包和实车跟随包。框架测试使用显式结果夹具，不能代替你实现和验证调度函数。

框架回归检查可在 Ubuntu 的本包目录运行 `python3 -m unittest discover -s test -v`，或在工作空间运行 `colcon test --packages-select leap_patrol` 后用 `colcon test-result --test-result-base build/leap_patrol --verbose` 查看报告。

## 来源

导航算法和 Action 复用 [Nav2 Humble](https://github.com/ros-navigation/navigation2/tree/humble)，仿真资源沿用 TurtleBot3。任务层参考已有项目中逐点调度的组织思路，没有复制其动态点队列、自定义消息或重规划观察模块；参考仓库为 [G-Gemmm/RM_LEAP_ROS](https://github.com/G-Gemmm/RM_LEAP_ROS)。
