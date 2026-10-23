# 多点巡航基础验证：2026-10-22

环境：Ubuntu 22.04 / ROS 2 Humble、TurtleBot3 Gazebo、固定地图、现有 Nav2 RPP 配置；`ROS_DOMAIN_ID=178`、`ROS_LOCALHOST_ONLY=1`。本轮没有接入实物小车。

用户补全 `Mission.on_result`，亲自启动仿真、调用启停接口并修改路线。功能包与异步通信框架由助手提供；规划、定位、控制与恢复复用 Nav2。

## 三轮人工演示

证据来源为用户在同一开发会话中粘贴的 `/patrol/status` 完整输出及运动观察。以下不是自动化评测报告，也没有视频或精确位姿误差测量。

| 演示 | 最终状态 | 当前点 / 下标 | 成功数量 | Nav2 结果 | 未结束的目标 | 用户观察 |
| --- | --- | --- | --- | --- | --- | --- |
| A → B → C | COMPLETED | C / 2 | 3 / 3 | SUCCEEDED | false | 运动符合预期 |
| 前往 A 时取消 | CANCELED | A / 0 | 0 / 3 | CANCELED | false | 取消与停车无异常 |
| B → A → C | COMPLETED | C / 2 | 3 / 3 | SUCCEEDED | false | 顺序符合预期；A 到 C 时有一次额外原地旋转 |

三次输出均为 `exercise_ready=true`、`active_goal_uuid=null`。取消原因是 `operator_cancel`；两次完成的原因字段为空。

输出中的 `distance_remaining` 分别为 `0.2826339304447174`、`3.1957099437713623`、`0.17071028053760529`。这是保留的最后一次 Nav2 反馈，不是本轮独立测量的最终位置误差，不能作为定位精度指标。

## 补充：RViz 中观察带静止障碍的巡航

用户在导航开始前手动生成现有红色箱体模型，中心位于 Gazebo `world` 坐标 `(-2.0, 0.25, 0.25)m`，箱体尺寸为 `0.35×0.35×0.50m`；随后开启巡航。通过 RViz 观察雷达、全局代价地图和规划路径。

用户反馈三项均符合预期：代价地图显示新增障碍，机器人绕过箱体，最终巡航为 `COMPLETED`。本次依据人工反馈，没有另行提供完整状态输出或提取碰撞日志；不增加精确误差或无碰撞统计结论。这是导航开始前新增静止障碍的巡航演示。

## 补充：2026-10-23 自建 SLAM 地图上的巡航

用户手动建图保存后，使用 `leap_mapping/navigation.launch.py` 重新加载地图，人工设置初始位姿并完成单点导航，再通过原巡航节点执行 `leap_mapping/config/patrol_route.yaml` 的 A → B → C。调度实现保持不变。

用户提供完整终态：`COMPLETED`、C / 2、3 / 3、`last_nav_status=SUCCEEDED`、`goal_unresolved=false`、`active_goal_uuid=null`，并反馈运动符合预期。状态字段原样保存在[本轮终态](../docs/evidence/saved-map-patrol-20261023.json)，地图处理及完整流程见[建图记录](../docs/mapping-status.md)。这是一轮同一仿真场景、自建地图的人工演示；没有测量最终位姿误差或重复成功率。

## 补充：用户选点并扩展至四点

用户从 RViz `/clicked_point` 取得 D 的 `map` 坐标，在 Ubuntu 编辑 `leap_mapping/config/patrol_route_abcd.yaml`，以 `route_file` 加载 A → B → C → D，未修改调度代码。

完整输出为 `COMPLETED`、D / 3、4 / 4、`last_nav_status=SUCCEEDED`、`goal_unresolved=false`、`active_goal_uuid=null`，见[四点终态](../docs/evidence/saved-map-patrol-abcd-20261023.json)。由此确认本次通过配置扩展四点并完成任务；本轮未额外提供轨迹或运动观察，不据此增加平滑性、无碰撞或精度结论。

## 补充：真实 Nav2 返回 ABORTED 后停止整轮

用户执行 `leap_mapping/config/patrol_route_failure.yaml` 的 A → B_outside → C，第二点 `(10,10)` 在当前全局地图之外。保留默认 NavFn 与 Nav2 恢复树，调度实现未改动。

用户提供两次相同终态，并反馈全部符合预期：`FAILED`、B_outside / 1、1 / 3、`last_nav_status=ABORTED`、`reason="Failed at B_outside with status ABORTED"`、`goal_unresolved=false`、`active_goal_uuid=null`。证据见[失败终态](../docs/evidence/saved-map-patrol-failure-20261023.json)。因此真实 Nav2 失败结果已经过任务层处理，后续点未执行；周期性重复输出不计作重复试验。

这是地图外无效目标场景，与“地图内目标合法但通道被完全封死”不同；后者尚未进行。本轮没有单独采集后续目标的 Action 发送日志，停止后续点的结论结合终态、用户观察及既有调度逻辑判断。

## 软件侧检查与限制

- Ubuntu 构建成功，16 项框架与路线解析测试通过。
- 用户实现通过 5 项调度检查，包括逐点推进、完成后重启、失败停止、外部取消、单点路线。
- 框架使用受控 Future 检查目标拒绝、执行失败、重复启动、受理前取消，以及取消与成功结果交错。
- 审阅时临时去掉取消优先处理，回归检查能发现错误；临时将失败分支写成 `self.status`，调度检查能发现错误。正式源码保留正确实现。
- 已执行真实 Nav2 的地图外目标失败场景；地图内通道封闭、长期稳定性、不同仿真场景和实车导航尚未验证。

额外旋转的原因尚未定位，不据此归因于恢复行为、目标朝向或任务调度。用户选择本轮先保留该现象；后续需要优化运动流畅性时再分析。

交付配置默认仍为 `config/route.yaml` 的 A → B → C；B → A → C 保存在 `config/route_bac.yaml`。修改路线后重启巡航节点即可，仿真会话可以复用。
