# LEAP 连续标记跟随

独立的 ROS2 Humble / ament_python 功能包。现有实车工作空间中的 `leap_object_follow`、预览节点和有界试验脚本保持原样；这些旧入口尚未收录到当前仓库。本包不导入旧跟随包，不自动启动 Agent、相机或底盘。项目概览和快速入口见[仓库首页](../README.md)。

**当前状态：已通过ROS2 Humble构建、18项逻辑测试，以及2026-10-22首轮宿舍场景实车人工验收。** 用户反馈整轮效果符合预期，最终车头到标记面约24.5厘米。

## 交付目标

- 启动后保持关闭，用户开启后持续跟随同一个 `DICT_4X4_50` ID0 标记，无试验时长、累计转角或里程预算。
- 用户在**车头最前缘到标记面约25厘米**处教一次画面大小，保存后可跨启动复用。它是视觉大小基准，不是精确测距或完整相机标定。
- 连续计算前进和转向，不使用“转向→停稳→前进”的串行试验阶段。本轮实车使用YAML前进上限0.10m/s、转向上限0.60rad/s；近目标降低速度，前进启动值为0.05m/s（仍受朝向缩放与加速限制）。
- 一两帧漏检时减速，保留已确认身份；超过0.25秒没有新目标则零输出。图传/scan/odom恢复后自动重新确认并继续，**用户关闭后不会自动重启**。
- 障碍检查只停车，不绕行。Nav2导航作为独立功能推进；本包不与Nav2或遥控器同时发布速度。
- 首版前进和转向，目标靠得更近时停止前进；没有后向感知，不自动倒车。标记保持大致正对相机、位于视野内并缓慢移动。

## 首轮实车结果（2026-10-22）

用户亲自完成启动数据节点、启动新跟随节点、采集25厘米基准、开启、缓慢直行/左右移动、短遮挡恢复及关闭停车。对这套流程的整体反馈为“全都符合预期效果”，最终车头到标记面粗量约24.5厘米，用户接受。

证据为用户现场观察及一次约值测量，未提供逐项运行日志、测量误差或长时间统计。记录为本次宿舍场景人工验收通过。归档了实际Ubuntu配置、800×600的ID0基准以及本轮源码快照。当前保持这一组参数，供日常使用和后续版本管理。

## 文件与数据流

```text
camera/image_raw/compressed ── ArUco检测 ── controller.py ──┐
scan + odom ──────────────── safety.py ────────────────────┤── cmd_vel
enable / capture_reference ─ node.py + reference.py ───────┘
```

`controller.py`负责连续控制；`safety.py`负责输入有效性和LEAP平面停止区域；`reference.py`保存基准；`node.py`接ROS接口。`config/`、`launch/`、`test/`分别放启动参数、入口和针对性测试。

## 构建和启动

把本目录放到Ubuntu的 `~/xuegeros_ws/src/leap_marker_follow`。Windows本机目录不是虚拟机中的安装副本。

依赖ROS2 Humble、NumPy及带`cv2.aruco`的OpenCV；沿用现有相机环境即可。支持`ArucoDetector`及旧版`aruco.detectMarkers`接口，无需为了本包升级ROS。

```bash
cd ~/xuegeros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select leap_marker_follow --symlink-install
source install/setup.bash
ros2 launch leap_marker_follow follow.launch.py
```

运行前，用户照原方式启动Agent、相机及车端数据；确认`/camera/image_raw/compressed`、`/scan`、`/odom`正常。关闭旧跟随试验、遥控和Nav2速度发布节点。新包开启时有第二个`/cmd_vel`发布者会拒绝，运行中发现冲突会关闭跟随，需要人工重新开启。

## 首次设置25厘米基准（以后通常不用重复）

小车停着、跟随关闭。标记正对相机，放到车头最前缘前约25厘米，画面中央静置一秒；雷达和里程计需正常且里程计显示静止。新终端：

```bash
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
ros2 service call /marker_follow/capture_reference std_srvs/srv/Trigger '{}'
```

返回`success=True`后保存到 `~/.ros/leap_marker_follow/reference.json`。相机安装位置、分辨率、镜头/数字变焦或标记大小改变后重教。透视倾斜也会改变大小，因此先保持标记大致正面。

## 日常操作

开启前把标记放在可见位置，等约一秒确认。只需在开始和结束时操作终端，移动标记期间无需反复输入：

```bash
# 开启，之后持续运行
ros2 service call /marker_follow/enable std_srvs/srv/SetBool '{data: true}'

# 关闭，立即发送零速度，并短暂重复零命令
ros2 service call /marker_follow/enable std_srvs/srv/SetBool '{data: false}'

# 需要时观察状态
ros2 topic echo /marker_follow/status --once --full-length
```

`enabled=true`且`state=paused`表示用户仍请求跟随，但当前因缺输入、丢标记或障碍停车，恢复后会继续。`enabled=false`表示保持关闭。返回成功表示节点处理了请求，实际停车以现场观察为准。

可选打开`rqt_image_view`并选择**`/marker_follow/debug`**，这里直接发布原始`Image`，无需选择`/compressed`或设置image_transport。只在有人订阅时生成调试图。

## 参数与范围

编辑`config/follow.yaml`，重新启动生效；运行中拒绝修改这些启动参数，避免参数值看似已变但控制器未变。可通过launch的`params_file:=绝对路径`选自己的参数文件。改变主题用ROS标准remapping。

日常主要关心`max_forward_mps`和`max_turn_rps`；`forward_gain`控制大小误差如何转换为前进速度，`turn_gain`控制横向纠偏。其余为目标ID、基准文件、输入超时及前向/侧向停止区域余量，不需要每轮调整。

雷达停止区域沿用Stage1底盘包围范围和URDF的laser/axle位置，前向余量改为候选0.18m以配合近距离跟随，转向检查保守包围圆。它不是主动避障，也不是实测制动距离；单平面雷达看不到低矮障碍和全部车体高度。时间检查覆盖接收时间与ROS发布时间，尚不能证明源画面曝光新鲜或排除重新打时间戳的冻结视频。

正常关闭及Ctrl+C会尝试发零速度；进程崩溃或网络断开仍依赖底盘自身的速度超时停车。已有固件源码中为300ms，此包不会刷写固件，也未核验当前车端烧录内容。

## 验证与学习安排

纯Python检查（无ROS环境也能执行）：

```bash
cd ~/xuegeros_ws/src/leap_marker_follow
python3 -m unittest discover -s test -v
```

本地测试涵盖启停、超过旧预算仍运行、前进转向同时输出、近距停车、短漏检减速、长间隔停车/恢复、输入/障碍阻断、基准保存及简单理想相机闭环。理想模型不等于Gazebo或实车。

2026-10-21 Windows检查：18项纯Python测试通过，9个Python文件语法与package.xml/resource检查通过。将阻断条件在内存中临时失效后，对应零输出测试确实失败；源文件未被此检查修改。Humble接口对照了官方[rclpy信号处理](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/signals.py)、[初始化](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/__init__.py)及[发布器](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/publisher.py)源码。随后已复制到Ubuntu，通过Humble构建及同样18项逻辑测试，核对安装入口、launch/config和16个传输文件；OpenCV为4.13.0。当时只做了节点模块导入检查；2026-10-22已进一步完成上文所述的首轮人工实车验收。

Ubuntu也可使用标准测试入口。`setup.py`中的`tests_require=['pytest']`登记测试执行器，pytest可以执行本包的unittest风格测试：

```bash
cd ~/xuegeros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select leap_marker_follow
colcon test-result --test-result-base build/leap_marker_follow --verbose
```

首轮人工实车验收已完成。接下来整理版本管理与展示材料，再回到独立Nav2导航主线；当前跟随参数保持不变。后续改动按影响检查相关行为，不重复整套历史试验。
