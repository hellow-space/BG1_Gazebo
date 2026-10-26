# BG1 在 leader 导入地图中的运动演示（2026-10-22）

## 场景和接口

- 原图及规划参数沿用 [上一阶段记录](../README.md)。本地地图不入 Git，需要单独保留。
- `prepare.py` 将与地图服务相同阈值确定的非自由格（含未知）按行合并矩形，再导出单一静态 STL 碰撞/显示网格；1,688 个矩形、20,256 个三角面，生成时逐格重建投影并断言与原始阈值结果完全相同。
- 场景地面平整，柱体高 1.2 m 是演示假设，不代表原场地真实高度。原 PGM 未改，生成物在 `data/maps/imported/A11123AJ04_231127_094529/gazebo_demo/`。
- `motion.launch.py` 复用底层现有 `sim.launch.py`、BG1 模型和 Nav2 配置；只覆盖世界、地图、出生位置与 AMCL 初始位置 `(34.325,52.425,0)`。AMCL 是唯一 `map → odom` 来源，没有加载 Fields2Cover overlay 或其真值定位节点。
- Leader 原服务重新规划，仍为 16,988 点、约 897.3 m；通过现有 `coverage_motion.py` 的 NavigateToPose → FollowPath/RPP 执行，限速仍为 0.3 m/s。
- `runtime.py` 只规划、发布路径并记录 Gazebo 独立位姿与 TF；运动必须另行使用 `motion.sh start`。起步后才修正并重启了记录探针的 Gazebo JSON 时间戳解析（sec 为字符串），因此其累计行程从重新开始采样计算，不包含此前约 1 m 接近段和最初部分跟踪段。

## 运行与停车

所有操作在项目根目录。先关闭其他 BG1/地图会话，再在各终端加载同一套环境：

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
source leader_coverage_ws/install/setup.bash
export ROS_LOG_DIR=/tmp/bg1_leader_motion_logs
mkdir -p "$ROS_LOG_DIR"
```

```bash
python3 leader_coverage_ws/docs/verification/imported_A11123AJ04_20261022/motion/prepare.py
ros2 launch leader_coverage_ws/docs/verification/imported_A11123AJ04_20261022/motion/motion.launch.py
# 另一个已加载环境的终端：
python3 leader_coverage_ws/docs/verification/imported_A11123AJ04_20261022/motion/runtime.py
# 再开一个终端，显式执行：
bash leader_coverage_ws/motion.sh start
```

停车和查看阶段仍使用原入口：

```bash
bash leader_coverage_ws/motion.sh status
bash leader_coverage_ws/motion.sh stop
```

`start` 终端 Ctrl+C 也会取消并停车。停止运动后可继续查看 Gazebo/RViz；关闭整个会话需在 launch 终端 Ctrl+C。此次由 AI 启动的临时 launch 为 `/tmp/bg1_leader_motion_20261022/motion.launch.py`，可以在停车后用以下命令关闭它，再退出记录探针：

```bash
pkill -INT -f '^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch /tmp/bg1_leader_motion_20261022/motion.launch.py$'
pkill -INT -f '^python3 /tmp/bg1_leader_motion_20261022/runtime.py$'
```

Gazebo 近距离跟随视角可设置：

```bash
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 2000 --req 'data: "bg1_pro"'
gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 2000 --req 'x: -5, y: -5, z: 5'
```

## 本次检查结果

- 起点接近成功，完整 16,988 点路径已进入 `following`；检查快照累计记录实际行程 37.775 m（不含最初未记录段）。
- 独立 Gazebo 姿态范围 3.301 rad，结合轨迹图确认第一次弓字掉头并进入返回条带；AMCL/真值位置差最大 0.157 m、平均 0.077 m。
- 2026-10-22 快照时仍在执行，`follow_succeeded=false`、`full_route_completed=false`；全程和覆盖率未验收。该会话之后已关闭，关闭前记录为 Nav2 code=105，未完成整条路线。
- [实际轨迹](actual_trajectory.png)、[检查快照](checked_snapshot.json)、[Gazebo/RViz 界面](desktop.png) 已保存。

## 验证材料

- `generation.json`：场景生成与地图投影一致性证据。
- `runtime.json`：本地原始运行记录，每约 10 秒更新，包含独立 Gazebo 位姿、行程、最近路径点及 AMCL/真值位置差。文件较大，与本目录 `*.log` 一同忽略，不随仓库分发；本地文件保留。
- 仓库保留检查点 `checked_snapshot.json`、`checked_samples.json`、轨迹图与界面截图；这些材料只证明快照时的仿真检查，不证明全程完成或真机定位能力。
- 原算法存在未覆盖支路，本次没有量化全区覆盖率；约 897 m 全程至少需约 50 分钟，不将短段运动验证写成整条路线完成。
