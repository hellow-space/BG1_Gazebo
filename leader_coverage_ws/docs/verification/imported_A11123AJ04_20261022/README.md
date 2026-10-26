# Leader 导入地图规划验证（2026-10-22）

## 输入与参数

- 地图：`data/maps/imported/A11123AJ04_231127_094529/A11123AJ04_231127_094529.pgm`，1208 × 1710；使用原始 PGM，不替换为同名 PNG。
- 文件头标注 0.05 m/像素。缺失原始 YAML，采用临时局部原点 `[0,0,0]`，trinary 阈值 free=0.25 / occupied=0.65；这些是本次规划假设，未恢复原始世界坐标。
- 选区为图像范围内缩一格的矩形，起点选已知自由空间距障碍最远的格子中心 `(34.325,52.425)`；起点只参与路径规划，不代表机器人实际位置。
- Leader 原 `CoveragePathPlanner` 弓字模式，条带间距 0.7 m，净空参数 1.0 m，island 输出保持现有禁用配置。21 个原始源文件哈希均与构建时清单一致。

## 本次结果

- 真实 `/leader_coverage/plan` 服务返回成功：16,988 点，897.3167 m，调用耗时 0.4347 s（包含 ROS 传输）。
- 以不超过半个栅格的间距独立采样全部路径线段：67,436 个样本全部在地图内已知自由空间；按格子中心的欧氏距离变换测得最小障碍距离约 0.9962 m。这不是整机 footprint 验证。
- [地图与路径叠加图](coverage.png) 已目视核对；图上有未覆盖支路。未评估覆盖率，不能将原算法生成结果解释为全区清扫完成。
- 无配套 Gazebo 世界，本次没有启动机器人仿真、定位、Nav2 控制或运动执行。
- [result.json](result.json) 包含完整路径和参数；[source_hashes.json](source_hashes.json) 为原源码指纹。`planning.log` 与 `probe.log` 为本地执行日志，已加入忽略规则，不随仓库分发。

## 复现

本地原始地图需存在，先构建并加载 Jazzy、底层与 Leader overlay。以下命令分别在终端运行；不得与其他 BG1 地图/仿真会话同时启动。

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
source leader_coverage_ws/install/setup.bash
export ROS_LOG_DIR=/tmp/bg1_leader_imported_logs
mkdir -p "$ROS_LOG_DIR"
```

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$PWD/leader_coverage_ws/docs/verification/imported_A11123AJ04_20261022/map.yaml"
# 另一个已加载环境的终端：
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 run bg1_leader_coverage planner_server
# 再开已加载环境的终端：
python3 leader_coverage_ws/docs/verification/imported_A11123AJ04_20261022/probe.py
# 再开终端显示已发布结果：
rviz2 -d leader_coverage_ws/docs/verification/imported_A11123AJ04_20261022/preview.rviz
```

`probe.py` 会重新运行规划、更新本目录结果，并保持路径发布；Ctrl+C 退出。其余节点也在各自终端 Ctrl+C 关闭。当前任务的临时 launch 位于 `/tmp/bg1_leader_imported_20261022/preview.launch.py`；该 launch 启动的地图服务由生命周期管理器自动激活。
