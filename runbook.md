# Runbook

## 环境准备

- 所有命令在项目根目录执行；现有环境为 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic。已检查 Jazzy setup、`colcon` 与 `gz` 入口存在；本次未重新检查全部运行依赖。
- 首次克隆后运行 `git submodule update --init --recursive`；依赖版本见 [coverage 版本记录](coverage_ws/docs/versions.json)。构建依赖清单见 [coverage README](coverage_ws/README.md) 和各 ROS 包 `package.xml`。
- 每个终端按需加载一个覆盖 overlay；启动脚本已处理环境加载。直接运行 ROS 命令前执行：

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
export ROS_LOG_DIR=/tmp/bg1_ros_logs
mkdir -p "$ROS_LOG_DIR"
# 仅在使用相应覆盖工作区时加载其中一个：
# source leader_coverage_ws/install/setup.bash
# source coverage_ws/install/setup.bash
```

## 构建

先构建底层，再按需构建独立 overlay：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base ros_ws/log build --base-paths ros_ws/src \
  --build-base ros_ws/build --install-base ros_ws/install --symlink-install
bash leader_coverage_ws/build.sh
bash coverage_ws/build.sh
```

只使用其中一套覆盖演示时，仅执行对应构建脚本。coverage 首次配置会下载固定的 FetchContent 依赖。不要直接修改构建期生成的 Leader 适配副本。

## 启动

以下场景互斥，每次只选择一个启动：

| 用途 | 命令 | 操作 |
| --- | --- | --- |
| 点到点导航 | `bash ros_ws/navigation_demo.sh scene:=demo` | RViz Nav2 Goal；也可选 warehouse/office |
| 遥控建图 | `bash ros_ws/mapping.sh scene:=warehouse` | 键盘探索；也可选 demo/office |
| Leader 选区覆盖 | `bash leader_coverage_ws/run.sh` | 默认 warehouse，先选区再手动执行 |
| 空场 Fields2Cover | `bash coverage_ws/run.sh` | 默认空矩形区域 |

无界面时给启动命令追加 `gui:=false headless:=true rviz:=false`；建图另加 `teleop:=false`。键盘使用 `i` 前进、`,` 后退、`j/l` 转向、`k` 停车，松开按键不会停车。

建图停车后保存；默认生成新的时间戳目录，拒绝覆盖已有前缀：

```bash
bash ros_ws/save_map.sh
# 或指定不带扩展名的前缀：
bash ros_ws/save_map.sh data/maps/slam/my_room/my_room
```

在 leader 导入原图上仅运行规划（不带 Gazebo 世界）的独立操作和本次结果，见 [导入地图规划验证](leader_coverage_ws/docs/verification/imported_A11123AJ04_20261022/README.md)。先关闭其他 BG1 会话，再按该记录加载地图、激活服务、启动规划器和预览。该地图的 BG1 实际运动、场景生成、启动/停车和复现步骤见 [运动演示](leader_coverage_ws/docs/verification/imported_A11123AJ04_20261022/motion/README.md)。

在 Leader 会话的另一终端执行：

```bash
bash leader_coverage_ws/region.sh demo
# 或 RViz Publish Point 顺序点至少三个顶点，再点回首个顶点 0.35 m 内
bash leader_coverage_ws/region.sh zigzag       # 弓字；backshaped 为回字
bash leader_coverage_ws/motion.sh start        # 前台运行，Ctrl+C 取消并停车
```

`region.sh reset/undo/plan` 分别清空、撤销、闭合规划；改选区取消当前运动，新路径须再次显式启动。

在 Fields2Cover 会话的另一终端执行：

```bash
bash coverage_ws/task.sh                      # 仅规划与显示
bash coverage_ws/task.sh --execute            # 接近起点并跟踪
```

任务输出默认 `/tmp/bg1_coverage_result.json`，日志默认 `/tmp/bg1_coverage_logs`。修改区域优先编辑共享 `config/task.yaml`；CLI 的 `--task-config` 不会同步预览节点边界。

## 测试

Leader 与 Fields2Cover 的同图规划基线入口、固定参数、双分母覆盖率和复算方法见 [AJ04 PNG benchmark](docs/benchmarks/2026-10-27-A11123AJ04/README.md)。该流程分别加载两个 overlay、只启动规划器，不启动仿真或运动；不要直接将无效连接的几何扫掠面积当成可执行覆盖率。

源码静态测试（已加载底层环境）：

```bash
python3 -m pytest ros_ws/src/bg1_description/test ros_ws/src/bg1_gazebo/test \
  ros_ws/src/bg1_navigation/test ros_ws/src/bg1_scenarios/test -q
```

构建后对受影响工作区运行包测试。下例为底层；Leader 使用相应目录并先加载其 overlay，coverage 增加 `--packages-select bg1_coverage_demo`：

```bash
colcon --log-base ros_ws/log test --base-paths ros_ws/src \
  --build-base ros_ws/build --install-base ros_ws/install \
  --event-handlers console_direct+
colcon test-result --test-result-base ros_ws/build --verbose
```

覆盖适配的独立源码测试：

```bash
# 新终端先加载 Jazzy、ros_ws 和 coverage_ws 的 setup.bash
python3 -m pytest coverage_ws/src/bg1_coverage_demo/test -q
```

运行验证会控制机器人，须使用独立测试会话，不与交互目标或遥控并行：

```bash
# 裸模型仿真中：接口、前进、转向及停车检查
ros2 run bg1_gazebo verify_runtime.py
# demo 导航会话中：固定目标及独立 Gazebo 位姿检查
ros2 run bg1_navigation demo_goal.py --x 4.8 --y 0 --yaw 0
```

`verify_runtime.py` 主要依据轮式里程计；实际底盘运动仍需独立 Gazebo 位姿证据。历史证据位置见各工作区 README；本次初始化未重跑上述测试。

三算法离线模块的依赖、输入和完整操作见 [模块 README](coverage_benchmark/README.md)，本次 [九组结果](docs/benchmarks/three-planners-2026-10-27/README.md) 保留原图轨迹及失败证据：

```bash
python3 -m pytest coverage_benchmark/tests -q
python3 -m coverage_benchmark
python3 -m coverage_benchmark.verify
```

运行前保留对应本地 PNG/PGM、构建 Leader；默认三图、0.6 m 作业宽度、1.0 m 中心净空。全量重新运行会更新结果目录，可用 `--output` 另存。只启动本次规划进程，不启动 Gazebo 或运动；不要与其他 domain 101 会话并用。

Fields2Cover 地图感知扩展使用既有 coverage overlay、GDAL C API 和 Python numpy/scipy/Pillow/matplotlib/PyYAML。详细版本/依赖见 [模块说明](coverage_benchmark/README.md)。在无其他 domain 102 服务的会话中，顺序执行：

```bash
python3 -m coverage_benchmark.f2c_compare
python3 -m coverage_benchmark.verify_f2c
```

结果写入独立 `docs/benchmarks/f2c-obstacle-2026-10-28/`，包括原生/修改版轨迹、三指标及逐段来源。新版本仅规划公共起点净空可达区，分母仍为全图自由像素；原生转弯失败可退回有序条带＋A*，原生错误保留。服务进程有2 GiB地址空间上限，不启动运动。不要把几何审计通过写成全区完成或真机保证。

## 常用检查

```bash
git status --short
git submodule status
ros2 node list
ros2 topic list
ros2 run tf2_ros tf2_echo map base_footprint
find ros_ws/install leader_coverage_ws/install coverage_ws/install -xtype l -print
```

确认同一会话没有多个 `map → odom` 来源。最后一条列出失效安装链接；全新克隆未构建时目录可能不存在。

## 停止与清理

- 导航或裸模型：在启动终端按 Ctrl+C；键盘遥控先按 `k`，再 Ctrl+C。
- 建图：`bash ros_ws/stop_mapping.sh`。
- Leader：`bash leader_coverage_ws/motion.sh stop` 仅停车；`motion.sh status` 查看阶段；`bash leader_coverage_ws/stop.sh` 关闭会话。
- Fields2Cover：任务终端 Ctrl+C 取消并停车，`bash coverage_ws/stop.sh` 关闭会话。
- 取消确认失败并触发生命周期停用后，须重启对应仿真再发新任务。
- 不把本地地图当缓存清理；SLAM 的 `.yaml/.pgm/.posegraph/.data` 四件套一起备份。symlink install 依赖构建树，清理后应重新构建，不能只保留安装软链接。

## 常见问题

- 第二次启动被拒绝：先关闭已有 BG1 会话，检查残留节点，避免直接并行启动。
- 地图放入 `data/maps/imported/` 后未显示：该目录不是自动加载入口，部分地图缺少原点/阈值 YAML；现有导航仍默认使用场景自带地图。
- office 规划失败：先核对场景专用 Dijkstra 配置是否生效，不修改所有场景的默认规划器。
- Leader 预览只覆盖部分区域：粗栅格、1 m 默认净空和原算法连接策略可能排除窄道或跳过分组；预览路径不等于全区覆盖。
- Fields2Cover 全量结果扫描提示损坏 XML：已有记录指出上游测试夹具含故意损坏样本；应检查实际包测试结果，不能仅凭该提示判断失败。
