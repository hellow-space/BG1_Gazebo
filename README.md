# BG1 Gazebo navigation and coverage

Lightweight BG1 Pro simulation for ROS 2 Jazzy and Gazebo Harmonic.
The project includes a differential-drive model, Nav2/RPP point-to-point
navigation, and an independent Fields2Cover-based coverage demonstration.

- [Navigation workspace and build instructions](ros_ws/README.md)
- [Coverage overlay and usage](coverage_ws/README.md)
- [Coverage planning/execution evidence](coverage_ws/docs/verification/summary.json)

## Clone

Clone with `git clone --recurse-submodules <repository-url>`, or initialize
the pinned third-party dependencies after cloning:

```bash
git submodule update --init --recursive
```

The submodules pin `opennav_coverage`'s Jazzy branch and Fields2Cover's
`v1.2.1-devel` branch. Build the underlay first using `ros_ws/README.md`,
then build the coverage overlay:

```bash
bash coverage_ws/build.sh
```

## Run

From this repository's root, choose one simulation at a time:

```bash
# Point-to-point Nav2 navigation
bash ros_ws/navigation_demo.sh

# Independent coverage demo
bash coverage_ws/run.sh
```

In another terminal, request coverage planning or motion:

```bash
bash coverage_ws/task.sh             # Plan and visualize only
bash coverage_ws/task.sh --execute   # Approach and follow the coverage path
bash coverage_ws/stop.sh             # Stop coverage Gazebo/RViz and nodes
```

The coverage demo uses simulator ground-truth localization to isolate
coverage planning and path tracking from wheel-odometry drift. It does
not validate real-robot localization or obstacle-aware cleaning coverage.
Build/install/log trees are generated locally and excluded from Git.

## 大场景建图与导航

新增两个可切换场景，详见 [场景说明与 Gazebo 截图](ros_ws/src/bg1_scenarios/README.md)：

| 场景 | 尺寸 | 内容 |
| --- | --- | --- |
| `warehouse` | 36 × 28 m | 货架通道、托盘、立柱 |
| `office` | 32 × 26 m | 16 间互通房间、十字走廊、桌柜 |

```bash
# 选择一个场景建图，打开 Gazebo、RViz 和遥控终端
bash ros_ws/mapping.sh scene:=warehouse
# 或：bash ros_ws/mapping.sh scene:=office

# 保存地图、关闭建图会话
bash ros_ws/save_map.sh
bash ros_ws/stop_mapping.sh

# 使用场景自带参考地图导航（另起会话）
bash ros_ws/navigation_demo.sh scene:=office
# 或：bash ros_ws/navigation_demo.sh scene:=warehouse
```

不传 `scene` 时保留原始 `demo`。每次只运行一个仿真会话。

## Leader 提供的覆盖规划器

`third_party/leader/coverage_path_planner/` 是 leader 提供的 ROS 1 代码，原文件保留。
新增的 [独立 ROS 2 适配与选区说明](leader_coverage_ws/README.md) 与上述
Fields2Cover 覆盖演示分别构建、分别启动：

```bash
bash leader_coverage_ws/build.sh
bash leader_coverage_ws/run.sh
# 在另一个终端先生成一个仓库示例区域
bash leader_coverage_ws/region.sh demo
```

自选区域：RViz **Publish Point** 顺序点顶点，最后点回首个绿色顶点闭合规划。
选好区域后，运行 `bash leader_coverage_ws/motion.sh start` 手动开始运动；
Ctrl+C 或 `bash leader_coverage_ws/motion.sh stop` 取消并停车。

## 项目目录

```text
BG1_Gazebo/
├── docs/manuals/              # BG1 用户手册
├── data/maps/                # 本地地图数据，见该目录 README
│   ├── imported/<地图编号>/   # 导入 PGM/PNG（默认不提交）
│   └── slam/<会话编号>/       # 建图四件套（默认不提交）
├── third_party/              # 原始第三方代码
│   ├── leader/coverage_path_planner/
│   ├── opennav_coverage/      # Git 子模块
│   └── Fields2Cover/          # Git 子模块
├── ros_ws/                   # BG1 模型、Gazebo、Nav2、SLAM、场景
├── leader_coverage_ws/        # Leader 算法的 ROS 2 适配与执行
└── coverage_ws/               # opennav/Fields2Cover 的 BG1 集成
```

[用户手册](docs/manuals/BG1_Pro_User_Manual_用户手册-CN_0506.pdf)、
[地图数据规则](data/maps/README.md)、[第三方来源](third_party/README.md)。
工作区中的 `build/install/log` 是本地生成内容，不纳入 Git；仿真场景自带的地图仍随 ROS 包提交。

## 覆盖算法离线 benchmark

[独立模块](coverage_benchmark/README.md) 对接 Leader、Richey BCD 和 PythonRobotics Spiral-STC；输入导入 PNG/PGM，输出原图上的完整全局轨迹、覆盖率、总长度和路径有效性。固定版本、三张 PNG 的九组实际结果及失败原因见 [三场景对比](docs/benchmarks/three-planners-2026-10-27/README.md)。该入口只规划，不启动运动。

Fields2Cover 的地图感知扩展及 [三图原生/修改版对照](docs/benchmarks/f2c-obstacle-2026-10-28/README.md) 已单列：保留原始条带来源，加入精确净空、条带裁剪和 A* 连接，每条路径边可追溯。复现入口为 `python3 -m coverage_benchmark.f2c_compare`，只规划、不运动。

Leader、BCD、STC 与 Fields2Cover 修改版的结果现集中在 [四算法统一对比](docs/benchmarks/four-planners-2026-10-28/README.md)，包含12组指标及每图四列轨迹/覆盖图。`python3 -m coverage_benchmark.compare_saved` 可从既有原始路径重建总览，不调用规划器。

新增 ETHZ Polygon Coverage（原生BCD＋可见图＋GK-MA）已适配，[五算法统一对比](docs/benchmarks/five-planners-2026-10-29/README.md)包含同三图15组指标和五列轨迹/覆盖图。ETHZ采用0.2 m保守多边形栅格，每块原像素必须全部满足1 m净空；最终在原图分辨率审计。复现：`python3 -m coverage_benchmark.ethz_compare`；复核：`python3 -m coverage_benchmark.verify_ethz`。构建、固定版本、预处理范围和原生失败诊断见[模块说明](coverage_benchmark/README.md)。
