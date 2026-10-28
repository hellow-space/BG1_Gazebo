# 三算法地图覆盖基准

独立离线模块：PNG/PGM → 统一地图语义 → Leader / Richey BCD / Spiral-STC adapter → 完整世界坐标路径 → 三项指标与原图叠加图。只调用规划，不启动 Gazebo、导航或运动。

设计见 [统一设计](../docs/benchmarks/three-planner-design.md)，实测见 [三图结果](../docs/benchmarks/three-planners-2026-10-27/README.md)。

## 依赖与复现

现有 Ubuntu 24.04 / ROS 2 Jazzy 环境；Python 需要 numpy、scipy、Pillow、matplotlib、PyYAML、pytest，BCD 构建需要 g++、pkg-config、OpenCV4 开发包及 Eigen3。Leader 使用已构建的本项目 overlay。

```bash
git submodule update --init third_party/bcd_planner third_party/python_robotics
bash leader_coverage_ws/build.sh
python3 -m pytest coverage_benchmark/tests -q
python3 -m coverage_benchmark
python3 -m coverage_benchmark.verify
```

导入地图默认不入 Git，需保留 `data/maps/imported/<场景>/<场景>.png` 及同名 PGM。实际输入为 PNG；PGM 文件头仅作为 0.05 m/像素的假设依据。不得把项目全部 `third_party` 加入 colcon 搜索路径。

在隔离 worktree 复用原 checkout 地图/Leader 构建时，加 `--runtime-root /home/jimmy/Desktop/BG1_Gazebo`。`--output` 指定结果目录。`--scene`、`--algorithm` 可限制运行范围，输出报告也限定该次范围；完整九组验收应运行默认全部范围。重新运行会更新指定结果目录，保留历史结果时另设 `--output`。

## 模块边界

- `models.py`：adapter 公共输入/输出；栅格为图像行序，输出路径为米制 XY、左下局部原点。
- `map_io.py`：阈值、最大四邻接自由分量、精确 EDT 中心净空、公共起点。
- `adapters/leader*.py`：隔离 ROS overlay/domain，发布地图并调用现有服务一次，退出时关闭自身进程。
- `adapters/bcd*.py`、`bcd_wrapper.cpp.in`：只读上游；生成 OpenCV4/无界面 C++ 副本，执行原生分解/静态规划；保留所有段及连接，不作过滤或修复。
- `adapters/stc.py`：保守图块缩放、偶数补障碍、原生 STC、还原原生插值连接；不改生成树逻辑。
- `metrics.py`：全部线段长度、作业圆盘扫掠并集、逐格穿越的障碍/净空核验。
- `runner.py`：顺序运行、完整路径 JSON、指纹、三指标 CSV、原图叠加图。
- `verify.py`：已保存结果复算、版本/原图指纹、独立点到路径几何见证；不重跑规划。

BCD 生成物在 `.build/`，临时地图和日志在 `.work/`，均忽略；上游 LICENSE 随子模块保留。公共层不添加 A*、不补接 disconnected components、不通过删坏段将无效路径改成有效。
