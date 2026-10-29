# 栅格覆盖规划基准

[Leader / Fields2Cover 形象流程分镜与Headland特写](../docs/diagrams/coverage_logic_2026-10-29/README.md)：同一示例地图展示每一步的变化。

[四算法统一总览与三场景轨迹图](../docs/benchmarks/four-planners-2026-10-28/README.md)。仅汇总已有轨迹、核对参数并重算指标：`python3 -m coverage_benchmark.compare_saved`；不启动规划器。

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

## Fields2Cover 障碍感知版本

[设计](../docs/benchmarks/f2c-obstacle-design.md) · [三图原生/修改版对照](../docs/benchmarks/f2c-obstacle-2026-10-28/README.md)。固定原生基线不变，新增自有地图感知流水线；这是净空条带重生成＋裁剪＋连接的整体改动。

```bash
# 需要现有 coverage overlay；首次使用按 coverage_ws/README.md 构建
bash coverage_ws/build.sh
python3 -m coverage_benchmark.f2c_compare
python3 -m coverage_benchmark.verify_f2c
# 本模块含一个真实 ROS 原生调用测试；与 benchmark 顺序运行
python3 -m pytest coverage_benchmark/tests -q
```

隔离 worktree 复用主目录构建/数据时，两个入口加 `--runtime-root /home/jimmy/Desktop/BG1_Gazebo`，测试设置 `BG1_RUNTIME_ROOT=/home/jimmy/Desktop/BG1_Gazebo`。`--output` 可另存；默认输出新目录，旧三算法结果不变。该流程会启动并关闭自身 domain 102 规划服务，不启动运动；同模块的原生服务调用有锁串行化，不要在同 domain 另启外部服务。

| 模块 | 职责 |
| --- | --- |
| `raster_geometry.py` | GDAL 精确像素边界矢量化/孔洞/回填 |
| `adapters/f2c_safe.py` | 显式起点可达区、原生调用编排、失败回退 |
| `adapters/f2c_worker.py` | 独立 ROS 环境、真实有序条带/路径、进程生命周期及2 GiB地址空间限制 |
| `swath_geometry.py` | 按格线精确裁剪，保留原条带方向 |
| `grid_connector.py` | 无切角 A*、检查后的可见性简化 |
| `obstacle_connector.py` | 原生索引匹配、连接/条带组装、逐段来源及整路复核 |
| `provenance.py` | 实现、依赖、二进制、栅格和文件指纹 |
| `f2c_compare.py` / `verify_f2c.py` | 六组对比、检查点、完整来源重放与独立几何复核 |

每条输出边的来源在 safe JSON 的 `details.path_sections`：`kind`、`method`、`source_swath_index`、顶点索引范围和裁剪前后端点。manifest 绑定实际源码内容及数据指纹；只看基准 Git 提交不足以定位运行时尚未提交的新实现。

1 m 净空可能使原图断连：只规划公共起点可达分量，其他区域仍计为未覆盖，不能称全图完成。可选原生转弯失败时保留已成功返回的有序条带及错误，再使用 A*；请求内任何连接/最终审计失败均不返回成功。该版本是可停车转向的折线规划，不保证 Dubins 曲率或真机执行。
