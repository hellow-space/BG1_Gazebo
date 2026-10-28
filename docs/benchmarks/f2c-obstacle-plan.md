# Fields2Cover 避障连接 Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans inline；项目 `AGENTS.md` 的自主实现/验证规则优先，已有用户确认的方案不重复请求批准。

**Goal:** 为固定 Fields2Cover 增加地图感知的连续全局连接，保留独立原生基线。
**Architecture:** 公共 raster/metrics；原生 F2C ROS worker 输出 swaths/turns；自有 adapter 裁剪、A*、最终审计；独立三图对照 runner。
**Tech Stack:** Python/numpy/scipy/GDAL C API、已有 ROS 2 Jazzy/opennav/Fields2Cover。
**Spec:** [已确认设计及实施选择](f2c-obstacle-design.md)。

## Global Constraints

- 不改原图、第三方、原生结果、其他 ROS overlay；只规划，不运动，不 push。
- 0.6 m 作业宽度、1.0 m 精确中心净空、原图自由像素分母；不可达区明示，不算覆盖。
- A* 为折线、可停转模式，不声称 Dubins 曲率；任何最终线段无效均失败。

## Review Focus

- 浮点端点落在像素边界、接缝方向反转，不能漏掉短障碍跨越。
- 净空后的碎片及不可达区统计不能缩小覆盖率分母。
- native 转弯索引必须与未裁剪的相邻条带对应，不能串错段。
- A* diagonal 不切角；缓存/简化不能跨过狭缝障碍。
- ROS 进程超时/异常必须清理，真实条带来源和完整路径应可复算。

### Task 1: 公共裁剪、A* 与路径组装

**Files:** `coverage_benchmark/{swath_geometry,grid_connector,obstacle_connector}.py`、`tests/test_obstacle_connector.py`。
**Interfaces:** `clip_swath(swath, allowed, resolution)`；`GridConnector(allowed,resolution).connect(start,end)`；`assemble_path(swaths,turns,allowed,resolution,start)` 返回完整路径和诊断；allowed/world 原点为 lower-left。

- [x] 先写上述几何小图测试，运行观察缺失实现的失败。
- [x] 实现精确线段裁剪、无切角 A*、经过复核的可见性简化、全接缝审计。
- [x] Run: `python3 -m pytest coverage_benchmark/tests/test_obstacle_connector.py -q`。Expected: 全通过。

### Task 2: 原生 F2C worker 与模块化 adapter

**Files:** `raster_geometry.py`、`provenance.py`、`adapters/f2c_worker.py`、`adapters/f2c_safe.py`、`tests/test_f2c_safe.py`。
**Interfaces:** 复用 `PlanningInput/Result`，`f2c_safe.plan(request,work_dir)` 输出原始基线详情、真实native候选swaths/turns和经过Task1的安全路径。

- [x] 写孔洞回填、起点净空分量、无有效条带/无法连接失败测试并观察 RED。
- [x] 实现 GDAL 转换、固定参数每图三次 action（独立有序条带用于来源核验）、独立 overlay/domain 进程退出、版本/输入指纹。
- [x] 运行小图和一次真实服务诊断，确认 native 返回 swaths/turns 和接口匹配。
- [x] Run: `python3 -m pytest coverage_benchmark/tests -q`。Expected: 全通过。

### Task 3: 三图实测、结果复核与文档

**Files:** `f2c_compare.py`、结果目录；更新 `plan/status/structure/runbook` 和模块 README。
**Interfaces:** Task2真实路径＋原生基线，复用统一 evaluate_path。

- [x] Run: `python3 -m coverage_benchmark.f2c_compare --runtime-root /home/jimmy/Desktop/BG1_Gazebo`。Expected: 六组真实结果；所有成功避障输出通过逐段净空审计，否则先查适配错误、如实记录限制。
- [x] 独立点到完整路径见证、原生 swaths 证据、版本指纹、图像和三指标复核；旧九组不变。
- [x] 一次独立代码审查，修复重要问题；合入当前目录，执行适当回归和状态/空白检查。沿用本地提交授权，不 push。
