# 三算法、三地图离线基准设计

## 用户要求

下载并模块化适配 Richey BCD、PythonRobotics Spiral-STC，与 Leader 在三个地图场景比较。只报告覆盖率、总路径长度、路径有效性。此前工作已本地提交为 `0f54459`；用户随后明确取消远端推送。

## 方案

- 新增独立 `coverage_benchmark/`，公共层负责读图、目标掩膜、尺度/起点、指标、绘图；adapter 只调用原算法并转换输入输出。不改变现有三个 ROS 工作区接口，不启动运动。
- 新上游为 `third_party/bcd_planner`、`third_party/python_robotics` 固定提交子模块，原文件只读。Richey 无界面入口及兼容修正仅生成到构建目录，并记录原文件指纹。
- 三图：`A11123AJ04_231127_094529.png`、`A11123AC22_231213_163036.png`、`A111241C03_240410_144535.png`。缺 YAML 时沿用暂定 0.05 m/像素、局部零原点和 trinary 阈值 0.25/0.65，明确为假设。
- 每图最大四邻接自由区为共同规划目标；全部原图自由区始终为覆盖率分母。作业宽度/间距 0.6 m、规划中心净空 1.0 m；新 adapter 使用公共精确距离掩膜，Leader 保持自身原生净空实现，输出均独立审计。
- STC fine cell 对应 0.6 m 作业宽度，2×2 合成 coarse cell；保守聚合、不足格及奇数维度补障碍。粗化损失和起点量化单列元数据，不改变覆盖率分母。
- 不增加公共 A* 修复原生连接，不以其他算法替代失败；返回完整连接轨迹，原生失败如实记录。
- 覆盖率为全路径半径 0.3 m 圆形扫掠并集覆盖的原图自由格中心比例；长度包含全部连接；有效性要求非空、有限、连续且所有线段在地图及允许净空栅格内。该条件不是实际运动或真机验收。
- 无效路线仍可列名义覆盖率并标否；无路径的长度为空，不能以零长度作为优势。新结果写入 `docs/benchmarks/three-planners-2026-10-27/`，历史结果不覆盖。

## 接口

`PlanningInput`：`map_id`、`free`（共同目标 top-down bool 图）、`navigable`（同方向精确净空图）、`resolution`、`start`（世界 xy）、`tool_width`、`clearance`、`source_root`、`runtime_root`。局部世界 y 向上，图像行向下，原点暂为零。

`PlanningResult`：`path`（完整世界 xy 列表）、`success`、`message`、`details`。统一入口 `plan(request: PlanningInput, work_dir: Path) -> PlanningResult`。超时、异常与空路径必须保留原因。

## 验证

先做公共指标、输入转换及 adapter 小地图解析测试，再构建和运行真实上游。完成九组调用，逐段/扫掠核验、原图叠加图目视检查、源码/版本哈希核查和独立代码审查。只本地提交，不推送远端。
