# PGM / OccupancyGrid 的 BCD、STC 开源候选（2026-10-27）

## 筛选结论

- 真正 STC 基线优先选 **PythonRobotics / SpiralSpanningTreeCPP**：源码能对应出四子格、生成树边及沿树覆盖路径，MIT，纯 Python，不需要先移植 ROS 1。
- BCD 的室内规划研究基线优先考察 **IPA 的标准 Boustrophedon（配置 2）**：C++/二值图核心、具有室内覆盖论文背景；但原包 ROS 1 依赖较多，许可声明存在差异，不能简单当作标准 LGPL 或 Jazzy 即插即用包。
- **RicheyHuang 的 BCD** 是不依赖 ROS 的 C++/OpenCV 实验候选，输入距离 PGM 最近，适合较轻量的离线适配；作者明确标注 toy/unfinished，跨 cell 连接也不能视为已验证可行。
- **Nobleo FCPP 应归为 BSA**，可加作第四种基线，不能因为类名 `SpiralSTC` 就把它标成 STC。
- 本轮只检索并核查公开源码/文档/许可及版本，没有安装、构建、运行或添加这些依赖，不能预判它们在 AJ04 图上的长度或覆盖率。

## 与现有输入的关系

现有 Leader 服务接收 `nav_msgs/OccupancyGrid`，PGM 是地图图像载体。候选实现的核心接收 `cv::Mat`、NumPy 二维数组或 bool 栅格，因此可以统一由同一张 PGM/PNG + YAML / OccupancyGrid 生成输入，不必限定仓库是否直接提供 `.pgm` 文件参数。

输入适配必须统一：图像占据阈值、unknown 作为障碍、分辨率与原点、Y 轴翻转、起点、机器人通行膨胀和作业宽度。不能把灰色像素直接当作非零自由格；也不能把每个仓库内部筛选后的可达格数当成全图覆盖率分母。

## 候选一：PythonRobotics 的 Spiral-STC

- [仓库](https://github.com/AtsushiSakai/PythonRobotics)、[官方覆盖规划文档](https://atsushisakai.github.io/PythonRobotics/modules/5_path_planning/coverage_path/coverage_path.html)。
- 固定源码：[spiral_spanning_tree_coverage_path_planner.py](https://github.com/AtsushiSakai/PythonRobotics/blob/8c392897c7f41a6e4d9aab9cb2e9737a7a6fa031/PathPlanning/SpiralSpanningTreeCPP/spiral_spanning_tree_coverage_path_planner.py)。
- 入口：`SpiralSpanningTreeCoveragePlanner(occ_map).plan(start)`；`occ_map` 是二维数组，True/非零表示自由；`start` 是合并后粗格的 `(row,col)`。示例读灰度 PNG，PGM 可在外层读取后转入相同数组。
- 已核实：2×2 子格合成粗格，四个子格都自由才能展开；递归遍历未访问四邻居，保存父子树边，然后生成子格路径。对应 Spiral-STC，而非只有螺旋形状。
- [MIT 许可证](https://github.com/AtsushiSakai/PythonRobotics/blob/8c392897c7f41a6e4d9aab9cb2e9737a7a6fa031/LICENSE)。固定核查提交：`8c392897c7f41a6e4d9aab9cb2e9737a7a6fa031`。
- 适配判断：离线适配成本低，无 ROS 依赖；没有米制分辨率、机器人半径和作业宽度参数，需外层明确映射。不能直接用 0.05 m 地图像素当 STC 子格、又宣称与 0.6 m 条带间距等价。
- 边界：输入宽高都要求偶数，奇数应补障碍边而非裁去地图；2×2 全自由条件会排除墙边残余格和窄通道，只覆盖起点所在的粗格连通区；大图递归深度需验证。不能靠改变覆盖率分母掩盖粗化损失。
- 输出 `edge, route, path`；`path` 是线段集合，部分线段间的连接是在可视化函数中重建。Benchmark 必须导出完整折线，计算这些连接的长度并检查碰撞，不能只累计每个孤立 `[p,q]`。

## 候选二：IPA Room Exploration 的 BCD

- [仓库](https://github.com/ipa320/ipa_coverage_planning)，对应 [室内覆盖论文](https://ieeexplore.ieee.org/abstract/document/8460566)。
- 固定核心：[boustrophedon_explorator.cpp](https://github.com/ipa320/ipa_coverage_planning/blob/986c18384ed884dadd3bc857cd0c47c13b7d4716/ipa_room_exploration/common/src/boustrophedon_explorator.cpp)。入口 `BoustrophedonExplorer::getExplorationPath(cv::Mat, ...)`，黑 0 / 白 255 的单通道二值图。
- ROS action 接收 [sensor_msgs/Image 及地图分辨率/原点](https://github.com/ipa320/ipa_coverage_planning/blob/986c18384ed884dadd3bc857cd0c47c13b7d4716/ipa_building_msgs/action/RoomExploration.action)，不是直接接 OccupancyGrid；需转换数据及坐标方向。
- [配置文件](https://github.com/ipa320/ipa_coverage_planning/blob/986c18384ed884dadd3bc857cd0c47c13b7d4716/ipa_room_exploration/ros/launch/room_exploration_action_server_params.yaml)：算法 **2** 为标准 Boustrophedon，**8** 是变体且为该 YAML 默认值，不能混为同一实现。可设 `return_path: true`、`execute_path: false`，只做规划比较。
- 已核实：扫线连通变化触发 cell 分裂/合并，含真正 BCD；有旋转、细胞内覆盖线和 A* 辅助。A* 出现于入口距离评估和 cell 内转换，但所查返回路径流程直接拼接 cell 路径，不能宣称已完整插入跨 cell 的无碰撞连接。
- 原包为 ROS 1 / catkin、C++，依赖 OpenCV/Eigen；完整集成还涉及 tf、move_base、dynamic_reconfigure、COIN 等。建议隔离适配算法核心，不先迁移整套 ROS 1 系统。适配成本静态判断为中到高，尚未构建验证。
- 许可须按原文区分：[文件头](https://github.com/ipa320/ipa_coverage_planning/blob/986c18384ed884dadd3bc857cd0c47c13b7d4716/ipa_room_exploration/common/include/ipa_room_exploration/boustrophedon_explorator.h)声明 LGPL-3.0-or-later 并附其他条款；[package.xml](https://github.com/ipa320/ipa_coverage_planning/blob/986c18384ed884dadd3bc857cd0c47c13b7d4716/ipa_room_exploration/package.xml)声明学术/非商业用途 LGPL，商业用途联系 Fraunhofer IPA。声明不一致，本轮没有作法律解释。
- 固定 `noetic_dev` 核查提交：`986c18384ed884dadd3bc857cd0c47c13b7d4716`（提交日期 2025-11-20）。服务侧还会做形态处理并取最大自由连通区；这些剔除应保留在公共分母中，不归为已覆盖。

## 候选三：RicheyHuang 的 C++ BCD

- [仓库](https://github.com/RicheyHuang/BoustrophedonCellularDecompositionPathPlanning)、[固定 main.cpp](https://github.com/RicheyHuang/BoustrophedonCellularDecompositionPathPlanning/blob/c90ffa61f4b84c3c09b41c1ba5a837f49d44d278/main.cpp)、[构建文件](https://github.com/RicheyHuang/BoustrophedonCellularDecompositionPathPlanning/blob/c90ffa61f4b84c3c09b41c1ba5a837f49d44d278/CMakeLists.txt)。
- C++14 + OpenCV/Eigen，无 ROS；`ReadMap()` 用 `cv::imread()` 读取图像，随后处理外边界和内部障碍，`ExecuteCellDecomposition()` 根据扫线事件分裂/合并 cell。PGM 可由 OpenCV 读取是格式兼容推断，本次未实际运行该仓库读取本项目地图。
- 输入原预处理在 128 处二值化，常见 unknown 灰度可能被当作自由；必须先按项目统一占据语义输出黑白图，不能直接沿用其默认阈值。
- 跨区路径使用 cell 邻接图 BFS、`FindLinkingPath()` 与 `WalkInsideCell()` 等边界连接；仓库附带 `a-star.hpp`，但主程序没有 include 或调用它，不能宣传已有栅格 A* 重连。
- `main()->TestAllExamples()` 的示例路径、分辨率、机器人参数有硬编码；需生成独立 CLI 包装/适配副本，保留原仓库只读，不直接覆写第三方源码。
- [GPL-3.0 LICENSE](https://github.com/RicheyHuang/BoustrophedonCellularDecompositionPathPlanning/blob/c90ffa61f4b84c3c09b41c1ba5a837f49d44d278/LICENSE)。固定 `master` 核查提交：`c90ffa61f4b84c3c09b41c1ba5a837f49d44d278`（提交日期 2020-12-07）。
- [README](https://github.com/RicheyHuang/BoustrophedonCellularDecompositionPathPlanning/blob/c90ffa61f4b84c3c09b41c1ba5a837f49d44d278/README.md)明确说明是练习/未完成项目；适合算法实验，不能当成成熟可部署导航包。

## 补充候选：Nobleo FCPP 是 BSA

- [固定 README](https://github.com/nobleo/full_coverage_path_planner/blob/113e627b2864e0a69de89185a3bc18f04e98bf1f/README.md)明确称 Backtracking Spiral Algorithm，引用 Gonzalez 等 2005 年 BSA 论文。
- [spiral_stc.cpp](https://github.com/nobleo/full_coverage_path_planner/blob/113e627b2864e0a69de89185a3bc18f04e98bf1f/src/spiral_stc.cpp) 中 `SpiralSTC::spiral_stc()` 用 `spiral()` 加 `a_star_to_open_space()` 寻找剩余自由格；未见生成 STC 树。应标为 **BSA / spiral + A***，不是标准 STC。
- C++、ROS 1 move_base/MBF 插件；实际 makePlan 从 `static_map` / `GetMap` 取得 OccupancyGrid。核心可接受 bool 栅格（True 为障碍，与 PythonRobotics 相反），有 robot/tool radius 参数。
- [parseGrid](https://github.com/nobleo/full_coverage_path_planner/blob/113e627b2864e0a69de89185a3bc18f04e98bf1f/src/full_coverage_path_planner.cpp) 仅以占据值大于 65 判障碍，unknown=-1 不会自动成为障碍；半径转离散窗口及边缘截断也需统一核查。建议抽核心做离线实验，避免整包移植。
- [Apache-2.0](https://github.com/nobleo/full_coverage_path_planner/blob/113e627b2864e0a69de89185a3bc18f04e98bf1f/LICENSE)，固定核查提交 `113e627b2864e0a69de89185a3bc18f04e98bf1f`。README 说明公司已不再使用该包，但仍接受贡献。

## 接入现有 benchmark 的建议

1. 保留现有 Leader / Fields2Cover 结果，新增候选独立目录和固定版本；不要改动已固定的 F2C 或原 Leader 源码。
2. 优先做离线规划适配：PGM/PNG + YAML 或同一 OccupancyGrid → 统一自由/未知/障碍掩膜 → BCD/STC 核心 → 世界坐标完整路径 JSON。
3. BCD 原生排序/连接与新增公共 A* 修复版必须分开命名；STC 的子格尺度、2×2 粗化、补边和丢失区域需单列，不能把原算法换掉后仍当作原生结果。
4. 继续复用现有公共长度、扫掠并集、碰撞/越界、精确 EDT 净空与双分母指标；另加转弯数、重复覆盖和粗化损失时先明确统计定义。文件预处理、算法计算、ROS 调用耗时分别统计。
5. 所有新返回路径仍需逐段检查，不假定 BCD 或 STC 名称本身保证实际可通；本轮未执行上述接入和测试。

## 研究范围

来源为公开仓库源码、官方 README/文档、LICENSE/包声明及提交记录，已把仅形状像弓字/螺旋的项目与真正 BCD/STC 区分。仓库是否具有近期整体提交不等于对应算法持续维护；未以星数替代可用性判断。本轮没有将 ROS 1 兼容说明当成 Jazzy 实测证据，也没有采用其他项目自报覆盖率来预测本项目结果。
