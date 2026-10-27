# Leader PNG 地图规划验证（2026-10-27）

## 输入与参数

- 用户指定 `data/maps/imported/A11123AC22_231213_163036/A11123AC22_231213_163036.png`，2454 × 1595。直接读取 PNG，没有替换成像素内容不同的同名 PGM；地图服务输出与 PNG 的白色自由、黑色占据、灰色未知分类逐格一致。
- PNG 缺配套 YAML；参考同尺寸、同名 PGM 文件头，临时采用 0.05 m/像素、局部原点 `[0,0,0]`、trinary 阈值 free=0.25 / occupied=0.65。这些是本次规划假设，未恢复真实世界坐标。
- 全图范围内缩一格的矩形选区，起点选择已知自由空间距障碍最远的格子中心 `(24.225,33.675)`，仅用于规划，不代表机器人位置。
- Leader 原算法的构建期 ROS 2 适配，弓字模式、间距 0.7 m、净空参数 1.0 m，island 输出保持禁用。使用现有构建；21 个原始源文件与安装清单哈希一致，适配源码早于现有可执行文件，未修改业务或第三方代码。

## 本次结果

- 两次真实 `/leader_coverage/plan` 调用均成功且点数、长度一致：7,918 点、500.3897 m。首次调用约 0.7222 s；清理有限运行探针后复跑约 0.7367 s，后一次结果保存在 [result.json](result.json)。耗时包含 ROS 传输。
- 独立按不超过半格间距采样全部线段，34,162 个样本全部处于地图范围内的已知自由空间；按格子中心欧氏距离变换测得最小障碍/未知空间距离约 0.9962 m。不是整机 footprint、动力学或跟踪可执行性验证。
- 已目视检查 [路径叠加图](coverage.png) 与 [原图/路径对照图](comparison.png)。上方支路、下方长通道及右侧部分区域仍有明显遗漏；未量化覆盖率，不能声称全图覆盖。
- 最后一次日志记录 577 个粗格访问步骤、33 组粗路径、26 组细路径；合并时 `path[11]` 因 A* 连接失败被跳过。诊断摘要见 [diagnostics.json](diagnostics.json)。日志中的 `loop reaches max(9000)` 在原函数尾无条件打印，不能单凭这条日志认定触发循环上限。
- PNG SHA-256 为 `f06dd04d9b02db67dd850f04f06f628c326b3b72420a904f53edce7e40b10a5b`；[source_hashes.json](source_hashes.json) 保存原算法指纹。地图原文件未修改。
- 本次仅启动地图、生命周期管理器和规划服务，没有启动 Gazebo、Nav2 控制或运动任务；验证后本次进程已关闭。

## 复现

本地原图需要存在；先加载已有构建，在项目根目录的两个终端分别执行。使用独立 `ROS_DOMAIN_ID=97`，避免与其他会话交换地图或服务请求。

两个终端都先执行：

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
source leader_coverage_ws/install/setup.bash
export ROS_DOMAIN_ID=97
export ROS_LOG_DIR=/tmp/bg1_leader_AC22_20261027_logs
```

终端一：

```bash
ros2 launch "$PWD/leader_coverage_ws/docs/verification/imported_A11123AC22_20261027/planning.launch.py" \
  > leader_coverage_ws/docs/verification/imported_A11123AC22_20261027/planning.log 2>&1
```

终端二：

```bash
python3 leader_coverage_ws/docs/verification/imported_A11123AC22_20261027/probe.py \
  > leader_coverage_ws/docs/verification/imported_A11123AC22_20261027/probe.log 2>&1
```

探针会覆盖本目录结果、指纹和图片，完成后自动退出；终端一用 Ctrl+C 关闭。本次实际执行的状态检查为 `ros2 lifecycle get /map_server`（`active [3]`）、`ros2 service list -t`（规划接口存在）和 `ros2 node list`（仅本次三个节点）。日志留在本地并由既有规则忽略。

## 选区与通道补充核验

- 运行 `python3 leader_coverage_ws/docs/verification/imported_A11123AC22_20261027/analyze_passages.py`，读取已保存请求并复现几何筛选；没有重跑 ROS 规划或修改参数。[诊断图](passage_analysis.png) 左图黄色虚线为实际选区，全部 645,425 个白色自由像素均在框内，框外自由像素为 0；上次叠加图未画此选区线。
- 选区坐标约为 `(0.05,0.05) → (122.65,0.05) → (122.65,79.70) → (0.05,79.70)`。米制数值仍依赖 0.05 m/像素假设，完整参数及测量保存在 [passage_analysis.json](passage_analysis.json)。
- 项目 `bg1_description/config/nav2_footprint.yaml` 中外廓宽为 0.760 m，配置的最小物理通道宽为 0.850 m；这不是本地图整条通道的可执行性证明。当前 Leader 的 1.0 m 参数是路径点到障碍的中心净空，直通道通常需要超过约 2 m；还要容纳粗格检查窗口，因此仅局部宽于机器人不足以保留覆盖格。

| 图中标记 | 测量位置 | 白色连续截面 | 本次几何筛选结果 |
| --- | --- | --- | --- |
| A | 左下纵向通道，y≈15 m | 70 像素，约 3.50 m | 截面中点保留净空，但膨胀后与起点主区断开，未保留到任务区域 |
| B | 中下纵向通道，y≈25 m | 136 像素，约 6.80 m | 同上；局部宽度并非入口瓶颈宽度 |
| C | 左上纵向通道，y≈62 m | 46 像素，约 2.30 m | 膨胀后与起点主区断开，未保留到任务区域 |
| D | 中上纵向通道，y≈60 m | 63 像素，约 3.15 m | 净空连通且保留到任务区域；局部粗格候选仍受固定采样对齐影响，已有路线穿过此段 |
| E | 上部横向连接，x≈40 m | 49 像素，约 2.45 m | 净空中心通路仍连通，但截面附近放不下完整粗格检查窗口 |
| F | 顶部 T 形横向段，x≈50 m | 53 像素，约 2.65 m | 同上，不能将缺少覆盖格解释成没有中心通路 |

- 离线复现相同 OpenCV 5×5 距离变换和 20 像素膨胀，产生 20 个自由连通分量。`task_helper.cpp::findBoudPath()` 只选最大外轮廓，`fillCostMat()` 将其外部排除；诊断图右侧紫色即膨胀后仍有自由点、但未保留到任务区域的部分。它不是“矩形没框住”。
- `FreeSpaceCalculator.cpp` 在 14 像素粗格中心检查 `[-7,+7]`，实际窗口为 15×15 个像素；碰到膨胀障碍就拒绝该格。直通道的理想连续尺度约需 `2×1.0+0.7≈2.7 m` 才可能容纳窗口，还受栅格量化、采样对齐、壁面倾斜和局部杂点影响，不能当成精确统一阈值。
- 测量是局部轴向截面；JSON 中的瓶颈半径是基于自由像素中心、精确欧氏距离变换和八邻接的圆形净空诊断，不包含长方形车体朝向、转弯及运动跟踪。粗格部分是离线几何复现，没有宣称复现完整 C++ 遍历。

## 验证范围

- 已执行：真实规划服务调用、PNG/加载地图逐格比较、全路径线段加密采样、原图及源码哈希、结果点数/长度一致性、探针/启动文件 AST、YAML、PNG 完整性、Markdown 本地链接及差异空白检查；补充全图选区包含关系、六处截面和离线膨胀/轮廓/粗格几何筛选。
- 未执行：重建、包测试、Gazebo 仿真、机器人实际运动、整机净空检查和全区覆盖率验收；本次没有业务代码变更。
