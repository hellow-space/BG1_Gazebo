# Leader / Fields2Cover 同图规划 Benchmark（2026-10-27）

## 结论

本次是固定版本原生规划流水线的基础对比，不是最优参数搜索或算法普遍优劣排名。两套接口均成功返回路径，但 Fields2Cover 的部分连接穿过原图非自由空间，不能将它的几何覆盖率解释成完整可执行覆盖。

| 指标 | Leader | opennav_coverage / Fields2Cover |
| --- | ---: | ---: |
| 完整输出路径长度 | 1,060.91 m | 2,472.52 m |
| 几何扫掠覆盖率：全图自由面积分母 | 46.5613% | 71.0212% |
| 净空合格片段扫掠覆盖率：全图分母 | 46.5607% | 58.4378% |
| 几何扫掠覆盖率：共同目标区分母 | 50.1681% | 76.3431% |
| 净空合格片段扫掠覆盖率：目标区分母 | 50.1675% | 62.9647% |
| 净空合格片段扫掠面积 | 590.40 m² | 741.01 m² |
| 经过非自由格的线段总长 | 0.00 m | 161.97 m |
| 未通过严格净空栅格审计的线段总长 | 0.36 m（6 段） | 767.13 m（14,074 段） |
| 通过严格净空审计的长度比例 | 99.9659% | 68.9736% |
| 完整中心线路径通过原图自由格检查 | 是 | 否 |
| 完整路径通过严格净空栅格检查 | 否 | 否 |
| ROS 规划调用耗时中位数，预热后 3 次 | 0.4507 s | 0.9987 s |
| 路径点数 | 20,191 | 47,658 |

- **几何覆盖率**：即使某些连接穿障碍，也把返回路线的扫掠带与自由区求交，属于忽略可执行性的名义数值。
- **净空合格片段覆盖率**：仅统计整个线段通过共同净空栅格审计的片段。这些片段可能断开，不能解释成机器人可连续完成的覆盖。没有裁剪/修复原路线，也没有以直线“传送”补连后宣称成功。
- Leader 未经过原图障碍/未知格，但独立严格距离审计发现 6 个短段跨过临界栅格。原生 OpenCV 5×5 近似距离口径会接受部分精确距离约 0.996 m 的格子；本次独立审计使用精确 EDT，保留这一差异，不宣称连续几何严格 1 m 净空成立。
- Fields2Cover 对本次复杂非凸区域生成了 120 条条带；原生 Dubins 连接未通过地图障碍/净空检查。固定版本的连接器以车辆和条带端点为输入，没有接入栅格障碍约束。没有加入区域分解、障碍感知重连或算法修改来补救。
- Leader 原生仍有粗格、最大外轮廓和短路径过滤等限制，未覆盖完整场景。路径更短可能意味着遗漏更多，不能单独当成更优。

[路径与覆盖对照图](comparison.png)：上排青色/红色为通过/未通过精确 EDT 净空栅格的线段；下排绿色为合格片段扫掠、橙色为仅名义扫掠、浅色为遗漏。灰色是原图占据或未知空间。

## 共同输入与参数

- 直接使用 `data/maps/imported/A11123AJ04_231127_094529/A11123AJ04_231127_094529.png`，1208 × 1710。SHA-256：`88ce806ef09044c2b57ba86e7dc3befab7a177b8a07690899900fc4ffca2b016`。没有换成 PGM，之前 PGM 历史结果不作为本次数据。
- 缺少 PNG 原始 YAML，参照同名 PGM 文件头暂用 0.05 m/像素、局部原点 `[0,0,0]`。使用 ROS OccupancyGrid 的 float32 分辨率数值，完整值见 [inputs.json](inputs.json)。米制结果依赖此尺度假设。
- 两者统一 trinary 分类：占据概率 `(255-gray)/255`；小于 0.25 为自由，大于 0.65 为占据，中间为未知。因此 **PNG 的 200 灰度也属于自由**。全图共有 507,209 个自由格，约 1,268.02 m²。
- 最大四邻接自由连通区为共同目标：470,743 格、约 1,176.86 m²，占全图自由面积 92.8105%。其他断开区域两者都不输入规划器，但仍计入全图覆盖率分母；目标区分母另列，避免隐藏范围损失。Leader 收到全图矩形选区及该目标掩膜，目标之外全部视为障碍。
- Fields2Cover 需要多边形而不是 PNG。用现有 GDAL 按像素方块边界矢量化同一目标，不简化成外接矩形/凸包：1 个外环、92 个内孔，共 2,779 个顶点，几何有效，面积相等，栅格回填与目标掩膜逐格一致。[target_polygon.json](target_polygon.json) 与 [target_map.png](target_map.png) 保存实际输入。
- 作业宽度与条带/粗格间距统一为 **0.6 m**；这是演示作业宽度假设，未标定为 BG1 实际清洁宽度。扫描方向统一为水平。
- Leader：弓字模式、净空 1.0 m，保持现有适配与 island 禁用设置。
- Fields2Cover：固定水平条带（其角度参数为 π/2）、Boustrophedon 排序、1.0 m headland、Dubins 连接、最小转弯半径 0.6 m、既有离散曲率设置。[f2c_config.yaml](f2c_config.yaml) 为实际服务配置。**headland 与 Leader 栅格膨胀不是等价算法**；二者输出均由同一独立审计口径检查。
- 请求起点选择共同目标区距非自由区最远的格子中心，约 `(34.375,52.475)`。Leader 将其映射到粗格，实际路线起点偏差约 0.25 m；Fields2Cover 1.2.1 的此接口不接收起点约束，实际起点距该点约 20.63 m。保留原生起点，两者均不计起点接近路径，也不人为拼接连接线。
- 两套 overlay 分开加载，顺序运行于 ROS_DOMAIN_ID 98 / 99；分别预热一次、测量三次。各自四次输出路径指纹相同，完整调用时间保存在原始结果中。

## 指标定义与限制

- 长度为全部输出相邻路径点的欧氏距离之和，包含转弯和连接；无效段也包含在总长内，另列无效段总长，未将它们删除来缩短路线。表中的“非自由线段总长”表示整个线段被判为不合格后计入，不是精确的障碍内部弧长。
- 覆盖范围是每条线段半径 0.3 m 的胶囊形扫掠并集；逐个栅格中心用解析点到线段距离判断，重复经过不重复计面积，最后与相应自由区域求交。统计包含所有移动段均开启工具的几何假设，不区分作业条带与转弯时工具开关。
- 原图通行性按每个线段实际经过的栅格检查；在全部横/纵格线交点处分段，以各子段中点确定经过格，避免固定步长漏掉很短的障碍穿越。仍只是二维中心线栅格检查。
- 严格净空允许格由 **SciPy 精确欧氏距离变换**独立生成：目标格中心到非目标格中心距离严格大于 `ceil(1.0/resolution)=20` 像素。线段经过的全部格都必须允许。该口径与连续障碍方块边界、真实机器人长方形 footprint、跟踪误差不同，不能称作整机实际 1 m 净空认证。
- 耗时为 `perf_counter` 测量的 ROS 请求到完整响应，预处理、启动和结果 JSON/指标/绘图不计入。Leader 为 service，F2C 为 action，响应点数与传输量不同；不能直接解释为纯算法耗时。F2C 自报规划时间中位数为 0.2816 s，Leader 没有等价服务端计时字段，因此不将二者混比。
- 本项目固定的 Fields2Cover 1.2.1 分支主要面向凸场地。这里直接测试带凹边界及内孔的复杂地图，用于展示当前集成的原生行为；结果不代表更高版本、添加区域分解后的能力，或库的最佳可达表现。
- 只测这一张图、这一组参数和三次计时，不具备统计泛化意义；没有参数调优、连续运动、定位、跟踪、整机净空或真机清洁验证。

## 版本与文件

- Fields2Cover：`87a91dbd9a7e762b15706b5337f7828f02a584ee`，`v1.2.1-devel`；opennav_coverage：`194250501889ce8896c5ee8b80a4d1317750be0e`，`jazzy`。
- Leader 的 21 个原始文件 SHA-256 与现有构建清单一致，服务可执行文件和 F2C 库指纹也保存在 [inputs.json](inputs.json)。原图、业务代码、第三方源码及固定提交均未修改；使用已有构建，没有重建底层或 overlay。
- [summary.csv](summary.csv) / [summary.json](summary.json)：基础表格和完整指标。
- [leader_raw.json](leader_raw.json) / [f2c_raw.json](f2c_raw.json)：四次调用状态/时间/指纹及最后一次完整路径。
- [inputs.npz](inputs.npz)：共同分类/目标/精确净空掩膜；保留 Leader 近似距离允许格用于解释原生检查差异。
- [leader_evaluated.npz](leader_evaluated.npz) / [f2c_evaluated.npz](f2c_evaluated.npz)：复算后的线段标记和扫掠掩膜。
- `.log` 和 `ros_logs/` 是本地日志，由本目录 `.gitignore` 忽略；原始 JSON 路径与摘要保留。

## 复现

在项目根目录运行。需要本地 PNG、现有 Jazzy/底层/两个 overlay 的构建及 NumPy/SciPy/OpenCV/Pillow/Matplotlib/PyYAML、系统 GDAL 共享库。脚本直接调用已安装 GDAL C API，不需要安装 Python GDAL 或 Shapely。

先准备共同输入并运行指标解析测试：

```bash
python3 docs/benchmarks/2026-10-27-A11123AJ04/prepare_inputs.py
python3 -m pytest docs/benchmarks/2026-10-27-A11123AJ04/test_metrics.py \
  docs/benchmarks/2026-10-27-A11123AJ04/test_raster_geometry.py -q
```

独立终端 A，仅加载 Leader：

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
source leader_coverage_ws/install/setup.bash
export ROS_DOMAIN_ID=98
export ROS_LOG_DIR="$PWD/docs/benchmarks/2026-10-27-A11123AJ04/ros_logs"
python3 docs/benchmarks/2026-10-27-A11123AJ04/run_planner.py --planner leader
```

等待 A 完成，再在新终端 B 仅加载 coverage overlay：

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
source coverage_ws/install/setup.bash
export ROS_DOMAIN_ID=99
export ROS_LOG_DIR="$PWD/docs/benchmarks/2026-10-27-A11123AJ04/ros_logs"
python3 docs/benchmarks/2026-10-27-A11123AJ04/run_planner.py --planner f2c
```

两个探针都会自行启动、配置并关闭自己的规划器。没有启动地图服务器、Gazebo 或运动节点；Leader 的共同地图由探针以 transient-local QoS 发布。

复算并绘图：

```bash
python3 docs/benchmarks/2026-10-27-A11123AJ04/analyze_results.py
python3 docs/benchmarks/2026-10-27-A11123AJ04/verify_results.py
git diff --check
```

这些脚本会覆盖本目录对应结果；如需保留本次计时，复现前先复制目录。

## 本次验证

- 已实际调用两套真实接口，各 1 次预热和 3 次测量，输出均重复一致；不是复用历史路径。
- 指标测试先观察缺失实现失败，再通过；独立审查发现近似距离审计的问题后，增加精确距离回归用例并先失败再通过。当前 10 项解析用例通过，涵盖长度、扫掠并集/端帽、障碍穿越、极短交叉、越界、精确距离及多边形内孔/断开区域。
- 独立审查确认共同地图、孔洞、回填、指标和时间口径；已修正其提出的近似距离问题，并移除不成立的“可覆盖上界”附加指标。
- 已复算 JSON/CSV、路径指纹、扫掠与线段掩膜，检查原图/源码/版本哈希、图像、AST/YAML、本地链接及进程退出；详见 [verification.json](verification.json)。
- 没有修改业务或第三方算法；未运行无关包测试、重建、Gazebo、运动或 Git 提交/推送。
