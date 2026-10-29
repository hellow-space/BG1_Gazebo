# Status

## 当前目标

ETHZ Polygon Coverage适配和三图实际规划、五算法统一对比已完成（2026-10-29）；功能提交为 `25d124c`，远端推送因本机缺少GitHub认证失败，未合并 `main`。

## 已完成

- 完成 [五算法统一对比](docs/benchmarks/five-planners-2026-10-29/README.md)：新增ETHZ三次真实规划，与历史Leader/BCD/STC/F2C修改版12组完整路径同图复算；15组只列覆盖率、完整长度、共同栅格有效性，三张五列轨迹/扫掠图已目视核对。
- ETHZ AJ04/AC22/C03为57.6468% / 26.3248% / 2.8335%，1596.87 / 1313.23 / 147.31 m；均为闭合完整路线，通过原图/1 m精确净空栅格审计，未启动仿真或运动。
- 上游固定子模块 `e4e977e5a52de72665f6f5f82e5770cf092068e1`，自有headless入口/兼容层独立部署；本地CGAL/Mono及原生GK-MA完成构建，原生BCD、最佳方向扫描、可见图和GK-MA负责规划。原文件未改；生成副本显式固定CGAL输出核、精确移除冗余共线点并核验面积不变。
- 为控制像素多边形复杂度，输入采用0.2 m全自由块和分块后起点分量；AJ04/AC22/C03从原起点净空分量舍弃8511 / 17957 / 14931格（原为278927 / 170405 / 20259格）。C03范围明显缩小，不能只比较长度；孔洞可能连到外部障碍变凹口，未将障碍填成自由。最终审计和覆盖分母仍使用原分辨率原图。
- 原生cell偏置失败、共线顶点断言、全方向/完整像素超时均单独保留输入、日志、空路径/未完成状态和运行源码快照；正式报告链接这些诊断。独立审查发现的中断残留、旧输出混入和依赖指纹问题均以先失败后通过测试修复；本次共11项ETHZ行为/真实集成用例，55项匹配回归全部通过。
- 工作位于当前目录的 `feature/ethz-polygon-coverage`，新增文件和修改可直接在IDE查看；用户现已明确授权提交和推送该分支。既有三算法/F2C/四算法报告原数据保持不变。

- 完成 [Headland特写与两算法分镜](docs/diagrams/coverage_logic_2026-10-29/README.md)：3张PNG及可放大SVG，用同一带障碍地图展示输入、膨胀/负buffer、粗格、遍历、分段/Bezier、扫描线求交、排序和连接；自有避障层另标，不冒充原生功能。
- 保存确定性生成脚本、示例参数与几何/源码指纹；红色穿障连接由固定F2C库实际生成教学Dubins曲线，绿色绕障线在教学净空图上计算。所有图均标明教学示意，不是benchmark实测路线；算法和既有数值结果未改。

- 已将保存的12组完整轨迹汇总为 [四算法统一对比](docs/benchmarks/four-planners-2026-10-28/README.md)：同一表格只列覆盖率、路径长度、有效性；每张地图四列并排显示轨迹/扫掠，保留失败、Leader净空临界段和BCD原生内存异常标记。地图/参数/起点元数据一致，所有指标复算与原记录一致。
- 新增 `coverage_benchmark.compare_saved`，来源JSON/两份manifest和汇总代码/图片/CSV均记录SHA；不复制改写原始路径，不重新运行任何规划器。旧报告及模块README已加统一总览入口。

- 新增自有 F2C 地图感知流水线：精确净空起点分量、原生条带、精确裁剪、原生转弯审计、8邻接A*及整路核验；第三方原文件/固定提交、既有 ROS/运动接口和历史结果不变。模块职责、逐段路径来源和运行源码/原图/栅格/二进制/依赖指纹见 [设计](docs/benchmarks/f2c-obstacle-design.md)。
- 三图原生/修改版共六组真实规划完成，修改版 AJ04/AC22/C03 覆盖率为58.8749%/31.1626%/9.1025%，完整长度2350.46/3961.54/2314.24 m，三条路线均通过共同原图/1 m精确净空栅格审计。见 [完整对照及轨迹图](docs/benchmarks/f2c-obstacle-2026-10-28/README.md)。范围为公共起点可达净空区，原图全部自由像素仍作分母，不声称全区完成。
- C03 可选原生转弯曾出现OOM终止；已保留诊断，增加2 GiB原生地址空间上限并修复条带被丢弃的问题。最终受限原生转弯仍失败，但成功返回的146条真实有序条带保留，119次A*生成并核验完整路线；原生失败未改记为原生成功。

- 之前工作本地提交为 `0f54459`；本次用户明确取消 push，不再重试远端推送。
- 两个新候选已固定为 Git 子模块，独立 `coverage_benchmark/` 提供公共读图、净空、起点、三套 adapter、完整路径、指标/绘图和复算入口；上游原文件保留不改，现有 ROS 工作区接口不变。
- 完成 AJ04、AC22、C03 三张 PNG × 三算法九组真实规划；统一作业宽度 0.6 m、中心净空 1.0 m，原图全部已知自由像素为分母。五组返回非空路径，四组明确失败；失败长度为空，不计零长度优势。结果见 [三场景 benchmark](docs/benchmarks/three-planners-2026-10-27/README.md)。
- 当前 AJ04 Leader/STC 为 46.56%/43.08%、1,060.91/927.00 m；AC22 Leader/BCD/STC 为 18.49%/12.15%/6.80%、607.63/525.38/185.40 m。Leader 两图有少量严格净空临界段，三条 BCD/STC 返回路线通过共同栅格审计；不代表运动或全区覆盖。

- 完成 [BCD / STC 候选核查](docs/benchmarks/2026-10-27-coverage-candidates.md)：确认 PythonRobotics Spiral-STC 实际生成树与四子格机制；IPA 标准 BCD 为配置 2（默认 8 是变体），Richey C++ BCD 为无 ROS 实验候选；Nobleo 虽类名 SpiralSTC，实装为 BSA，已单独分类。记录官方源码、输入语义、固定提交、许可及适配边界，未安装、构建或运行新候选。
- 核查发现：IPA 的源码头与包许可声明不同，不能简写成纯 LGPL；IPA 中有 A* 但返回路径的跨 cell 拼接仍须验证。Richey 附带的 a-star.hpp 未集成到 main，跨 cell 使用图遍历/边界连接，不能据文件名宣称已有 A* 障碍重连。PythonRobotics 需统一栅格尺度、2×2 粗化及完整线段间连接后再比较。

- 完成 [同图规划 benchmark](docs/benchmarks/2026-10-27-A11123AJ04/README.md)：直接使用用户指定 AJ04 PNG，共同目标为最大四邻接自由连通区（全图自由面积的 92.8105%），统一作业宽度/间距 0.6 m；F2C 输入保留 92 个内孔，精确像素边界矢量化后回填逐格一致。两套 overlay 分开、顺序运行，各 1 次预热及 3 次测量，未改算法或启动运动。
- 本次 Leader / F2C 路径长分别为 1,060.91 / 2,472.52 m；全图自由面积分母的几何覆盖率为 46.5613% / 71.0212%，独立精确净空审计合格片段的覆盖率为 46.5607% / 58.4378%。ROS 调用耗时中位数为 0.4507 / 0.9987 s，不作为纯算法计时比较。
- Leader 中心线未穿原图非自由格，但精确净空审计仍有 6 个临界短段（合计 0.36 m）不合格；F2C 有约 161.97 m 线段经过非自由格、约 767.13 m 未通过严格净空栅格审计。合格片段覆盖率不代表连续可执行覆盖；两者都没有运动验收。
- 已保存完整路径、JSON/CSV、输入与版本指纹、对照图及复现脚本；独立审查发现的近似距离问题已用精确 EDT 修正，并删除不成立的可覆盖上界附加指标。10 项解析用例及完整数据复算通过，本次规划器均已退出。

- 已补充 [全图选区与通道诊断](leader_coverage_ws/docs/verification/imported_A11123AC22_20261027/README.md)：645,425 个已知自由像素全部在请求框内；六处局部截面宽 2.30～6.80 m。1 m 膨胀后产生 20 个自由连通分量，原算法仅保留最大外轮廓，下方宽通道也因入口瓶颈断连而被排除；上部 2.45/2.65 m 的局部通道仍有中心通路，但不能容纳当前粗格检查窗口。没有修改算法、参数或重跑运动。

- 直接使用指定 PNG 完成两次真实 Leader 服务调用，路线均为 7,918 点、500.3897 m，最后一次耗时约 0.7367 s；间距 0.7 m、净空 1.0 m。PNG 缺元数据，临时采用同名 PGM 文件头的 0.05 m/像素及局部原点，未使用 PGM 替代 PNG。
- 已保存 [本次 PNG 规划记录](leader_coverage_ws/docs/verification/imported_A11123AC22_20261027/README.md)、路线/参数 JSON、叠加图、诊断和复现探针；独立采样 34,162 个点全部处于已知自由空间。日志显示一组细路径 A* 连接失败被跳过，图上仍有明显未覆盖区域。没有运动任务，验证后本次节点已关闭。

- Git 收录策略已整理：保留 Leader 的设计、计划、进度及验证摘要/截图/复现脚本；忽略导入地图验证目录的运行日志，以及约 136.44 MiB 的原始 `motion/runtime.json`，本地文件保留。根目录项目上下文文档一并收录。
- 导入地图运动说明已明确区分快照状态与后续关闭前的 Nav2 code=105；没有将局部运动证据写成全程完成。第三方算法、地图和业务代码未改。

- 按用户要求，静态船模、配置、生成工具、测试、GUI、独立入口及全部截图/验证记录已移至桌面 [Workboat_Gazebo](/home/jimmy/Desktop/Workboat_Gazebo/README.md)，工程不再集成船模。入口为 `bash /home/jimmy/Desktop/Workboat_Gazebo/run.sh`；Gazebo 保持关闭。
- 迁移前对 18 个文件记录哈希，移动后全部一致；修改仅涉及独立入口和说明，原模型网格保持原样。包安装/测试/说明入口已移除，船模安装软链接已按来源清理。

- 接续用户“把机器放进去跑”的要求：从同一原 PGM 生成本地 1.2 m 高柱体网格世界（1,688 矩形/20,256 三角面），投影与地图非自由格逐格一致；复用现有 BG1、传感器、footprint、Nav2/RPP 和 0.3 m/s 限速。AMCL 为唯一定位来源，没有加载 coverage overlay。
- 实际执行起点接近成功并跟踪完整 Leader 路线；[本次运动检查](leader_coverage_ws/docs/verification/imported_A11123AJ04_20261022/motion/README.md) 快照记录 Gazebo 实际行程 37.775 m、完成首次弓字掉头，AMCL/真值位置差最大 0.157 m。快照为 `following`，不是全程完成。

- 本次使用原始 `A11123AJ04_231127_094529.pgm`，独立启动地图服务和 Leader ROS 2 规划服务；弓字规划成功，16,988 点、897.3167 m、服务调用约 0.4347 s。原地图和第三方算法未修改，21 个原始源文件哈希匹配构建清单。
- 保存 [导入地图验证记录](leader_coverage_ws/docs/verification/imported_A11123AJ04_20261022/README.md)、完整路径 JSON、叠加图、复现探针及日志。临时采用局部原点和 trinary 阈值；未恢复原始地图世界坐标。

- 核对 Git、根目录及三个工作区说明、设计/进度记录和关键代码/脚本；初始化前仅八份根目录 Markdown 为未跟踪文件，未发现已跟踪文件改动。
- 补充 `AGENTS.md` 项目约定；填写 `goal.md`、`plan.md`、`structure.md`、`runbook.md`、`experiences.md`。保留通用规则与模板主要结构。
- 文档核对后删除 `初始化prompt.md`，初始化要求已落实到上下文文档和长期工作规则。
- 现有功能证据（历史记录，本次未重跑）：
  - 目录整理后 5 个底层包、2 个 Leader 包和 4 个 coverage 包重建；底层/Leader/coverage 适配 colcon 分别 45/23/23 项通过。见 [目录整理记录](docs/development/2026-10-22-project-layout.md)。
  - demo 导航往返成功，实际终点误差约 0.161/0.163 m；Fields2Cover 7 条带、35.71 m 路线完成，误差约 0.188 m。见各工作区 README。
  - Leader 已支持手动执行与取消；120 点短段实际行程约 5.87 m，终点误差约 0.206 m；停止及改选区取消有停车记录。见 [Leader 进度](leader_coverage_ws/docs/progress.md)。
  - warehouse/office 导航规划探针与静止 SLAM 启动已验证，不能推断全程运动或完整建图通过。见 [大场景记录](ros_ws/docs/development/2026-10-21-large-scenes.md)。

## 当前进行

本轮离线适配及提交前55项测试、15组证据复算均已完成；成果已提交至 `feature/ethz-polygon-coverage`，推送待本机GitHub认证可用后重试，不启动仿真/运动。

## 当前问题

- ETHZ本次是0.2 m保守多边形输入＋每cell原生最佳方向模式，额外舍弃细通道且要求闭合回起点，与F2C修改版范围/终点不同；不是只替换排序器的消融或全局最优性证明。完整像素模式在AJ04超过180秒，CGAL/Mono仍为Ubuntu Noble的本地依赖适配。

- 新 F2C 版本保留原生条带排序，跨条带往返仍很长，未优化区域分解/顺序。几何路径允许停车转向，不满足 Dubins 最小半径/曲率保证；原生条带/转弯的零长度触角边界语义与A*不同，三图未见影响，本轮明确保留该限制。连续 footprint、运动和真机均未验证。

- BCD AJ04 分解产生不等长边缘，被 wrapper 拒绝；绕过 guard 后 ASAN 证实原生 sweep 独立堆越界。AC22 正常构建虽返回几何有效路径，ASAN 同样查出原生堆越界（`main.cpp:381`），不能认定实现稳定；见 [BCD 诊断](docs/benchmarks/three-planners-2026-10-27/bcd_diagnostics.md)。
- C03 在统一参数下：Leader 返回空路径，BCD 原生轮廓提取 SIGSEGV，STC 净空/粗化后无完整 2×2 自由粗格。Richey 额外 0.55 m 内缩和最大外轮廓限制、STC 保守粗化和连通分量限制均未修复；这些是具体实现/参数的结果，不代表整个 BCD/STC 算法族能力。

- 本次 F2C 原生条带/Dubins 流水线在复杂非凸地图上产生无效连接，不能以其更高的几何扫掠率声称更优或完整覆盖。Leader 也存在遗漏与少量精确净空临界差异；当前只是单图、单参数的规划基线，起点约束及接口计时能力不完全相同，未做区域分解、参数搜索或运动验证。

- 本次 PNG 路线未覆盖完整选区；已确认膨胀断连/最大外轮廓选择、粗格筛选和一次 A* 路径组合并跳过均会造成遗漏，不能全部归因于机器人通不过。尚未量化遗漏面积，也未验证实际运动；PNG 的分辨率与局部原点仍为本次规划假设。

- 本次实际HTTPS推送返回 `could not read Username for 'https://github.com': terminal prompts disabled`；本机未安装 `gh`，没有默认SSH私钥且SSH agent无身份，SSH亦未配置信任GitHub主机。本次 `git ls-remote origin refs/heads/feature/ethz-polygon-coverage` 未返回该分支，尚未上传；需要先在本机完成GitHub Git认证。
- 船模仅依据截图比例复刻，缺原始 CAD 和尺寸；遮挡结构简化，不应当作精确船舶模型。

- 部分历史设计/进度含迁移前路径及旧阶段约束；当前入口与后续已批准扩展已在新文档中明确，历史文件原样保留。
- Leader 长路线全程、覆盖率及窄通道能力尚未验证；仿真测试不证明真机定位、动力学或清洁性能。
- 导入地图部分缺少配套 YAML 元数据，本地地图不随 Git 克隆，需要单独备份。
- 本次导入图规划可见部分支路遗漏；尚未量化覆盖率。现已生成二维栅格挤出的平地仿真场景并验证一段运动；柱高和局部原点为演示假设，不代表真实三维场地，未完成全路线/整机净空验收。

## 下一步

- 如继续优化ETHZ，单独评估保守栅格尺度、保持连通的边界简化或可见图性能；保留当前版本/参数/诊断和三图基线，避免将更小规划范围的短路程直接视为优势。

- 若继续优化新F2C方案，单独评估区域分解/条带顺序、统一连续接触规则或带曲率约束连接；保留当前版本/参数及溯源数据，勿用删坏段伪造完整路径。

- 后续若调参或修复 BCD 原生缺陷，另作新实验，保留当前固定版本/参数基线；IPA 尚未接入。
- 若继续改进 AJ04 benchmark，先确认 PNG 元数据和实际作业宽度，再确定是否增加障碍感知区域分解/连接或参数扫描；本次未扩大范围替换、升级或修复第三方算法。

- 如继续改进本次地图规划，先确认元数据，再明确物理通行、中心净空和覆盖粗格的不同要求，处理多轮廓及路径组合并遗漏；本次没有扩大范围修改算法或执行机器人运动。

- 船模后续操作在桌面 `Workboat_Gazebo` 完成；BG1 工程按后续任务继续。
- 当前没有仿真进程；船模使用方式见桌面独立目录 README。
- 维护本地地图独立备份；加载导入地图前补齐并确认物理尺度、原点与阈值。

## 验证结果

以下为2026-10-29用户授权提交/推送前的复验：

- `python3 -m pytest coverage_benchmark/tests docs/benchmarks/2026-10-27-A11123AJ04/test_metrics.py docs/benchmarks/2026-10-27-A11123AJ04/test_raster_geometry.py -q`：55项通过（3.75秒）。
- `python3 -m coverage_benchmark.verify_ethz`：15组原始轨迹、实现/输入/动态依赖指纹及独立几何见证全部通过；本轮未重跑三张大地图规划或运动。
- `git diff --cached --check` 通过；功能提交 `25d124c` 收录161个文件。`GIT_TERMINAL_PROMPT=0 git push -u origin feature/ethz-polygon-coverage` 因认证缺失退出128；远端未出现该功能分支，未创建PR、未合并main，两个worktree均保留。

以下为2026-10-29 ETHZ适配与五算法对比：

- `python3 -m coverage_benchmark.adapters.ethz_build`：本地CGAL 5.6/Mono依赖解包、原生GK-MA及headless核心编译完成；上游工作区无改动。依赖包、解包文件/符号链接、实际动态库/Mono程序集和源码/二进制均作指纹核验。
- `python3 -m pytest coverage_benchmark/tests/test_ethz.py -q`：11项通过；覆盖保守分块不填障碍/部分块排除、孔洞/起点分量、原生矩形和带孔闭合路线、真实超时、实际中断后进程消失、过期输出/验证清理、依赖缓存失效、优化模式拒绝和BCD共线点回归。新增行为均观察到对应失败后修复。
- `python3 -m coverage_benchmark.ethz_compare`：三图顺序实际运行完成，原生完整输出329 / 458 / 42点，26 / 55 / 9个BCD cell；相同参数再次运行指标一致。保存最终run_id `46b3425e743e4dcc8fe5b6d8a2c38389`，完整路线/原生输入输出/日志/来源及三张五列图。
- `python3 -m coverage_benchmark.verify_ethz`：15组原图/参数/文件和实际实现/动态依赖指纹、ETHZ原生输出逐点重放、起终点、三指标/CSV重算、非空路线各256个独立扫掠见证、图片解析全部通过；最终verification run_id与manifest一致。
- `python3 -m pytest coverage_benchmark/tests docs/benchmarks/2026-10-27-A11123AJ04/test_metrics.py docs/benchmarks/2026-10-27-A11123AJ04/test_raster_geometry.py -q`：55项通过。`python3 -m coverage_benchmark.verify`及`python3 -m coverage_benchmark.verify_f2c`分别通过历史9组、6组证据复算；历史报告/原始数据无Git差异。
- Python AST、Markdown围栏/本地链接、诊断快照/原artifact SHA和`git diff --check`通过，三张最终对照图已目视核对。进程检查未见本次ETHZ规划器残留；没有启动Gazebo/机器人运动，未提交或推送。


以下为2026-10-29教学分镜：

- `python3 docs/diagrams/coverage_logic_2026-10-29/render.py`：生成3套PNG/SVG；使用真实OGR负buffer计算Headland外边界/内孔，验证原生Dubins示例确实经过原障碍内部，教学A*路线通过对应允许栅格。
- `dubins_example.cpp` 通过g++编译，加载既有Jazzy/coverage环境后运行，输出教学曲线CSV；不启动ROS服务或运动。复现命令、源码索引、示例输入和范围已写入图集README。
- 几何检查：负buffer后的外边界为[1,1]～[15,9]，障碍孔偏置边界为[6,3]～[10,7]，条带区面积96.85984 m²；原生示例穿障点数为正，教学A*通过对应栅格。3组PNG/SVG可解析，中文/编号/箭头/颜色及裁切已目视核对，源码/CSV指纹和Markdown源链接通过；`git diff --check`通过。


以下为2026-10-28四算法保存结果汇总：

- `python3 -m pytest coverage_benchmark/tests/test_compare_saved.py docs/benchmarks/2026-10-27-A11123AJ04/test_metrics.py docs/benchmarks/2026-10-27-A11123AJ04/test_raster_geometry.py -q`：12项通过；新增测试先观察缺失模块失败，随后验证12组选择/失败保留和不同地图/参数拒绝。
- `python3 -m coverage_benchmark.compare_saved --runtime-root /home/jimmy/Desktop/BG1_Gazebo`：两份manifest的全部地图/公共参数一致，12组完整路径三指标复算与原结果一致，生成统一CSV、来源指纹、验证记录和三张四列轨迹/覆盖图。额外核验12行数据、14个源文件SHA、图片/CSV/帮助模块指纹、Markdown链接/围栏与Python AST，通过；目视核对并排图。旧实验目录仅README新增总览链接，原始数据/第三方均无变化，`git diff --check`通过。本轮未重新规划或启动运动。


以下为2026-10-28 Fields2Cover 地图感知扩展：

- 公共连接、裁剪、来源映射、指纹及原生失败回退测试均先RED后GREEN。`BG1_RUNTIME_ROOT=/home/jimmy/Desktop/BG1_Gazebo python3 -m pytest coverage_benchmark/tests -q`：32项通过，含真实ROS矩形原生服务。原生worker有锁串行化，未并发运行BG1会话或运动。
- `python3 -m coverage_benchmark.f2c_compare --runtime-root /home/jimmy/Desktop/BG1_Gazebo`：三图分别请求原生基线、独立有序条带和可选原生完整路径，保存六组比较；三条新路线的原图障碍/严格净空不合格段均为0。原生错误/资源限制明确保留，所有本次worker已退出。
- `python3 -m coverage_benchmark.verify_f2c --runtime-root /home/jimmy/Desktop/BG1_Gazebo`：源码/结果/原图/栅格/原生依赖及共享库指纹、多边形回填、从swaths重放完整路径和每段trace、CSV/三指标及每条路线256个独立距离见证通过。
- 回归命令 `BG1_RUNTIME_ROOT=/home/jimmy/Desktop/BG1_Gazebo python3 -m pytest coverage_benchmark/tests docs/benchmarks/2026-10-27-A11123AJ04/test_metrics.py docs/benchmarks/2026-10-27-A11123AJ04/test_raster_geometry.py -q`：42项通过；旧九组 `coverage_benchmark.verify` 再次复算通过，旧结果/第三方路径无差异。Markdown链接/围栏、Python AST、YAML及空白检查通过，目视核对三图。
- 一次独立代码审查指出原生转弯失败丢失条带的重要问题，新增测试复现失败后修复；边界触角语义差异作为明确限制保留。本次未修改或重建原生库，不启动Gazebo/运动、不push。


以下为 2026-10-27 运行、2026-10-28 收尾的三算法离线基准：

- `python3 -m coverage_benchmark --runtime-root /home/jimmy/Desktop/BG1_Gazebo`：九组实际规划顺序完成，保存九份路径/失败 JSON、三张原图对照图及三指标 CSV。五组非空、四组失败；本次规划器已退出。
- `python3 -m pytest coverage_benchmark/tests -q`：15 项通过，含地图语义、覆盖并集/全部自由分母、障碍/净空/越界、失败长度、STC 保守缩放和真实 BCD 构建/调用/超时/原文件不变。
- `python3 -m coverage_benchmark.verify --runtime-root /home/jimmy/Desktop/BG1_Gazebo`：九组指标/CSV 全量复算、原图/固定版本/21个 Leader 原文件与构建清单/二进制哈希、三张图片解析通过；五条非空路线各 256 个独立点到完整路径距离见证通过。独立审查另完成每条 512 个见证，未发现公共指标、坐标或原生路径保真问题。
- 2026-10-28 收尾复验：`python3 -m pytest coverage_benchmark/tests docs/benchmarks/2026-10-27-A11123AJ04/test_metrics.py docs/benchmarks/2026-10-27-A11123AJ04/test_raster_geometry.py -q`：25 项通过；九组已保存结果再次复算通过。目视核对三张原图叠加图，BCD 内存缺陷单独醒目标记；Markdown 链接/围栏、Python AST、YAML 与空白检查通过。
- BCD 额外诊断：AJ04/AC22 的 ASAN/UBSAN 均报原生 heap-buffer-overflow，C03 GDB 证实空轮廓访问；日志已归档，诊断生成命令已执行核验。本次未重建 ROS 包、未运行无关包测试、未启动 Gazebo/运动或推送远端；BCD 生成副本已实际编译。

以下为 2026-10-27 的开源 BCD/STC 候选研究：

- 联网检索并读取候选官方 README、源码、输入接口、包声明和 LICENSE；分别核查 BCD 的 cell 分解/拼接以及 STC 的四子格/生成树路径机制，记录完整固定 SHA 与来源。没有采用第三方总结或仓库自报覆盖率替代代码和本项目实测。
- 执行 `python3 /tmp/bg1_coverage_candidates_check.py`：候选文档的固定 SHA、官方链接格式、章节/围栏和根目录上下文引用检查通过；`git diff --check` 通过。
- 本轮只新增研究文档并更新当前计划/状态；未修改业务、第三方源码或已有 benchmark 数据，未新增子模块、安装/构建/运行候选、仿真、运动或提交/推送。

以下为 2026-10-27 的 AJ04 PNG 同图 benchmark 验证：

- 执行 `python3 docs/benchmarks/2026-10-27-A11123AJ04/prepare_inputs.py`：507,209 个自由格（含 200 灰度），共同目标 470,743 格；GDAL 多边形有效、93 个环，面积及栅格回填一致，原图/21 个 Leader 源码/固定子模块/可执行文件指纹已记录。
- 分别加载 Leader 与 coverage overlay，执行 `python3 docs/benchmarks/2026-10-27-A11123AJ04/run_planner.py --planner leader`（ROS_DOMAIN_ID=98）和 `--planner f2c`（99），各 4 次真实调用成功且路径指纹一致。两探针关闭各自规划器，退出码均为 0。
- 指标实现前解析用例失败；实现后通过。独立审查核验输入、孔洞和度量后，指出原净空审计的 OpenCV 近似距离误差；独立复现并新增精确距离用例先失败再通过。当前 `python3 -m pytest docs/benchmarks/2026-10-27-A11123AJ04/test_metrics.py docs/benchmarks/2026-10-27-A11123AJ04/test_raster_geometry.py -q`：10 项通过。
- 执行 `python3 docs/benchmarks/2026-10-27-A11123AJ04/analyze_results.py`，再执行 `python3 docs/benchmarks/2026-10-27-A11123AJ04/verify_results.py`：完整路径长度、净空/障碍线段标记、扫掠掩膜、双分母 JSON/CSV、各 512 个独立点到全路径距离见证、输入/版本/源码/二进制指纹、AST/YAML/PNG/链接及进程退出检查通过，保存 `verification.json`。
- 目视检查路径/覆盖对照图；`git diff --check` 通过。没有修改业务及第三方源码，没有重建、运行无关包测试、Gazebo、运动或提交/推送。

以下为 2026-10-27 的 PNG 地图规划验证：

- 补充执行 `python3 leader_coverage_ws/docs/verification/imported_A11123AC22_20261027/analyze_passages.py`：全图自由像素无一在请求框外；完成六处截面宽度及离线几何筛选诊断，保存 JSON、诊断图和复现脚本。执行 `python3 /tmp/bg1_leader_ac22_passage_check.py` 检查选区像素计数、截面宽度、筛选结果、AST/JSON/图片和文档链接，通过；`git diff --check` 通过。未重跑 ROS 规划、重建或运动。

- 执行 `ros2 launch "$PWD/leader_coverage_ws/docs/verification/imported_A11123AC22_20261027/planning.launch.py"`，随后执行 `python3 leader_coverage_ws/docs/verification/imported_A11123AC22_20261027/probe.py`（加载 Jazzy、底层、Leader overlay，`ROS_DOMAIN_ID=97`）。两次规划成功，点数和长度一致，最后一次调用约 0.7367 s。
- `ros2 lifecycle get /map_server` 为 `active [3]`；`ros2 service list -t` 确认 `/leader_coverage/plan` 使用自有服务类型；`ros2 node list` 仅见本次地图、生命周期和 Leader 规划三个节点。
- 探针确认加载地图与指定 PNG 分类逐格一致，21 个原始算法文件哈希匹配；独立线段采样 34,162 点全部在已知自由空间，格子中心距离变换最小净空约 0.9962 m。不是整机净空或覆盖率结论。
- 执行 `python3 /tmp/bg1_leader_ac22_check.py`：原图/源码哈希、结果点数/长度、全路径独立采样、探针和 launch AST、YAML、两张 PNG、Markdown 本地链接检查通过；`git diff --check` 通过。仅新增验证材料及更新计划/状态文档，未重建、重跑包测试或执行运动。
- 对本次 launch 发送 SIGINT，并通过进程检查确认本次地图、生命周期管理器和规划器均已退出；未提交或推送。

以下为 2026-10-26 的提交前验证：

- `git fetch origin` 成功；当前 `main` 比 `origin/main` 多一个既有整理提交，无分叉；`git submodule status` 显示两个依赖仍为固定提交。
- 加载 Jazzy 和底层后执行 `python3 -m pytest ros_ws/src/bg1_description/test ros_ws/src/bg1_gazebo/test ros_ws/src/bg1_navigation/test ros_ws/src/bg1_scenarios/test -q`：37 项通过。
- 加载 Leader overlay 后执行 `colcon --log-base leader_coverage_ws/log test --base-paths leader_coverage_ws/src --build-base leader_coverage_ws/build --install-base leader_coverage_ws/install --event-handlers console_direct+` 及 `colcon test-result --test-result-base leader_coverage_ws/build --verbose`：2 个包测试完成，汇总 23 tests、0 errors、0 failures、0 skipped（含 CTest 包装层）。
- `git check-ignore -v` 确认大型原始记录及验证日志命中规则；Python 检查确认本地原始记录仍为 143,071,781 字节。拟收录的 5 份 JSON、4 份 Python AST、3 份 YAML/RViz 解析通过；Markdown 围栏成对。候选文件合计约 1.74 MiB，无单个超大文件；既有待推送提交最大 blob 不足 1 MiB，凭据模式扫描未发现命中。
- 本轮没有重建、启动 Gazebo 或重跑全路线；功能测试依据当前已有构建，不能据此声称全区覆盖或真机能力。
- 暂存后执行 `git diff --cached --check` 通过；修正 `structure.md` 末尾多余空行，25 个暂存文件的大小、收录边界及 Markdown 本地链接检查通过。
- 已实际执行 `git push origin main`，返回 `could not read Username for 'https://github.com': No such device or address`。随后 `git ls-remote origin refs/heads/main` 确认远端仍为 `190bced4f507069250129dcf6904a3e3de3aa591`，本地比远端多两个提交；未上传成功。

以下为船模迁移及此前任务的历史验证：

- 本次在桌面执行 `python3 -m pytest test -q`：4 项通过；`bash -n run.sh` 通过，模型与世界 `gz sdf -k` 均为 `Valid.`。模型/网格/世界/参数等 8 个资源文件哈希保持一致；工程源码和安装资源中不再有船模。
- 原 `bg1_scenarios` 源码测试 6 项通过，colcon 构建和包测试成功（最终汇总 7 tests、0 errors / 0 failures，含 CTest 包装层）。已清除仅属于移出船模的旧测试缓存/结果，避免结果汇总混入历史船模测试。Gazebo 未重新启动；下面的船模构建/渲染结果为迁移前历史验证，材料已一并移至桌面。

- 2026-10-23 按用户要求向当前船模 Gazebo GUI/server 和启动进程发送 SIGINT，随后通过进程检查确认三者全部退出。模型及验证文件保留。

- 本次船模：底层相关 pytest 41 项通过；`bg1_scenarios` colcon 重建成功，包测试汇总 12 tests、0 errors / 0 failures（含 CTest 包装层）。模型和世界的 `gz sdf -k` 均为 `Valid.`；`gz model --list` 实际确认船模已加载。
- 已实际保存并检查 Gazebo 斜视、侧视、俯视；独立评审发现推进器藏于船壳，新增失败检查后修正位置并重新测试/渲染，现前后两处均可见。脚本语法、资源引用及差异空白检查通过，未提交或推送。
- 旧 Leader 任务本次切换时已是失败/空闲，Nav2 code=105；停车命令报告 idle 后因既有失败状态返回非零。旧 launch、采样和残留 Gazebo 已关闭；未追认旧路线完成，未扩大范围修复历史执行问题。

以下为 2026-10-22 的 Leader 运动验证记录：

- 本次执行 `python3 /tmp/bg1_leader_motion_20261022/prepare.py`：非自由格重建投影断言通过；`gz sdf -k data/maps/imported/A11123AJ04_231127_094529/gazebo_demo/world.sdf` 返回 `Valid.`。
- 本次执行 `ros2 launch /tmp/bg1_leader_motion_20261022/motion.launch.py`、记录探针和 `bash leader_coverage_ws/motion.sh start`：导航接近成功，FollowPath 正在执行。`motion.sh status` 返回 `following`；`ros2 action info /follow_path` 确认一个 controller action server。
- 独立 Gazebo 位姿序列及 TF 检查已保存为 `motion/checked_snapshot.json` 和 `motion/checked_samples.json`，记录 37.775 m 行程和首次掉头，最大 AMCL 位置差 0.157 m；原 PGM 与 21 个第三方源码哈希保持一致。
- 运行探针最初遇到 Gazebo JSON 的 `sec` 为字符串，修正为显式 `float` 后重新采样；早期接近段和最初部分跟踪段未计入行程。未改机器人/算法/公共启动代码，未重建包；本次为运行验证。

- 本次收尾执行保存脚本 AST、RViz YAML、快照/采样点一致性、运动/掉头/AMCL 断言、文档链接和空白检查，均通过；`git diff --check` 无错误。最后 `motion.sh status` 仍为 `following`，实时记录累计实际行程约 54.58 m；快照和图保留在较早的 37.775 m 检查点。未提交或推送。

以下为此前同图规划验证记录：

- 本次执行 `python3 /tmp/bg1_leader_imported_20261022/plan_map.py`（留存为验证目录 `probe.py`）：真实规划服务成功；以半格以内间距加密采样 67,436 个点，全在已知自由空间；欧氏栅格中心最小障碍距离约 0.9962 m。图像已目视核对，不能据此声称全区覆盖或整机可执行。
- 本次执行 `ros2 node list`、`ros2 topic info /map`、`ros2 topic info /leader_coverage/coverage_path`、`ros2 lifecycle get /map_server`：地图为 active，地图与路径发布/订阅存在；没有仿真和运动任务。
- 本次未修改业务代码，未重建或重跑包测试；历史测试结果继续标记为历史证据。

- 本次还执行结果文件一致性检查（JSON 点数/长度、地图 YAML 相对路径、探针 AST、RViz YAML、文本空白）及 `git diff --check`、`git status --short`，均通过；仅新增验证材料并更新上下文文档，未提交或推送。

以下为先前文档初始化验证记录：

- 已执行 `git status --short`、`git log -4 --oneline`、`git submodule status`，核对仓库状态与依赖固定提交。
- 已执行 `cat /etc/os-release`、`test -f /opt/ros/jazzy/setup.bash`、`command -v colcon`、`command -v gz`：Ubuntu 24.04、Jazzy 环境文件及工具入口存在；不等同于运行时健康检查。
- 已执行 `python3 /tmp/bg1_context_docs_check.py`：7 份 Markdown 的主要章节、代码围栏、21 个本地链接、21 个入口/测试路径及空白检查通过；已跟踪文件无改动，初始化说明已删除。检查脚本仅在 `/tmp`，不属于项目交付文件。
- 核验过程中修正了检查脚本对 `git diff --no-index --check` 差异退出码的误判，并移除了操作文档中不存在的 mapping 测试目录；最终重新核验通过。
- 最终执行 `git diff --check`、`git status --short`：无已跟踪文件差异，七份目标文档仍为未跟踪文件；未暂存、提交或推送。
- 未执行：构建、源码测试、GUI/无界面启动和运动测试；本次仅修改文档，不声称历史验证已重新通过。
