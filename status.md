# Status

## 当前目标

整理 Git 收录范围，并按用户授权提交、推送当前分支（2026-10-26）。

## 已完成

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

Git 文件整理、验证和本地提交已完成；推送因本机缺少 GitHub 认证凭据受阻。船模留在桌面独立目录；本轮未启动 Gazebo 或运动任务。原 Leader 会话已关闭，既有失败状态不代表整条路线完成。

## 当前问题

- `git push origin main` 无法读取 HTTPS 用户名，本机未配置 credential helper，也无可用 SSH agent 密钥。已连接 GitHub 服务可写仓库，但其创建提交接口不能保留现有本地提交元数据；未改写历史代替原样推送。
- 船模仅依据截图比例复刻，缺原始 CAD 和尺寸；遮挡结构简化，不应当作精确船舶模型。

- 部分历史设计/进度含迁移前路径及旧阶段约束；当前入口与后续已批准扩展已在新文档中明确，历史文件原样保留。
- Leader 长路线全程、覆盖率及窄通道能力尚未验证；仿真测试不证明真机定位、动力学或清洁性能。
- 导入地图部分缺少配套 YAML 元数据，本地地图不随 Git 克隆，需要单独备份。
- 本次导入图规划可见部分支路遗漏；尚未量化覆盖率。现已生成二维栅格挤出的平地仿真场景并验证一段运动；柱高和局部原点为演示假设，不代表真实三维场地，未完成全路线/整机净空验收。

## 下一步

- 本机完成 GitHub Git 认证后执行 `git push origin main`，上传两个本地提交，再核对远端与本地一致；无需重新整理或提交文件。
- 船模后续操作在桌面 `Workboat_Gazebo` 完成；BG1 工程按后续任务继续。
- 当前没有仿真进程；船模使用方式见桌面独立目录 README。
- 维护本地地图独立备份；加载导入地图前补齐并确认物理尺度、原点与阈值。

## 验证结果

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
