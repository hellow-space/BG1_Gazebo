# Leader 提供的覆盖规划器：ROS 2 独立适配

**来源：** `third_party/leader/coverage_path_planner/` 是用户的 leader 提供的 ROS 1 原包。
本目录是本项目新增的 ROS 2 Jazzy 适配层，与 `coverage_ws/` 中的
`opennav_coverage`/Fields2Cover 是两套不同的实现。

原包保持不变。构建时从原始 C++ 源码生成 ROS 2 副本，使用原来的
`CoveragePathPlanner`、自由格划分、弓字/回字遍历、A* 连接和曲线处理。
适配副本位于 `build/bg1_leader_coverage/generated/`；原始文件 SHA-256
记录在安装资源中的 `source_hashes.json`。原包未提供 `robot_msg`，因此采用
本项目的 `bg1_leader_coverage_interfaces/srv/PlanCoverage`，不冒充原消息接口。

## 构建与启动

在项目根目录执行。先按 `ros_ws/README.md` 构建底层工作区，再运行：

```bash
bash leader_coverage_ws/build.sh
bash leader_coverage_ws/run.sh
```

打开 **warehouse 仓库场景、Gazebo 和 RViz**。选区和规划只显示路径，
需要另行运行 `motion.sh start` 才开始运动。一次只运行一个 BG1 仿真。

先快速看一个预置选区的效果，在另一终端执行：

```bash
bash leader_coverage_ws/region.sh demo
```

预置区域为 `(-10,-8) → (10,-8) → (10,8) → (-10,8)`，包含中央交叉通道
和部分货架。默认弓字形。黄色线表示选区，绿色点是第一个顶点，其余为红点；
青色线是覆盖路径，橙色线是原算法提取的工作轮廓。

## 自己选区

1. RViz 的 Fixed Frame 使用 `map`，点击上方 **Publish Point**。
2. 在地图上沿区域边界顺序点选至少三个顶点，避免边界自交。
3. 最后再点击第一个绿色顶点附近（0.35 m 内），闭合并自动规划。
4. 路径显示后，点击新位置会开始新的选区；旧路径会清除。

顶点不能超出地图。区域可以包含地图中的障碍物；未知和所有非零占据格均视为
障碍物。默认按 1.0 m 圆形净空膨胀，外侧选区也作为不可通行区域处理。
参考地图分辨率为 0.05 m，默认条带间距 0.7 m；米单位参数会换算成栅格数量。

操作命令：

```bash
bash leader_coverage_ws/region.sh reset       # 清空选区和路径
bash leader_coverage_ws/region.sh undo        # 撤销最后一个顶点
bash leader_coverage_ws/region.sh plan        # 当前顶点直接闭合并规划
bash leader_coverage_ws/region.sh zigzag      # 切换弓字形，已有闭合选区时重新规划
bash leader_coverage_ws/region.sh backshaped  # 切换回字形，已有闭合选区时重新规划
bash leader_coverage_ws/stop.sh               # 关闭本次会话
```

命令返回“accepted”表示已提交，最终结果在 RViz 和启动终端的
`Preview ready` / `Planning rejected` 日志中显示。

## 开始运动与停车

选好区域并看到青色覆盖路径后，在项目根目录的新终端运行：

```bash
bash leader_coverage_ws/motion.sh start
```

机器人先通过 Nav2 导航到青色路线起点，再由现有 RPP 控制器跟踪路线；
保留原配置 0.3 m/s 限速。终端会显示 `approaching → following → completed`。
开始后保持该终端运行，**Ctrl+C 会取消任务并等待停车确认**。

也可以在另一个终端控制：

```bash
bash leader_coverage_ws/motion.sh stop     # 停止运动，保留 Gazebo/RViz 和选区
bash leader_coverage_ws/motion.sh status   # 查看阶段与执行结果
bash leader_coverage_ws/stop.sh            # 关闭整个会话
```

重置、撤销顶点、重新规划或改选区会清空/替换路线，正在执行的任务会自动取消；
新路线不会自动开始，需要再次运行 `motion.sh start`。重复 start 会拒绝，
不会替换正在执行的任务。停车命令等待 Nav2 动作终止并持续发送零速度。
如果取消无法确认，节点会尝试停用相关 Nav2 节点并拒绝再次开始，需要重启仿真。

执行入口使用当前有效路线的快照。到达路线起点的 Nav2 接近路径由现有全局规划器
生成，可能经过所选区域以外的已知自由空间；青色覆盖路线本身仍受选区约束。
不要同时发送其他 Nav2 目标或键盘速度命令。

运动验证见 [motion/runtime.json](docs/verification/motion/runtime.json)：
包含弯道的 120 点路线短段完成，独立 Gazebo 位姿测得行程 5.87 m、终点误差
0.206 m。手动停止、修改选区自动取消、Ctrl+C 取消均通过，停车后 2 秒位移为 0。
这不代表已完整执行当前约 86 m 的整条选区路线。

## 当前边界

- 已接入手动执行与取消；已验证短段跟踪，尚未完成长路线全程及覆盖率评估。
- 当前粗栅格间距和保守净空会排除窄通道，可能只覆盖选区中一部分可用区域；
  显示路径不代表全区清扫完成。原算法也可能跳过无法连接的路径组。
- 输出覆盖路径的采样点及连接线段会检查是否进入膨胀后的障碍物或选区外；
  不满足检查时拒绝输出。这是二维栅格检查，不能替代后续整机轨迹执行验证。
- 可视化轮廓仅用于观察，不作为已经验证可执行的沿边路线。
- 首版关闭可选的 island 补扫输出；未接入原系统的设备、禁区和地毯 JSON 文件。
- 地图通过现有 `/map` 获取；`data/maps/imported/` 中的部分 PGM 文件头标注 0.05 m/像素，但没有配套的原点、阈值 YAML，
  当前不会直接作为有物理尺度的演示地图加载。

无界面测试启动：

```bash
bash leader_coverage_ws/run.sh gui:=false headless:=true rviz:=false
```

验证记录和两种模式的路径对比见 [warehouse.png](docs/verification/warehouse.png)、
[warehouse.json](docs/verification/warehouse.json)。这两份规划记录由真实 ROS 服务与
`/clicked_point` 消息测试生成；实际运动测试单独记录在 `motion/` 目录。
