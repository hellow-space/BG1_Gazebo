# 当前代码架构

```text
BG1_Gazebo/
├── ros_ws/src/
│   ├── bg1_description/       # YAML 几何/物理、Xacro、外观、footprint
│   ├── bg1_gazebo/            # Gazebo 世界、启动、ROS/Gazebo 桥接
│   ├── bg1_navigation/        # Nav2/RPP、AMCL、demo 世界与地图
│   ├── bg1_mapping/           # SLAM Toolbox、遥控、地图四件套保存
│   └── bg1_scenarios/         # warehouse/office 配置及世界/地图生成器
├── leader_coverage_ws/src/    # bg1_leader_coverage + 自有 interfaces 包
├── coverage_ws/src/           # bg1_coverage_demo
├── third_party/               # leader 原包；opennav、Fields2Cover、BCD、PythonRobotics 子模块
├── coverage_benchmark/        # 独立三算法离线适配、统一指标及原图轨迹
├── data/maps/                 # 本地导入地图与 SLAM 会话
└── docs/                      # 手册及目录整理记录
```

各工作区的 `build/`、`install/`、`log/` 是生成目录。底层为 `/opt/ros/jazzy → ros_ws/install`，两个覆盖 overlay 分别加载在其上，不互相叠加。历史设计和验证证据保留在各工作区 `docs/` 中。

## 运行时数据流

```text
导航：RViz / NavigateToPose → Nav2/NavFn → RPP → velocity_smoother
     → /cmd_vel → ros_gz_bridge → Gazebo 差速底盘
     Gazebo → /odom、/scan、/joint_states、/clock → ROS 2
     静态地图 + /scan + odom TF → AMCL → map → odom

建图：/scan + odom TF → SLAM Toolbox → /map、map → odom
     遥控 → /cmd_vel；保存 → YAML/PGM + posegraph/data

Leader：/map + RViz /clicked_point → region_selector
       → /leader_coverage/plan → 原算法的构建期 ROS 2 适配
       → 选区/轮廓/路径预览
       motion.sh start → NavigateToPose 接近起点 → FollowPath/RPP

Fields2Cover：task.sh → /compute_coverage_path → 规划结果与 RViz 预览
             --execute → 空场接近连接线 + FollowPath/RPP
             Gazebo 真值 + /odom → sim_localization → map → odom
```

## 接口边界

### 关键模块

- 模型权威配置：`ros_ws/src/bg1_description/config/{bg1_geometry,bg1_physics,nav2_footprint}.yaml`。`primitive` / `wrapped` 仅切换外观。
- 桥接配置：`ros_ws/src/bg1_gazebo/config/bridge_topics.yaml`。`/cmd_vel` 为 `geometry_msgs/Twist`，`/odom` 为 `nav_msgs/Odometry`，`/scan` 为 `sensor_msgs/LaserScan`；使用仿真时间。
- 导航 TF 链：`map → odom → base_footprint → base_link → nav_lidar_link`。导航、建图、空场覆盖分别由 AMCL、SLAM Toolbox、仿真定位提供 `map → odom`，不能同时启用。
- `bg1_navigation/config/nav2.yaml` 定义现有 NavFn/RPP 与速度平滑链 `/cmd_vel_nav → /cmd_vel`。office 通过 `bg1_scenarios/config/office_navigation.yaml` 单独覆盖为 Dijkstra；demo/warehouse 保留 A*。
- Leader 自有服务定义：[PlanCoverage.srv](leader_coverage_ws/src/bg1_leader_coverage_interfaces/srv/PlanCoverage.srv)。输入为 map 坐标多边形、起点、模式（0 弓字/1 回字）、米制间距与净空；输出成功状态、说明及 coverage/boundary/island 路径。当前 island 输出禁用，轮廓仅作可视化。
- Leader 控制接口：`/leader_coverage/selection/*` 管理选区，`/leader_coverage/motion/{start,stop,status}` 管理执行；有效路径发布到 `/leader_coverage/coverage_path`，改选区会取消运动。转换工具读取原 C++ 文件，在 `leader_coverage_ws/build/` 生成副本并记录原文件哈希。
- Fields2Cover 使用 `ComputeCoveragePath` action 和现有 `FollowPath`；`config/task.yaml` 定义区域/测试区，`config/coverage.yaml` 定义条带与转弯参数。真值定位仅属于该独立演示。
- 场景 YAML 同时生成世界与 0.05 m 参考地图；本地导入地图不会自动接入 `/map`。数据规则见 [地图说明](data/maps/README.md)。

- 本次导入原图运动演示使用本地 `data/maps/imported/A11123AJ04_231127_094529/gazebo_demo/` 中的挤出网格世界；临时启动配置复用底层仿真/Nav2 和 Leader 运动节点，AMCL 独立提供 `map → odom`。不是新增公共场景入口；复现配置与边界见 [本次运动记录](leader_coverage_ws/docs/verification/imported_A11123AJ04_20261022/motion/README.md)。

### 独立离线 benchmark

`coverage_benchmark` 不属于三个 ROS overlay。`map_io → PlanningInput → adapters/{leader,bcd,stc} → PlanningResult → metrics/runner`，共同输入/输出定义见 [模块说明](coverage_benchmark/README.md)。图像数组为 top-down，路径为左下局部原点的米制 XY。BCD 从只读子模块生成无界面兼容副本，STC 直接导入固定上游；Leader worker 独立加载现有 overlay、ROS domain 101，发布地图并顺序调用现有服务，完成后关闭自身规划器。模块不修复原生路线，不触发机器人运动。

Fields2Cover 地图感知扩展仍位于独立 `coverage_benchmark`：`map_io → f2c_safe/raster_geometry → f2c_worker → swath_geometry → grid_connector → obstacle_connector → metrics`。worker 分别取得原生基线、独立有序条带及可选原生转弯；可选转弯失败时保留原生状态和已成功条带进入 A*，不冒称原生成功。`path_sections` 给所有路径边绑定原生条带/连接方式，`provenance` 绑定实际源文件/输入/依赖/二进制指纹，`verify_f2c` 可从条带完整重放。范围与接口见 [设计](docs/benchmarks/f2c-obstacle-design.md)；原有 ROS 工作区与运动接口不变。


ETHZ适配同属独立离线模块：`map_io → ethz.prepare_input → ethz_native → ETHZ原生BCD/扫描/可见图/GK-MA → PlanningResult → metrics`。先取公共起点净空分量，再按0.2 m保守全自由块取起点分量、矢量化并保留孔洞，舍弃格数写入结果；输入微退1微米避免半开栅格边界歧义，输出整路不修复。原生每cell最佳方向、可见图与GK-MA负责扫描和排序。`ethz_build`在本地生成目录构建依赖和核心，保留上游；`ethz_compare/verify_ethz`将三组新结果与历史12组汇总/复核，原有ROS overlay和运动接口不变。见[模块说明](coverage_benchmark/README.md)。
