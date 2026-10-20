# BG1 最小覆盖导航模块设计

状态：用户已确认并要求执行；最小部署及规划、执行、取消、原框架完整性验证已完成，详见 progress.md。

## 目标与约束

部署 open-navigation/opennav_coverage 的 Jazzy 分支，以一个无障碍矩形区域完成“生成覆盖路径 → RViz 显示 → BG1 使用 RPP 跟踪”的最小闭环。

用户要求模块化，不修改现有框架代码。已有 ros_ws 的模型、配置、启动脚本与源码全部只读复用；原来的点到点导航入口继续独立可用。第三方依赖和新代码全部位于同级 coverage_ws。

## 方案选择

采用独立 colcon overlay：先加载 /opt/ros/jazzy，再加载 ros_ws/install，最后加载 coverage_ws/install。相比把第三方包混入 ros_ws，此方案便于单独编译、启动和移除。相比直接使用上游 TurtleBot 演示，此方案验证实际 BG1 模型和当前 RPP。

固定上游分支并记录实际 commit：opennav_coverage 使用 jazzy，Fields2Cover 使用上游明确要求的 v1.2.1-devel，不替换成 v2.x。构建最小依赖闭包：fields2cover、opennav_coverage_msgs、opennav_coverage，以及本地适配包 bg1_coverage_demo。首版不接入 CoverageNavigator 插件、覆盖行为树或 row_coverage。

## 模块边界

- coverage_ws/src：两个上游仓库及 bg1_coverage_demo；不直接修改上游实现。若遇到版本兼容问题，记录原始错误并优先通过受支持的版本或编译选项处理。
- bg1_coverage_demo/config：矩形区域、工程假设的作业宽度、转弯半径与服务端参数。
- bg1_coverage_demo/launch：独立演示启动，包含现有 bg1_gazebo 的 sim.launch.py，启动 coverage_server、controller_server、velocity_smoother、生命周期管理器和 RViz。
- bg1_coverage_demo/scripts：生成路径、发布可视化、按需执行和结果记录。规划与执行分别由显式命令触发，启动场景本身不让机器人移动。
- coverage_ws/docs：部署说明、版本清单和测试记录；新增独立构建及启动脚本。

controller_server 和 velocity_smoother 读取现有 bg1_navigation/config/nav2.yaml，任何演示专用覆盖参数仅在新包中设置。保持 FollowPath、goal_checker、progress_checker 标识兼容，保持 /cmd_vel_nav → velocity_smoother → /cmd_vel 链路。

## 最小场景与坐标系

复用现有无障碍 bg1_flat.sdf；机器人在 (0,0,0) 出生。首版专门验证覆盖算法与跟踪，不启动 AMCL，也不声称完成真实定位验证。原有 AMCL 导航演示不受影响。

运行验证调整（2026-10-19）：原定静态 map → odom 单位变换在约 35.7 m 的覆盖路线后出现 4.63 m 实际终点误差，无法满足验收。因此独立新增 sim_localization.py，用时间对齐的 Gazebo 位姿与轮式 odom 计算 map → odom 校正；不修改底盘或原框架。此仿真专用定位避免累计误差影响覆盖跟踪验证，不代表真实定位能力。

默认区域为 map 坐标下 6 m × 4 m 矩形。预留转弯区域；路径生成后检查完整 BG1 最大包络落在更大的空旷测试区域内。作业宽度初值 0.6 m 是演示假设，不是手册确认的实际刷盘宽度；车体宽度仍使用 0.76 m。

使用固定方向平行条带、往复式排序和仅前进的 Dubins 连接曲线；以保守非零转弯半径生成连接，避免让当前禁用倒车的 RPP 执行反向路径。参数在试运行中依据实际路径与跟踪结果调整，记录最终值。

## 数据流

1. 客户端调用 coverage_server 的 ComputeCoveragePath，传入 map 坐标多边形，启用 headland、route、path。
2. 检查 action 成功、error_code 为零、nav_path 非空、坐标有限、frame 正确，并检查路径及车体包络处于空旷测试区；发布持久可见的区域与路径。
3. 默认只规划。执行命令先使机器人到达路径起点附近，再调用现有 controller_server 的 FollowPath 执行覆盖路径。
4. 首段接近路径只能在此无障碍场景中使用经过边界检查的连接线，并单独执行，避免把远处路径直接交给跟踪器。它不作为真实环境的绕障导航方案。
5. 记录规划时间、路径长度、执行状态、Gazebo 实际轨迹、最终误差；失败、超时或中断时取消 action 并发布零速度。

RPP 通过现有 /scan 和局部代价地图进行碰撞检查；覆盖服务负责区域条带与连接曲线生成，不替代局部感知。

## 验证与交付

- 对新适配逻辑验证非法区域、失败结果、边界越界与规划/执行开关，避免只验证配置文本本身。
- 构建上游最小依赖和本地包，记录依赖、commit 与完整复现命令。
- 先仅规划，确认多条平行作业段与连接路径在 RViz 可见。
- 再在 Gazebo 执行，要求 FollowPath 成功、实际机器人移动且终点误差不超过 0.35 m。记录轨迹，不以轮式 odom 单独证明真实运动。
- 比较部署前后的 ros_ws/src 及现有入口文件内容哈希，确认未被修改；必要时重跑原有静态测试。
- 交付独立构建、启动、仅规划、执行、停止命令。与原导航演示交替运行，避免同时启动两套 Gazebo/controller_server。

## 范围限制

不包含内障碍区域、凹多边形、动态障碍覆盖重规划、整场清洁完成率证明、真实传感器定位或清洁机构控制。上游此版本对转弯始终限制在输入多边形内不作保证，因此首版使用外围空旷测试区并执行包络检查，不把输入区域边界直接当作可碰撞墙体。

## 上游依据

- https://github.com/open-navigation/opennav_coverage/tree/jazzy
- https://github.com/open-navigation/opennav_coverage/blob/jazzy/README.md
- https://github.com/open-navigation/opennav_coverage/blob/jazzy/opennav_coverage_msgs/action/ComputeCoveragePath.action
- https://github.com/open-navigation/opennav_coverage/blob/jazzy/opennav_coverage_demo/params/demo_params.yaml

设计自检：已区分作业宽度与车体宽度、覆盖规划与运动执行、演示坐标变换与真实定位；不修改现有框架文件，不直接套用上游 TurtleBot 参数，不宣称首版覆盖复杂障碍环境。
