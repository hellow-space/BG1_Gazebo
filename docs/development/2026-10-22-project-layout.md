# 项目目录整理 — 2026-10-22

按用户确认的目录方案整理，并进行本地提交，不推送远端。

- 用户手册：根目录 -> docs/manuals/。
- 导入地图：maps/<原名>.pgm/png -> data/maps/imported/<地图编号>/<原名>.pgm/png。
- SLAM结果：ros_ws/maps/<原名>.* -> data/maps/slam/<会话编号>/<原名>.*。
- Leader原代码：coverage_path_planner/ -> third_party/leader/coverage_path_planner/。
- Git子模块：coverage_ws/src/{opennav_coverage,Fields2Cover} -> third_party/对应目录。
- 自研模型、导航、建图、场景和两种覆盖适配仍在原工作区。ROS包中的配套场景地图继续提交。

53个搬移文件逐一SHA-256检查，内容均保持一致；SLAM YAML相对PGM引用仍可解析。
两个子模块保持原固定提交，工作树干净，未修改其源码。子模块配置节名保留历史标识，路径更新。

修改coverage构建扫描路径、leader构建源位置及CMake默认值、默认地图保存位置和当前使用说明。
历史设计、测试记录中的旧路径保留作为历史，不代表当前入口。
本地导入地图/SLAM会话及build/install/log、个人IDE设置忽略；地图未删除。
本地地图不会随git clone获取，需单独备份。

验证：ros_ws5包、leader2包、coverage4包重新构建成功，三个install目录没有失效链接。
ros_ws45项、leader23项、coverage适配23项colcon测试均通过；coverage21项源码pytest通过。
保存地图入口通过不写文件的命令捕获检查，默认使用data/maps/slam/<会话>/<会话>前缀，显式前缀保持原样。
Leader、mapping、opennav启动参数解析通过；整理后headless实际leader服务两种模式规划和RViz点选消息流程通过。
测试过程不发送机器人运动命令，结束时关闭测试会话。

构建保留了原上游未使用变量/未调用函数缺返回值的警告；Fields2Cover测试结果目录中的故意损坏XML样本会被colcon结果扫描器跳过，不是测试失败。

补充运行检查：迁移后的opennav覆盖演示也以headless模式启动，规划返回成功，7条带、路径35.7077m，execute=false。两个测试会话均已关闭。
项目自有文件的Git空白检查通过；初次纳入版本的leader原文件包含原始行尾空格，为保证源码逐字节一致而保留。
