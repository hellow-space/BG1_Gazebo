# Experiences

- 轮式 odom 或 action 成功不能单独证明底盘实际运动。空场覆盖曾出现 action 成功但实际终点误差 4.63 m；增加独立仿真定位后误差降至约 0.188 m。依据：[覆盖进度与失败证据说明](coverage_ws/docs/progress.md)。
- 本机 SDFormat 的 URDF 转换可能因自定义轮子 collision 名丢失接触参数；现有模型故意省略这些名称，脚轮低摩擦与吸水扒抬高设置有实际转向验证。依据：[模型验证记录](ros_ws/docs/development/progress.md)。
- 本机 Jazzy NavFn A* 在 office 某房间入口出现势场可用但提取失败；仅对 office 改用 Dijkstra 后 20 个规划探针通过，不应扩展为所有场景的默认修改。依据：[大场景验证](ros_ws/docs/development/2026-10-21-large-scenes.md)。
- ROS 异步规划响应应带失效标记和超时；重置选区或新请求后，迟到响应不能覆盖当前状态。取消服务请求只丢弃客户端响应，不停止服务端计算。依据：[Leader 设计](leader_coverage_ws/docs/design.md) 与相关节点测试。
- 迁移源码后须重建受影响 CMake 缓存与 symlink install，并检查失效链接；本项目目录迁移已有重建、哈希和启动检查记录。依据：[目录整理验证](docs/development/2026-10-22-project-layout.md)。

- 本机 `gz topic --json-output` 的时间戳 `sec` 可能为字符串，采样程序应显式转换数值类型后计算时间，避免采集线程退出而运动仍继续。此次导入图运动探针已遇到并修正，证据见 [运动记录](leader_coverage_ws/docs/verification/imported_A11123AJ04_20261022/motion/README.md)。
