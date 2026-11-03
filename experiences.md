# Experiences

- 轮式 odom 或 action 成功不能单独证明底盘实际运动。空场覆盖曾出现 action 成功但实际终点误差 4.63 m；增加独立仿真定位后误差降至约 0.188 m。依据：[覆盖进度与失败证据说明](coverage_ws/docs/progress.md)。
- 本机 SDFormat 的 URDF 转换可能因自定义轮子 collision 名丢失接触参数；现有模型故意省略这些名称，脚轮低摩擦与吸水扒抬高设置有实际转向验证。依据：[模型验证记录](ros_ws/docs/development/progress.md)。
- 本机 Jazzy NavFn A* 在 office 某房间入口出现势场可用但提取失败；仅对 office 改用 Dijkstra 后 20 个规划探针通过，不应扩展为所有场景的默认修改。依据：[大场景验证](ros_ws/docs/development/2026-10-21-large-scenes.md)。
- ROS 异步规划响应应带失效标记和超时；重置选区或新请求后，迟到响应不能覆盖当前状态。取消服务请求只丢弃客户端响应，不停止服务端计算。依据：[Leader 设计](leader_coverage_ws/docs/design.md) 与相关节点测试。
- 迁移源码后须重建受影响 CMake 缓存与 symlink install，并检查失效链接；本项目目录迁移已有重建、哈希和启动检查记录。依据：[目录整理验证](docs/development/2026-10-22-project-layout.md)。

- 本机 `gz topic --json-output` 的时间戳 `sec` 可能为字符串，采样程序应显式转换数值类型后计算时间，避免采集线程退出而运动仍继续。此次导入图运动探针已遇到并修正，证据见 [运动记录](leader_coverage_ws/docs/verification/imported_A11123AJ04_20261022/motion/README.md)。

- 栅格转多边形供CGAL覆盖库使用时，完整像素轮廓可能使候选/可见图计算过慢。本次采用“块内原格全可走才保留”的保守分块后可规划，并仍按原分辨率审计；分块可能切断通道，必须记录舍弃范围，不能只凭更短路径判断更优。依据：[ETHZ五算法对照与原生诊断](docs/benchmarks/five-planners-2026-10-29/README.md)。

- 覆盖算法返回的目标访问序列不能直接视为连续可行路线。IPA Local Energy三图原生直接连线均有穿障段，保留全部目标顺序后补入经审计的起点接近/无切角A*，才通过完整路径检查；不能删坏段再计算覆盖率。依据：[六算法统一对照](docs/benchmarks/six-planners-2026-10-30/README.md)。

- 清扫目标与机器人中心可行区不能混为一张掩膜：F外邻接边带可能仍在F内工具圆盘可扫范围，中心已扫也不等于工具边缘没有收益。历史 Hybrid 补扫实验从原图残余映射可行中心，并按实际新增扫掠验收；正收益仍不保证路径划算，本轮三图新增边带伴随较大绕行。这两个混合变体现已删除，此处保留已验证的经验。依据：[Hybrid同输入四组消融](docs/benchmarks/hybrid-local-bcd-2026-10-30/README.md)。

- 基于固定源码哈希保存的 benchmark 是历史证据。移除变体后，应区分现有入口和历史结果，另设输出目录重新验证，不能用新源码覆盖旧证书或把旧结果标为本次重跑。依据：[当前 IPA 模块边界](coverage_benchmark/hybrid/README.md)与 `status.md` 的两算法验证记录。
