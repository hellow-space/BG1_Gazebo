# ETHZ全扫描方向模式：资源诊断

2026-10-29。可见图共线顶点归一化后，AJ04在原生`computeAllSweeps`模式生成75个BCD分区，但180秒内未完成候选图，按已声明资源限制终止。本目录保留实际空路径、输入、多边形、日志、未完成manifest与源码快照。

随后AC22已启动，但为切换库自带`SweepPlanGraph::Settings::sweep_single_direction=true`，主动中断该次调用；中断后原生进程已清理。AC22本轮不计成功/失败结果；C03本轮未运行。此轮不是完整三图对比，不用于排名。

最终版在每个BCD cell中只使用`findBestSweepDir`得到的一个原生扫描方案，再按原生可见图和GK-MA连接、排序；保留完整像素边界输入，不做地图简化。见[最终五算法报告](../five-planners-2026-10-29/README.md)。
