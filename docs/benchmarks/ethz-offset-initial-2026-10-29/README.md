# ETHZ初次配置诊断：offset_polygons=true

2026-10-29。此目录保留首次实际运行证据；三图都在原生 `calculateDecompositionAdjacency` 返回失败，未生成路线。该配置开启了C++ Settings默认的相邻cell偏置，官方ROS示例则设为false。最终对比使用官方示例的false，见[五算法报告](../five-planners-2026-10-29/README.md)。

| 场景 | 输入顶点 | 分解cell | 原生耗时 | 结果 |
| --- | ---: | ---: | ---: | --- |
| AJ04 | 1834 | 75 | 6.476 s | 邻接检查失败 |
| AC22 | 4082 | 102 | 28.059 s | 邻接检查失败 |
| C03 | 1124 | 59 | 1.776 s | 邻接检查失败 |

原始路径记录（均为空）、输入、多边形及原生日志保留；`source_snapshot/`保存当时关键实现，以当时manifest里的SHA核对。`manifest.json`和CSV是首次完成时的原始记录；本README为后续诊断说明；运行时原页保留为`README.original.md`，其SHA对应原manifest的README项。此处不是最终算法排名，不应单独运行当前版本的verify_ethz复算本目录，当前版本参数已变化。
