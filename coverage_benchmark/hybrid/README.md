# IPA BCD / Local Energy

保留纯 `ipa_bcd` 和纯 `ipa_local` 两个离线入口。按用户要求，已移除 `bcd_local`、`hybrid_local`、混合区域分类和 Local 补扫调度。目录名 `hybrid` 保留，以兼容现有报告脚本的导入；IPA 上游全部算法源码和原有 Local Energy 适配不变。

## 运行

依赖既有 benchmark Python 环境、g++、OpenCV4/Eigen3 开发头及 nlohmann/json.hpp。地图来自本地 `data/maps/imported/<场景>/<场景>.png`。

```bash
python3 -m pytest coverage_benchmark/tests/test_hybrid.py coverage_benchmark/tests/test_hybrid_native.py -q
python3 -m coverage_benchmark.hybrid.experiment --output docs/benchmarks/ipa-native-local
python3 -m coverage_benchmark.hybrid.experiment --verify --output docs/benchmarks/ipa-native-local
```

默认原三图、0.05 m、作业宽 0.6 m、公共净空 1 m。`--maps-root` 替换地图根目录。API `experiment.run` 可配置场景及安全参数；已删除补扫轮数参数。默认输出使用新目录，避免覆盖历史四组结果。

## 模块边界

| 文件 | 职责 |
| --- | --- |
| `build.py`、`native/` | 从固定 IPA986c183 生成副本，导出 BCD 分解/扫线及 Local Energy；上游原文件不改 |
| `native.py` | 原生进程限制及输入、输出、日志指纹 |
| `planner.py` | BCD 安全候选、纯 BCD 入口调度、纯 Local 路线、安全连接与覆盖账本；原生 cell 栅格化仅用于查看 |
| `experiment.py` | 两组运行、指标/图片、证据保存和独立重放 |

输入数组为 top-down，进入模块翻转为 lower-left。F 是起点可达净空分量；路线包含起点接近、所有绕行和跨区连接。BCD 在最近 8 个端点候选中按实际安全连接距离贪心选择，不保证全局最优。Local 独立运行，不再补扫或按区域混合切换。覆盖分母仍是原图全部已知自由格。

生成副本关闭额外半格腐蚀，与保留原生额外腐蚀的旧 IPA 适配不同。最终路线通过闭格及独立像素方框相交审计；不证明运动学、实际执行或全区覆盖。模块不启动 ROS/Gazebo，不发送运动指令。

## 溯源

`manifest.json` 绑定源码、编译器、生成副本、二进制、动态库、地图与原生证据 SHA。失败撤销输出目录旧成功证书；原生进程限 180 秒/3 GiB。证据包含本机绝对路径，异机需重新运行。

[原四组设计](../../docs/benchmarks/hybrid-local-bcd-design.md)和[原三图四组结果](../../docs/benchmarks/hybrid-local-bcd-2026-10-30/README.md)仅作为历史记录保留，不代表当前仍提供混合算法入口。历史证书绑定旧源码，不使用当前代码覆盖或重签。
