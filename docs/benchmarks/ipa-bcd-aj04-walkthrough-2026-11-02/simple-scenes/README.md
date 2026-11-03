# 四个简单场景：IPA BCD 分区图解

阅读 [图解网页](index.html)。四张合成栅格地图直接调用固定版本 IPA 的 `computeCellDecomposition`，使用已有只增观察记录的副本，不经过全局旋转；向下逐行扫描，面积和宽度的合并阈值为 0。图片里的线是分区标记，不代表机器人行驶轨迹。

| 场景 | 观察 | 最终区数 |
| --- | --- | --- |
| 空房间 | 内部始终 1 段；扫出房间后 1→0，检查无切割 | 1 |
| 中央障碍 | IN 时 1→2；OUT 时 2→1；两处均画界 | 4 |
| 双入口汇合 | 1→2 的 IN 未产生界，2→1 的 OUT 产生界 | 3 |
| 侧墙凸起 | 房间内部虽变窄仍是 1 段 | 1 |

每个场景的 `evidence/<场景>/trace.json`、输入、临时边界和标签图都是该小地图的原生执行记录；`evidence/verification.json` 和 `evidence/figures.json` 保留数量及哈希。真实 AJ04 结果与本教学案例分别记录。

在项目根目录复现（需要本报告既有的原生追踪编译上下文和 OpenCV 等开发依赖）：

```bash
python3 docs/benchmarks/ipa-bcd-aj04-walkthrough-2026-11-02/simple-scenes/build_scenes.py
python3 docs/benchmarks/ipa-bcd-aj04-walkthrough-2026-11-02/simple-scenes/render_scenes.py
```

两脚本会更新本目录下的生成图与证据；上游源码、生产规划器及 AJ04 的历史输出不变。
