# 地图数据

- `imported/<地图编号>/`：导入的 PGM、PNG 原文件。目前10组；部分PGM头标注0.05m/像素，仍缺配套YAML原点等元数据。PNG与同名PGM像素不同，均保留。
- `slam/<会话编号>/`：本项目遥控建图结果，原名的 `.yaml`、`.pgm`、`.posegraph`、`.data` 成组保存。

上述两类为本地数据，默认被根目录 `.gitignore` 忽略。换机器或全新克隆后不会自动获得这些文件，需另行备份/复制。

```bash
# 默认保存到 data/maps/slam/bg1_<时间>/bg1_<时间>.*
bash ros_ws/save_map.sh
# 自选不带后缀的文件前缀
bash ros_ws/save_map.sh data/maps/slam/my_room/my_room
```

`.yaml`+`.pgm` 用于地图加载与导航，`.posegraph`+`.data` 用于恢复 SLAM。四个文件应一起备份；移动整组时保留文件名，使YAML内的相对图像路径继续有效。

仿真参考地图不是本地数据：demo 地图留在 `ros_ws/src/bg1_navigation/maps/`，warehouse/office地图留在 `ros_ws/src/bg1_scenarios/maps/`，与对应世界共同生成并纳入版本管理。

现有导航默认加载场景包自带地图，leader规划器订阅 `/map`。导入文件夹中的地图不会因为放入此处就自动被加载。
