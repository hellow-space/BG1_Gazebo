# BCD 原生异常诊断

诊断运行于 2026-10-27，2026-10-28 整理归档。上游 `c90ffa61f4b84c3c09b41c1ba5a837f49d44d278` 未改。所有调试均在生成副本中，输入仍为本轮公共精确净空 PGM，spacing=12、native radius=11；不改变地图、净空或原生规划逻辑。归档日志仅去除行尾空格，保留错误、调用栈及变量内容。

## AJ04：包装层拒绝与原生越界是两条证据

正式基准中，`ConstructCellGraph` 生成的 cell 4 有 14 个 ceiling 点、13 个 floor 点。ceiling 的 x 序列为 `[383,383,384,...,395]`，floor 为 `[383,384,...,395]`，不能假设同一索引总是配对的同一列。包装层在调用原生静态路径前明确拒绝，因此正式结果为失败、无路径。

为了判断这是否仅是过度保守的包装检查，**仅在诊断副本**中把该 guard 改成记录警告后继续，保留原生分解和规划。未经检测的调试构建可以正常退出，但 ASAN/UBSAN 构建明确报 `heap-buffer-overflow`，退出 1：

```text
READ of size 4
GetBoustrophedonPath -> StaticPathPlanning -> main
bcd_asan.cpp:387 -> 原始 main.cpp:381
4 bytes after 2592-byte region (324 Point2D)
```

GDB 捕获 `__asan_report_load4` 时，实际为 **cell 17、i=323、TOPLEFT、radius=11**，读取 `ceiling[324].y`。原生条件在 `&&` 左侧先读取 `ceiling[i+1]`，右侧才检查 `i+1<ceiling.size()`。这次已证实的越界不是 cell 4 不等长直接触发，二者不能混写成同一因果。

证据：[AJ04 ASAN 完整输出](bcd_aj04_asan.txt)、[GDB 调用栈与索引](bcd_aj04_gdb.txt)。正式 wrapper 的 guard 保留，诊断副本输出未用来替代 benchmark。

## AC22：路径几何通过，但原生内存诊断失败

正式 O2 构建返回非空路径，保存轨迹的长度为 525.38 m、覆盖率 12.15%，通过共同栅格几何检查。使用相同图、起点和 spacing 的 ASAN/UBSAN 构建，同样在原始 `main.cpp:381` 检出 heap-buffer-overflow、退出 1：读取位于 5264 字节区域（658 个 Point2D）末端之后 4 字节。

因此表中 `path_valid=true` 只表示**已保存轨迹**满足几何定义，不表示程序没有未定义行为，也不证明再次运行稳定或实现可投入使用。ASAN 错误不构成第四个排名指标，单独记录为实现缺陷。

证据：[AC22 ASAN 完整输出](bcd_ac22_asan.txt)。

## C03：空轮廓被原生代码访问

正式构建在 `contour_extraction` 阶段 signal 11。GDB 定位到 `ExtractRawContours` 原始 `main.cpp:3142`（生成副本 3148），此时 `contours` 和 `wall_cnt_indices` 均为空，仍执行 `wall_cnt_indices.front()`。调用来自 `ExtractContours` 的第二次轮廓提取，保留原生圆内缩及形态学开操作，没有通过减小参数隐藏失败。

证据：[C03 GDB 调用栈与局部变量](bcd_c03_gdb.txt)。这不能推出原图没有物理可通行区域。

## 诊断复现

先按 [模块说明](../../../coverage_benchmark/README.md) 运行完整 benchmark，生成各图 `.work/<场景>/bcd/bcd_navigable.pgm`。以下命令仅生成诊断副本；正式 wrapper 和上游原文件保持原样：

```bash
python3 - <<'PY'
from pathlib import Path
from coverage_benchmark.adapters.bcd_build import build
root = Path.cwd()
executable, _ = build(root)
source = (executable.parent/'bcd_headless.cpp').read_text()
guard = 'std::cerr << "native decomposition returned malformed cell edges\\n"; return 3;'
assert source.count(guard) == 1
# Allow malformed native graph only for diagnosing the upstream call chain.
source = source.replace(guard, 'std::cerr << "BCD_DIAGNOSTIC malformed cell; continuing\\n";')
directory = root/'coverage_benchmark/.build/bcd/diagnostic_repro'
directory.mkdir(parents=True, exist_ok=True)
(directory/'bcd_diagnostic.cpp').write_text(source)
PY
```

随后在项目根目录用系统 g++ 构建，并用 JSON 中的 `requested_start_pixel`（列、行）复现：

```bash
g++ -std=c++14 -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I/usr/include/eigen3 coverage_benchmark/.build/bcd/diagnostic_repro/bcd_diagnostic.cpp \
  -o coverage_benchmark/.build/bcd/diagnostic_repro/bcd_diagnostic \
  $(pkg-config --cflags --libs opencv4)
ASAN_OPTIONS=detect_leaks=0 coverage_benchmark/.build/bcd/diagnostic_repro/bcd_diagnostic \
  coverage_benchmark/.work/A11123AJ04_231127_094529/bcd/bcd_navigable.pgm \
  687 660 12 /tmp/bcd_aj04_diagnostic_path.txt
ASAN_OPTIONS=detect_leaks=0 coverage_benchmark/.build/bcd/diagnostic_repro/bcd_diagnostic \
  coverage_benchmark/.work/A11123AC22_231213_163036/bcd/bcd_navigable.pgm \
  484 921 12 /tmp/bcd_ac22_diagnostic_path.txt
```

日志中的生成文件行号取决于诊断 instrumentation；上游原始行号和函数不变。需要 GDB 时另以 `-O0 -g` 编译同一副本，再对 C03 输入 `421 744 12` 执行 `gdb --batch -ex run -ex bt --args <诊断程序> <PGM> 421 744 12 <输出文件>`。不要将移除 guard 的诊断程序用于正式评估。
