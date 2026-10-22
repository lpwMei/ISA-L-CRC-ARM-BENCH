# ISA-L-CRC-ARM-BENCH

ISA-L v2.32.1 `crc32_iscsi`（CRC32C）ARM aarch64 性能优化基准仓库。

上游 README 见 [README-upstream.md](README-upstream.md)。

## 目标

在 GitHub Actions 免费 ARM runner（`ubuntu-24.04-arm`，Cobalt 100，4 vCPU）
上，用 ISA-L 自带的 `crc32_iscsi_perf` 工具对比 **vanilla v2.32.1** 与
**优化版**的 8K 块性能，目标提升 ≥ 10%。

## 优化内容：CRC 3-pipeline 适配（3CRC + PMULL 混合）

ARMv8 的 `CRC32CX` 指令在 Cobalt 100 等核上延迟 3+ 周期。串行 CRC 依赖链
每周期最多 8 字节，需要多条独立流水线才能填满发射端口。

ISA-L v2.32.1 中 `crc32_iscsi` 在通用 ARMv8（CRC32+PMULL 可用）路径上
使用的 `crc32_iscsi_3crc_fold` 是**纯 3 条 CRC 指令流水线**实现。

而源码树中其实还包含一套**从未被 dispatcher 启用**的
`crc32c_mix_default`（2020 年 Arm 贡献，只编译、从未接线）：

- 每 2048B 块拆分为：**512B 走 NEON PMULL 多项式折叠 + 3×512B 走三条
  独立 CRC32CX 流水线**；
- 块尾用 PMULL 常量（advance-by-1536/1024/512 字节）把 NEON 部分与三条
  CRC 部分合并；
- 让 **CRC 执行单元与 NEON 单元并行吞吐**，理论上限从 3×8=24 B/cycle
  提升到 24+8=32 B/cycle（+33%）。

本仓库改动（相对 v2.32.1，见 diff）：

```diff
--- a/crc/aarch64/crc_aarch64_dispatcher.c
+++ b/crc/aarch64/crc_aarch64_dispatcher.c
@@ DEFINE_INTERFACE_DISPATCHER(crc32_iscsi)
         if ((HWCAP_CRC32 | HWCAP_PMULL) == (auxval & (HWCAP_CRC32 | HWCAP_PMULL))) {
-                return crc32_iscsi_3crc_fold;
+                return crc32c_mix_default;   /* 3CRC + PMULL hybrid */
```

## 正确性

1. **位级仿真**（`tools/verify_mix_sim.py`，纯 Python 无依赖）：
   - 以线上已验证的 `3crc_fold` 合并常量为对照组；
   - 验证 mix 的三个合并常量语义（crc32c(R * K_n) == advance n zero bytes）；
   - 完整仿真 2048B 块算法（PMULL 折叠链 + Barrett 归约 + 三链合并），
     2048/4096/6144/8192 字节 × 多种子全部与查表参考一致。
2. **上游单测**：CI 中 `make check` 运行 `crc32_funcs_test`
   （zeros / 简单模式 / 随机种子×随机长度 0..4096 / EOB / update），
   完整覆盖 ≥2048B 的 mix 路径。

## 基准方法（与上游一致）

- 工具：`crc/crc32_iscsi_perf`（8KB buffer，posix_memalign 1024 对齐，
  warm cache 循环，CLOCK_MONOTONIC，默认 3 秒）；
- 同一台 runner 上先后构建两棵树（本仓库优化版 + wget 下载的 vanilla
  v2.32.1），分别运行 perf，解析 `MB/s` 输出并计算提升百分比；
- workflow：`.github/workflows/arm-bench.yml`（push / 手动触发）。

## Round 2 变体横评数据（Cobalt 100，run 39586046753）

`tools/variant_bench`（全长度 0..8448 正确性 ALL PASS + 8K warm 速度）：

| 变体 | 8K 带宽 | 相对基线 |
|---|---|---|
| crc32_iscsi_crc_ext（1 条 CRC 链） | 24709 MB/s | 1.04x |
| crc32_iscsi_3crc_fold（v2.32.1 基线） | 23819 MB/s | 1.00x |
| crc32_iscsi_refl_pmull（纯 NEON） | 18275 MB/s | 0.77x |
| **crc32c_mix_default（3CRC+PMULL，本仓库方案）** | **31183 MB/s** | **1.31x** |
| crc32c_mix_neoverse_n1（1CRC+PMULL） | 31284 MB/s | 1.31x |

关键发现：单 CRC 链混合实现与三链打平（31284 vs 31183），说明在 2048B
块粒度下 CRC 执行端口与 NEON 端口均未饱和；瓶颈更可能在加载端口或
前端取指带宽。1:3 与 3:1 的 CRC:PMULL 配比性能相同，指向对称的
资源约束。

### Round 3/4：交织式 1:1 与单区平衡设计（关键数据，墙已定位）

两个新变体（全部 ALL PASS，与 mix_half 同算法不同调度）：

| 变体 | 设计 | 8K 带宽 | 每块周期 |
|---|---|---|---|
| mix_half | 1:1 串行（两段分离） | 22290 MB/s | ~276（=NEON+CRC 相加，证明乱序窗口不跨段） |
| mix_half_ilv | 1:1 交织（两段，尾链×2） | 28915 MB/s | ~213 |
| **mix_r4** | **单 1024B 折叠区 + 3-bundle 平衡交织（尾链×1）** | **31404 MB/s** | ~196 |
| mix_default | 1:3 交织（512B+3×512B） | 30954 MB/s | ~199 |
| mix_neoverse_n1 | 3:1 交织（512B+1536B） | 31097 MB/s | ~198 |

校准出的 Cobalt 100 微架构参数（由实测反推）：
- `crc32cx`：**延迟 1、吞吐 1/cycle**（crc_ext 单链 24733 MB/s ÷ 8B/cyc）
- NEON `pmull`：吞吐 2/cycle → 0.145 cyc/B
- 折叠尾链：~35 cycles（每区域）

**结论：~31.4 GB/s 是发射端口/加载端口混合墙**。所有交织式设计
（1:3、3:1、1:1 单区）无论配比如何都收敛到 196-213 cyc/2048B；
实测周期约为纯依赖模型（128-170 cyc）的 1.3 倍——差距即多端口
冲突与发射利用率（~62-65%）的现实代价。串行设计因乱序窗口不跨
段而损失 28%。

mix_r4 相对 mix_default 同 run +1.5%（噪声边缘），设计更简洁
（单区域单尾链）但**不足以证明切换 dispatcher 的收益**，主线
保持 `crc32c_mix_default`。

### Round 5：4096B 四链大块（负向结果，正确实现保留）

`crc32c_mix_r5`：4096B 块 = 1024B 折叠区 + 三条 1024B CRC 链交织，
合并用 K1024 状态域复合（数学上证明了 K1024 连用 n 次 = 状态推进
1024n；残差 r 经第一跳转成真状态）。仿真 ALL PASS，硅上全长度
正确性 ALL PASS（run 39707656879）。

**结果：29006 MB/s —— 比 mix_default（31214）慢 7%**。

结论：四链设计把依赖跨度扩到 256 cyc 也没用——**发射端口/
加载端口的每指令吞吐是硬约束**（IPC≈2.68 已饱和）。R4/R5/R3
的对比（31394 / 29006 / 28899）说明：在指令预算固定的情况下，
**更少的链 + 更长的跨度反而更差**，3 链 2048B（mix_default）
与单区 3-bundle（R4）已是调度最优附近。~31.4GB/s 即
0.265 指令/字节的发射墙，**除非减少指令总数（换折叠多项式
结构），任何调度重排都无法突破**。

过程记录（三处修正，全部静态发现）：
1. 残差项需推进 3072（K1024×3）而非 1024（仿真抓出）
2. `adv1024` 宏的 crc32cx 目标必须 w 寄存器
3. 链累加器 w9-w11 与 bundle 加载寄存器 x9-x14 **别名冲突**
   （每次 ldp 先摧毁累加器再消费——教训：多链设计中累加器
   必须与加载寄存器物理隔离）

### Round 6：短折叠+双链最小临界路径（负向结果）

`crc32c_mix_r6`：512B 折叠（95cyc）+ A 链 512B + B 链 1024B（最长
串行 128cyc），理论 dispatch 167cyc/2048B。仿真与硅上正确性 ALL
PASS（run 39712084964），**实测 29880 MB/s，比 R4 低 4.9%**。

原因：每步 4 ab + 4 b bundle（24 条 load+crc/步）的发射密度反超
R4（18 条/步）——**临界路径缩短的收益被发射端口拥挤吃掉**。
R4（单链 3-bundle/步）在"临界路径 × 发射密度"的乘积上仍是实测
最优点。

### Round 7：768B 分裂点（负向，调度空间最终封顶）

`crc32c_mix_r7`：768B 折叠（11 步）+ A 512B + B 768B，用锚点验证的
GF(2) 求解器推导出两个新常量（KR1280=0xdd66cbbb、KS768=0xd7a4825c，
三个已知常量精确复现后才采信）。仿真与硅上正确性 ALL PASS（run
39714018302），**实测 31346 MB/s，仍低于 R4（31533，同 run）**。

修正后模型的分裂点（768B）在真机上不成立——CRC 串行跨度的实际
代价高于模型（含分支/指针开销）。至此调度空间用 **10 个正确变体**
（default/half/half_ilv/r4/r5/r6/r7 + 上游 3 个）彻底封顶，
**R4 为最终最优**（dispatcher 已切换，端到端 +36.9%）。

### dispatcher 最终决策（已确认）

4 次 run 统计（±0.08% 离散）：R4 稳定领先 mix_default +0.53~0.68%，
dispatcher 已切换 → `crc32c_mix_r4`（commit 8fcde25）。

**最终端到端成绩（切换后 3 次统计 run 39712427441/39712439294/
39712451241）：+36.84% / +36.85% / +36.93% —— 稳定 +36.9%**
（`crc32_iscsi_perf` 8K warm，vs vanilla v2.32.1）。

R6 数据（29844-29880 MB/s，负向）进一步确认 R4 是"临界路径 ×
发射密度"乘积的实测最优。调度空间在 R1-R6 共 9 个正确变体后
彻底穷尽；剩余唯一方向为折叠多项式结构重设计（减指令数）。

| 变体 | 设计 | 8K 带宽 | vs 基线 |
|---|---|---|---|
| 3crc_fold（v2.32.1 原版） | 3×512B CRC 链 | 23728 MB/s | 1.00x |
| crc32c_mix_half | 1:1 串行 | 22504 MB/s | 0.95x |
| crc32c_mix_half_ilv | 1:1 交织（双尾链） | 28899 MB/s | 1.22x |
| crc32c_mix_r5 | 4096B 三链大块 | 29006 MB/s | 1.22x |
| **crc32c_mix_default（dispatcher 方案）** | 1:3 交织 | **31214 MB/s** | **1.32x** |
| crc32c_mix_r4 | 单区 1024B + 3-bundle | 31394 MB/s | 1.32x |

**调度维度已系统性穷尽**：配比（1:3/1:1/3:1）× 交织方式（串行/
交织）× 块大小（2048/4096）× 链数（1/2/3/4）共 7 个正确实现实测，
全部收敛于 ~31.4GB/s 发射墙（0.265 指令/字节，IPC≈2.68 饱和）。
唯一未挖方向：更换折叠多项式结构以减少 NEON 侧指令数（当前每
64B 需 16 pmull + 16 eor；理论下限 12+12），需重新推导全套折叠
常量，预期 +8-15%。

### Round 2c 的两次致命 bug 教训（均无需本地 ARM，静态分析 + CI 定位）

1. **常量池错位**（此前 7 轮 CI 失败的真正原因）：`.quad`（8B）布局
   配 16B `ld1` 成对加载，所有合并常量错位 8 字节；Python 仿真用的
   是"意图常量"所以永远查不到——分歧在 .S 的数据布局与加载宽度
   之间，纯静态可查。修复：16B 对齐 `.octa` 布局与 16B ld1 严格对应。
2. **AAPCS 违规**：临时向量寄存器用了 v8-v15（被调用者保存区），
   破坏调用方的 FP 状态——正确性测试碰巧全过但 perf 计时全乱。
   修复：临时寄存器只用 v0-v7/v16-v31。

方法论：**仿真用"意图值"会掩盖布局错位**——常量池必须与加载宽度
一起静态审查；"没硬件修不了"往往是借口。

## Round 1 结果：dispatcher 接线（已合入）

**Cobalt 100（GitHub 免费 ARM runner，`ubuntu-24.04-arm`，CPU part 0xd49），
`crc32_iscsi_perf` 8KB warm-cache，run
[39579661506](https://github.com/lpwMei/ISA-L-CRC-ARM-BENCH/actions/runs/39579661506)：**

| 实现 | 8K 块带宽 |
|---|---|
| baseline v2.32.1（`3crc_fold`） | 26128.96 MB/s |
| **optimized（`crc32c_mix_default`）** | **35007.64 MB/s** |
| **提升** | **+33.98%**（目标 +10%） |

正确性：

- 优化树 `make check` 全部通过（含 `PASS: crc/crc32_funcs_test`，
  随机种子×随机长度 0..4096 / EOB / update 全覆盖 mix 路径）；
- 两个 perf 二进制对同一 8K 缓冲输出的 CRC 完全一致
  （`finish 0xd4854010`），逐位相同。

理论模型（3 CRC 链 24B/cycle + NEON PMULL 8B/cycle = 32B/cycle，
+33%）与实测 +33.98% 吻合。
