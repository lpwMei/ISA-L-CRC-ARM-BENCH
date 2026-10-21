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

**后续可挖方向（按预期收益排序）**：
1. 减少 NEON 侧 eor 指令数（4 lanes×2 eor/64B 是数学下限，需换
   折叠多项式结构，预期 +5-10%，难度高）
2. 4096B 大块摊薄尾链与块间开销（预期 +2-4%）
3. 预取调优（prfm 指令重排，预期 +1-3%）

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
