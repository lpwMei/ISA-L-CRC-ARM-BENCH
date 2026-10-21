#!/usr/bin/env python3
"""Bit-level simulation of ISA-L crc32c_mix_default (3xCRC + PMULL hybrid) vs table CRC32C.

Validates:
 1. The merge constants (advance-by-n-bytes via clmul + crc32cx)
 2. The full 2048B block algorithm (PMULL fold chain + 3 CRC chains + merge)
Control group: crc32_iscsi_3crc_fold merge constants (shipped, known-good).
"""
import random

POLY = 0x82F63B78  # CRC32C reflected
TAB = []
for i in range(256):
    c = i
    for _ in range(8):
        c = (c >> 1) ^ (POLY if c & 1 else 0)
    TAB.append(c)

def crc32c_raw(data, init=0):
    c = init
    for b in data:
        c = (c >> 8) ^ TAB[(c ^ b) & 0xFF]
    return c

def crc32cx(S, D):  # ARM CRC32CX semantics: 8 bytes of D, little-endian
    c = S
    for b in D.to_bytes(8, 'little'):
        c = (c >> 8) ^ TAB[(c ^ b) & 0xFF]
    return c

def clmul64(a, b):
    r = 0
    while a:
        lsb = a & -a
        r ^= b << (lsb.bit_length() - 1)
        a ^= lsb
    return r

M64 = (1 << 64) - 1
M128 = (1 << 128) - 1
MASK32 = 0xFFFFFFFF

def le(b):
    return int.from_bytes(b, 'little')

# ---------- constants from crc32c_mix_default.S / crc_aarch64_common.h ----------
K1K2 = (0x009e4addf8 << 64) | 0x00740eef02      # fold-16 pair (512B step)
K3K4 = (0x014cd00bd6 << 64) | 0x00f20c0dfe      # fold-16 pair (aggressive)
K5   = 0x00dd45aab8                              # 64->32 fold const (k5k0 low)
POLY_B, MU = 0x0105ec76f0, 0x00dea713f1         # Barrett pair
K1536, K1024, K512 = 0x9ef68d35, 0x170076fa, 0xdd7e3b0c  # merge consts (dist 1536/1024/512 B)
# control: 3crc_fold merge consts (distances 680/344 bytes)
K680, K344 = 0xe417f38a, 0x8f158014

def fold_step(X, Kq, nxt):
    return (clmul64(X & M64, Kq & M64) ^ clmul64(X >> 64, Kq >> 64) ^ nxt) & M128

def pmull_reduce512(data, crc):
    """Simulate the NEON fold of buf[0:512) with init crc xored into first 4 bytes."""
    lanes = [le(data[i:i+16]) for i in range(0, 64, 16)]
    lanes[0] ^= crc & MASK32
    off = 64
    for _ in range(7):
        lanes = [fold_step(lanes[j], K1K2, le(data[off+16*j:off+16*j+16])) for j in range(4)]
        off += 64
    a = lanes[0]
    for j in (1, 2, 3):
        a = fold_step(a, K3K4, lanes[j])
    # 128 -> 64
    X = clmul64(a & M64, K3K4 >> 64) ^ (a >> 64)
    # 64 -> 32  (result may exceed 64 bits; keep full width)
    Y = (X >> 32) ^ clmul64(X & MASK32, K5)
    # Barrett
    t = clmul64(Y & MASK32, MU) & MASK32
    t = clmul64(t, POLY_B)
    Z = Y ^ t
    return (Z >> 32) & MASK32

def mix_block2048(data, crc):
    r_pm = pmull_reduce512(data[0:512], crc)
    c0 = crc32c_raw(data[512:1024], 0)
    c1 = crc32c_raw(data[1024:1536], 0)
    c2 = crc32c_raw(data[1536:2048], 0)
    m0 = crc32cx(0, clmul64(c0, K1024) & M64)
    m1 = crc32cx(0, clmul64(c1, K512) & M64)
    mp = crc32cx(0, clmul64(r_pm, K1536) & M64)
    return m0 ^ m1 ^ c2 ^ mp

def sim_3crc_block1024(data, crc):
    """Control: simulate crc32_iscsi_3crc_fold 1024B block (344/336/344 geometry)."""
    c0 = crc32c_raw(data[0:344], crc)    # chain 0 seeded with running crc
    c1 = crc32c_raw(data[344:680], 0)
    c2 = crc32c_raw(data[680:1024], 0)
    m0 = crc32cx(0, clmul64(c0, K680) & M64)
    m1 = crc32cx(0, clmul64(c1, K344) & M64)
    return m0 ^ m1 ^ c2

def main():
    rng = random.Random(42)
    ok = True

    # --- 1. control: 3crc_fold constants must pass (shipped code) ---
    for n, K in ((680, K680), (344, K344)):
        for _ in range(8):
            R = rng.getrandbits(32)
            target = crc32c_raw(b"\x00" * n, R)          # advance = process n zero bytes
            cand = crc32cx(0, clmul64(R, K) & M64)
            if target != cand:
                ok = False
                print(f"CONTROL FAIL n={n} K={K:#x} R={R:#x} target={target:#x} cand={cand:#x}")
    print("[control] 3crc_fold merge constants:", "PASS" if ok else "FAIL")

    # --- 2. mix merge constants ---
    ok2 = True
    for n, K in ((1536, K1536), (1024, K1024), (512, K512)):
        for _ in range(8):
            R = rng.getrandbits(32)
            target = crc32c_raw(b"\x00" * n, R)
            cand = crc32cx(0, clmul64(R, K) & M64)
            if target != cand:
                ok2 = False
                print(f"MIX CONST FAIL n={n} K={K:#x} R={R:#x} target={target:#x} cand={cand:#x}")
    print("[mix] merge constants:", "PASS" if ok2 else "FAIL")

    # --- 3. full 3crc_fold block simulation (control) ---
    ok3 = True
    for L in (1024, 4096, 8192):
        data = bytes(rng.getrandbits(8) for _ in range(L))
        seed = 0x1234
        c = seed
        for off in range(0, L, 1024):
            c = sim_3crc_block1024(data[off:off+1024], c)
        if c != crc32c_raw(data, seed):
            ok3 = False
            print(f"3crc full sim FAIL at L={L}: sim={c:#x} ref={crc32c_raw(data, seed):#x}")
    print("[control] 3crc_fold full block sim:", "PASS" if ok3 else "FAIL")

    # --- 4. full mix_default block simulation ---
    ok4 = True
    for L in (2048, 4096, 6144, 8192):
        for seed in (0, 0x1234, rng.getrandbits(32)):
            data = bytes(rng.getrandbits(8) for _ in range(L))
            c = seed
            for off in range(0, L, 2048):
                c = mix_block2048(data[off:off+2048], c)
            ref = crc32c_raw(data, seed)
            if c != ref:
                ok4 = False
                print(f"MIX full sim FAIL L={L} seed={seed:#x}: sim={c:#x} ref={ref:#x}")
    print("[mix] mix_default full block sim:", "PASS" if ok4 else "FAIL")

    print("OVERALL:", "ALL PASS" if (ok and ok2 and ok3 and ok4) else "FAILURES PRESENT")

if __name__ == "__main__":
    main()
