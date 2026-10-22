#!/usr/bin/env python3
"""Bit-level simulation of crc32_gzip_refl_mix (R4 geometry, CRC32 reflected poly).

R4 geometry per 2048B block (ported from crc32c_mix_r4):
  [0,1024)    NEON PMULL fold region (seed xored in) -> r
  [1024,2048) single CRC32X chain from state 0       -> chain
  merge: crc32x(0, r*K1024) ^ chain

Control group: crc32_mix_default constants (shipped in tree, known-good for
CRC32 poly) -- both the 512B fold constants and the merge constants.

CRC32 (gzip) semantics: input seed and return value are complemented
(~crc) at the API boundary, as in crc32_gzip_refl_3crc_fold.
"""
import random

POLY = 0xEDB88320  # CRC32 reflected
TAB = []
for i in range(256):
    c = i
    for _ in range(8):
        c = (c >> 1) ^ (POLY if c & 1 else 0)
    TAB.append(c)

def crc32_raw(data, init=0):
    c = init
    for b in data:
        c = (c >> 8) ^ TAB[(c ^ b) & 0xFF]
    return c

def crc32x(S, D):  # ARM CRC32X semantics: 8 bytes of D, little-endian
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

# ---------- constants from crc32_mix_default.S (CRC32 poly, shipped) ----------
K1K2 = (0x01c6e41596 << 64) | 0x0154442bd4      # fold-16 pair
K3K4 = (0x00ccaa009e << 64) | 0x01751997d0      # fold-16 pair
K5   = 0x0163cd6124                              # 64->32 fold const
POLY_B, MU = 0x01db710641, 0x01f7011641         # Barrett pair (poly low word, mu)
# merge consts (crc32_const in crc32_mix_default.S): K1536, K1024, K512
K1536, K1024, K512 = 0x1753ab84, 0xbbf2f6d6, 0x0c30f51d

def fold_step(X, Kq, nxt):
    return (clmul64(X & M64, Kq & M64) ^ clmul64(X >> 64, Kq >> 64) ^ nxt) & M128

def pmull_reduce1024(data, crc):
    """Simulate the NEON fold of buf[0:1024) with init crc xored into first 4 bytes.

    16 steps of 64B (4x16B lanes), same structure as crc32c_mix_r4 pf_step.
    """
    lanes = [le(data[i:i+16]) for i in range(0, 64, 16)]
    lanes[0] ^= crc & MASK32
    off = 64
    for _ in range(15):
        lanes = [fold_step(lanes[j], K1K2, le(data[off+16*j:off+16*j+16])) for j in range(4)]
        off += 64
    # fold_tail: lanes[0..3] -> single 128b -> 64b -> 32b (barrett)
    a = lanes[0]
    for j in (1, 2, 3):
        a = fold_step(a, K3K4, lanes[j])
    # 128 -> 64
    X = clmul64(a & M64, K3K4 >> 64) ^ (a >> 64)
    # 64 -> 32 (keep full width)
    Y = (X >> 32) ^ clmul64(X & MASK32, K5)
    # Barrett
    t = clmul64(Y & MASK32, MU) & MASK32
    t = clmul64(t, POLY_B)
    Z = Y ^ t
    return (Z >> 32) & MASK32

def mix_block2048(data, crc):
    r_pm = pmull_reduce1024(data[0:1024], crc)
    chain = crc32_raw(data[1024:2048], 0)
    m = crc32x(0, clmul64(r_pm, K1024) & M64)
    return m ^ chain

def main():
    rng = random.Random(2026)
    ok = True

    # --- 1. control: shipped crc32_mix_default merge constants ---
    for n, K in ((1536, K1536), (1024, K1024), (512, K512)):
        for _ in range(8):
            R = rng.getrandbits(32)
            target = crc32_raw(b"\x00" * n, R)
            cand = crc32x(0, clmul64(R, K) & M64)
            if target != cand:
                ok = False
                print(f"CONTROL FAIL n={n} K={K:#x} R={R:#x} target={target:#x} cand={cand:#x}")
    print("[control] crc32_mix_default merge constants:", "PASS" if ok else "FAIL")

    # --- 2. full R4-geometry block simulation with complement boundary ---
    ok2 = True
    for L in (2048, 4096, 6144, 8192, 16384):
        for seed in (0, 0x1234, rng.getrandbits(32), rng.getrandbits(32)):
            data = bytes(rng.getrandbits(8) for _ in range(L))
            # API semantics: crc_in/crc_out complemented
            c = seed ^ MASK32
            for off in range(0, L, 2048):
                c = mix_block2048(data[off:off+2048], c)
            c ^= MASK32
            ref = crc32_raw(data, seed ^ MASK32) ^ MASK32
            if c != ref:
                ok2 = False
                print(f"R4 full sim FAIL L={L} seed={seed:#x}: sim={c:#x} ref={ref:#x}")
    print("[R4] crc32_gzip_refl_mix full block sim:", "PASS" if ok2 else "FAIL")

    print("OVERALL:", "ALL PASS" if (ok and ok2) else "FAILURES PRESENT")
    return 0 if (ok and ok2) else 1

if __name__ == "__main__":
    raise SystemExit(main())
