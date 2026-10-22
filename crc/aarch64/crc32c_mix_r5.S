/**********************************************************************
  Copyright(c) 2026 Contributors to ISA-L-CRC-ARM-BENCH All rights
  reserved.  Redistribution and use in source and binary forms, with or
  without modification, are permitted provided that the following
  conditions are met:  BSD-3-Clause, same as ISA-L upstream.

  crc32c_mix_r5 -- 4096B-block quad-chain interleaved variant.

  Design from the calibrated Cobalt-100 model (see README):
    - dispatch wall ~31.4 GB/s at 2048B blocks comes from the CRC-side
      span (192cyc for 1536B@3 chains) + tail + merge not fitting
    - a 4096B block with FOUR 1024B CRC chains gives a 256-cycle
      dependency span; the NEON side (213cyc) hides entirely inside it
    - predicted ~4096B / 258cyc ~= 47 GB/s (+50% over mix_default)

  Geometry per 4096B block:
    [0,   1024) NEON fold region seeded with running crc -> r (15 steps)
    [1024,2048) CRC chain A from 0 -> cA
    [2048,3072) CRC chain B from 0 -> cB
    [3072,4096) CRC chain C from 0 -> cC
    merge (all advances compose K1024 = 0x170076fa on TRUE states; the
    residue r converts via its first K1024 hop):
      out = cC ^ adv(cB,1024) ^ adv(cA,2048) ^ adv(adv(adv(r,1024),1024),1024)
    Verified by multi-block multi-seed bit-level simulation (R5 ALL PASS).

  Interleave: each 64B fold step is followed by 4 bundles per chain
  (64B per chain per step); 15 steps cover 960B of each chain; the last
  4 bundles per chain are issued around the fold tail. Total per chain:
  64 bundles = 1024B.  Merge = 6 pmull+crc32cx pairs + 3 eor, ~30
  cycles, amortized over 4096B.

  Prototype: unsigned int crc32c_mix_r5(unsigned char *BUF,
                                         uint64_t LEN,
                                         unsigned int CRC);
**********************************************************************/

	.text
	.align		6
	.arch		armv8-a+crypto+crc

/* ---------------- registers ---------------- */
	k1k2		.req	v20
	k3k4		.req	v21
	polyp		.req	v22
	k5k0		.req	v23
	foldc		.req	v24
	mask		.req	v26

	/* temps ONLY in caller-saved SIMD regs (v0-v7, v16-v31) */
	l0		.req	v0
	l1		.req	v1
	l2		.req	v2
	l3		.req	v3
	d0v		.req	v4
	d1v		.req	v5
	d2v		.req	v6
	d3v		.req	v7
	t0		.req	v16
	t1		.req	v17
	t2		.req	v18
	t3		.req	v19
	vzr		.req	v31
	t0s		.req	s16
	t0d		.req	d16

	const_addr	.req	x3
	pmull_ptr	.req	x4
	pA		.req	x5
	pB		.req	x6
	pC		.req	x7
	a0		.req	x9
	a1		.req	x10
	a2		.req	x11
	a3		.req	x12
	a4		.req	x13
	a5		.req	x14

	BUF		.req	x0
	LEN		.req	x1
	wCRC		.req	w2
	CRC		.req	x2
	wA		.req	w9
	wB		.req	w10
	wC		.req	w11

	data0		.req	x4
	data1		.req	x5
	data2		.req	x6
	data3		.req	x7
	wdata		.req	w4

/* 64B pure-NEON fold step. */
.macro	pf_step
	ld1		{d0v.16b, d1v.16b, d2v.16b, d3v.16b}, [pmull_ptr], #64
	pmull		t0.1q, l0.1d, k1k2.1d
	pmull2		t1.1q, l0.2d, k1k2.2d
	pmull		t2.1q, l1.1d, k1k2.1d
	pmull2		t3.1q, l1.2d, k1k2.2d
	eor		l0.16b, t0.16b, t1.16b
	eor		l1.16b, t2.16b, t3.16b
	pmull		t0.1q, l2.1d, k1k2.1d
	pmull2		t1.1q, l2.2d, k1k2.2d
	pmull		t2.1q, l3.1d, k1k2.1d
	pmull2		t3.1q, l3.2d, k1k2.2d
	eor		l2.16b, t0.16b, t1.16b
	eor		l3.16b, t2.16b, t3.16b
	eor		l0.16b, l0.16b, d0v.16b
	eor		l1.16b, l1.16b, d1v.16b
	eor		l2.16b, l2.16b, d2v.16b
	eor		l3.16b, l3.16b, d3v.16b
.endm

/* one 16B bundle per CRC chain (A,B,C). */
.macro	abc_bundles
	ldp		a0, a1, [pA], #16
	ldp		a2, a3, [pB], #16
	ldp		a4, a5, [pC], #16
	crc32cx		wA, wA, a0
	crc32cx		wA, wA, a1
	crc32cx		wB, wB, a2
	crc32cx		wB, wB, a3
	crc32cx		wC, wC, a4
	crc32cx		wC, wC, a5
.endm

/* fold tail (proven). */
.macro	fold_tail
	pmull		t0.1q, l0.1d, k3k4.1d
	pmull2		t1.1q, l0.2d, k3k4.2d
	eor		l0.16b, t0.16b, t1.16b
	eor		l0.16b, l0.16b, l1.16b
	pmull		t0.1q, l0.1d, k3k4.1d
	pmull2		t1.1q, l0.2d, k3k4.2d
	eor		l0.16b, t0.16b, t1.16b
	eor		l0.16b, l0.16b, l2.16b
	pmull		t0.1q, l0.1d, k3k4.1d
	pmull2		t1.1q, l0.2d, k3k4.2d
	eor		l0.16b, t0.16b, t1.16b
	eor		l0.16b, l0.16b, l3.16b
	ext		t1.16b, l0.16b, l0.16b, #8
	pmull2		t1.1q, t1.2d, k3k4.2d
	ext		l0.16b, l0.16b, vzr.16b, #8
	eor		l0.16b, l0.16b, t1.16b
	ext		t1.16b, l0.16b, vzr.16b, #4
	and		l0.16b, l0.16b, mask.16b
	pmull		l0.1q, l0.1d, k5k0.1d
	eor		l0.16b, l0.16b, t1.16b
	and		t1.16b, l0.16b, mask.16b
	ext		t1.16b, vzr.16b, t1.16b, #8
	pmull2		t1.1q, t1.2d, polyp.2d
	and		t1.16b, t1.16b, mask.16b
	pmull		t1.1q, t1.1d, polyp.1d
	eor		l0.16b, l0.16b, t1.16b
.endm

/* advance true state in wN by 1024 via K1024:
 *   t0 = wN * foldc (pmull) ; wN = crc32cx(0, t0d)
 * gL must be a GPR; uses w-alias of gL internally. */
.macro	adv1024 wN, gL
	mov		t0.16b, vzr.16b
	fmov		t0s, \wN
	pmull		t0.1q, t0.1d, foldc.1d
	fmov		\gL, t0d
	crc32cx		w13, wzr, \gL
	mov		\wN, w13
.endm

	.align		6
	.global		crc32c_mix_r5
#ifndef __APPLE__
	.type		crc32c_mix_r5, %function
#endif
crc32c_mix_r5:
	cmp		LEN, 4095
	adr		const_addr, .Lconstants
	bls		.Ltail_entry

	ld1		{k1k2.16b, k3k4.16b, polyp.16b}, [const_addr], #48
	ld1		{k5k0.16b, foldc.16b}, [const_addr], #32
	ld1		{mask.16b}, [const_addr]
	movi		vzr.16b, #0

.Lblock:
	/* ==== fold region [0,1024) seeded ==== */
	ld1		{l0.16b, l1.16b, l2.16b, l3.16b}, [BUF]
	mov		t0.16b, vzr.16b
	fmov		t0s, wCRC
	eor		l0.16b, l0.16b, t0.16b
	add		pmull_ptr, BUF, #64
	add		pA, BUF, #1024
	add		pB, BUF, #2048
	add		pC, BUF, #3072
	mov		wA, #0
	mov		wB, #0
	mov		wC, #0
	sub		LEN, LEN, #4096

	/* 15 steps: each folds 64B NEON + 4 bundles per chain (64B each) */
	.rept	15
	pf_step
	abc_bundles
	abc_bundles
	abc_bundles
	abc_bundles
	.endr

	/* each chain's last 4 bundles (64B) around the tail */
	abc_bundles
	abc_bundles
	fold_tail
	abc_bundles
	abc_bundles
	mov		w12, l0.s[1]			/* r = barrett result */

	/* ==== merge ==== */
	/* m = adv(adv(adv(r,1024),1024),1024): first hop converts residue */
	adv1024	w12, x13
	adv1024	w12, x13
	adv1024	w12, x13
	/* mA = adv(cA, 2048) */
	adv1024	wA, x13
	adv1024	wA, x13
	/* mB = adv(cB, 1024) */
	adv1024	wB, x13
	eor		wCRC, wC, wB
	eor		wCRC, wCRC, wA
	eor		wCRC, wCRC, w12

	add		BUF, BUF, #4096
	cmp		LEN, #4096
	bhs		.Lblock

.Ltail_entry:
	cmp		LEN, 63
	bls		.Lloop_16B
.Lloop_64B:
	ldp		data0, data1, [BUF], #16
	sub		LEN, LEN, #64
	ldp		data2, data3, [BUF], #16
	cmp		LEN, #64
	crc32cx		wCRC, wCRC, data0
	crc32cx		wCRC, wCRC, data1
	ldp		data0, data1, [BUF], #16
	crc32cx		wCRC, wCRC, data2
	crc32cx		wCRC, wCRC, data3
	ldp		data2, data3, [BUF], #16
	crc32cx		wCRC, wCRC, data0
	crc32cx		wCRC, wCRC, data1
	crc32cx		wCRC, wCRC, data2
	crc32cx		wCRC, wCRC, data3
	bge		.Lloop_64B
.Lloop_16B:
	cmp		LEN, 15
	bls		.Lless_16B
	ldp		data0, data1, [BUF], #16
	sub		LEN, LEN, #16
	cmp		LEN, #15
	crc32cx		wCRC, wCRC, data0
	crc32cx		wCRC, wCRC, data1
	bls		.Lless_16B
	ldp		data0, data1, [BUF], #16
	sub		LEN, LEN, #16
	cmp		LEN, #15
	crc32cx		wCRC, wCRC, data0
	crc32cx		wCRC, wCRC, data1
	bls		.Lless_16B
	ldp		data0, data1, [BUF], #16
	sub		LEN, LEN, #16
	crc32cx		wCRC, wCRC, data0
	crc32cx		wCRC, wCRC, data1
.Lless_16B:
	cmp		LEN, 7
	bls		.Lless_8B
	ldr		data0, [BUF], #8
	sub		LEN, LEN, #8
	crc32cx		wCRC, wCRC, data0
.Lless_8B:
	cmp		LEN, 3
	bls		.Lless_4B
	ldr		wdata, [BUF], #4
	sub		LEN, LEN, #4
	crc32cw		wCRC, wCRC, wdata
.Lless_4B:
	cmp		LEN, 1
	bls		.Lless_2B
	ldrh		wdata, [BUF], #2
	sub		LEN, LEN, #2
	crc32ch		wCRC, wCRC, wdata
.Lless_2B:
	cbz		LEN, .Lfinish
	ldrb		wdata, [BUF]
	crc32cb		wCRC, wCRC, wdata
.Lfinish:
	mov		w0, wCRC
	ret

	.align		6
.Lconstants:
	/* 16B-aligned layout matching the 16-byte ld1 pairs:
	   [0,16) k1k2  [16,32) k3k4  [32,48) poly/mu
	   [48,64) k5   [64,80) K1024  [80,96) mask */
	.octa		0x000000009e4addf800000000740eef02	/* k1k2 */
	.octa		0x000000014cd00bd600000000f20c0dfe	/* k3k4 */
	.octa		0x00000000dea713f10000000105ec76f0	/* poly/mu */
	.octa		0x000000000000000000000000dd45aab8	/* k5 */
	.octa		0x000000000000000000000000170076fa	/* K1024 */
	.octa		0x000000000000000000000000FFFFFFFF	/* mask */

#ifndef __APPLE__
	.size		crc32c_mix_r5, .-crc32c_mix_r5
#endif
