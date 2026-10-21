/**********************************************************************
  Copyright(c) 2026 Contributors to ISA-L-CRC-ARM-BENCH All rights
  reserved.  Redistribution and use in source and binary forms, with or
  without modification, are permitted provided that the following
  conditions are met:  BSD-3-Clause, same as ISA-L upstream.

  crc32c_mix_half_ilv -- INTERLEAVED 1:1 PMULL:CRC variant.

  Motivation (all measured on Cobalt 100):
    mix_default (512B PMULL + 3x512B CRC, interleaved) : 31186 MB/s
    mix_n1      (512B PMULL + 1536B CRC, interleaved)  : 31440 MB/s
    mix_half    (1024B PMULL + 1024B CRC, SERIAL)      : 22298 MB/s
  The serial mix_half matches a "NEON-time + CRC-time" additive model
  (166 + 128 cycles per 2048B), proving the OoO window does not span
  the two far-apart loops.  This variant keeps the exact same
  geometry and math as mix_half (two 512B fold segments + K1536/K1024
  advance merge) but interleaves the CRC chain into the fold steps at
  64B granularity, putting both streams in the same instruction
  window.  Model prediction: max(166, 128) = ~170 cycles/block
  = ~37 GB/s (+18% over mix_default).

  Per 2048B block:
    [0,   512)   NEON fold segment 1 (running crc xored in) -> r1
    [512, 1024)  NEON fold segment 2 (from state 0)         -> r2
    [1024,2048)  single CRC32CX chain (from state 0)        -> chain
    merge: crc32cx(0, r1*K1536) ^ crc32cx(0, r2*K1024) ^ chain
  The CRC chain advances in 4x16B bundles after each 64B fold step
  (7 steps x 4 bundles = 448B) plus 4 bundles around each fold tail
  (4 x 16B = 64B) = 512B per segment, 1024B total.

  Prototype: unsigned int crc32c_mix_half_ilv(unsigned char *BUF,
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
	foldc1		.req	v24
	foldc2		.req	v25
	mask		.req	v26

	/* temps live ONLY in caller-saved SIMD regs (v0-v7, v16-v31):
	   v8-v15 are callee-saved per AAPCS and must not be touched */
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
	crc_ptr		.req	x5
	cd0		.req	x6
	cd1		.req	x7

	BUF		.req	x0
	LEN		.req	x1
	wCRC		.req	w2
	CRC		.req	x2

	data0		.req	x4
	data1		.req	x5
	data2		.req	x6
	data3		.req	x7
	wdata		.req	w4

/* 64B pure-NEON fold step (no crc instructions). */
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

/* one 16B CRC bundle: ldp + 2 crc32cx on the serial chain */
.macro	crc_bundle
	ldp		cd0, cd1, [crc_ptr], #16
	crc32cx		wCRC, wCRC, cd0
	crc32cx		wCRC, wCRC, cd1
.endm

/* full 512B fold+tail producing the residue in l0.s[1];
 * identical to mix_half's fold512_tail (proven on silicon). */
.macro	fold512_tail
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
	/* 128 -> 64 (+32 zero append) */
	ext		t1.16b, l0.16b, l0.16b, #8
	pmull2		t1.1q, t1.2d, k3k4.2d
	ext		l0.16b, l0.16b, vzr.16b, #8
	eor		l0.16b, l0.16b, t1.16b
	/* 64 -> 32 */
	ext		t1.16b, l0.16b, vzr.16b, #4
	and		l0.16b, l0.16b, mask.16b
	pmull		l0.1q, l0.1d, k5k0.1d
	eor		l0.16b, l0.16b, t1.16b
	/* Barrett */
	and		t1.16b, l0.16b, mask.16b
	ext		t1.16b, vzr.16b, t1.16b, #8
	pmull2		t1.1q, t1.2d, polyp.2d
	and		t1.16b, t1.16b, mask.16b
	pmull		t1.1q, t1.1d, polyp.1d
	eor		l0.16b, l0.16b, t1.16b
.endm

	.align		6
	.global		crc32c_mix_half_ilv
#ifndef __APPLE__
	.type		crc32c_mix_half_ilv, %function
#endif
crc32c_mix_half_ilv:
	cmp		LEN, 2047
	adr		const_addr, .Lconstants
	bls		.Ltail_entry

	ld1		{k1k2.16b, k3k4.16b, polyp.16b}, [const_addr], #48
	ld1		{k5k0.16b, foldc1.16b}, [const_addr], #32
	ld1		{foldc2.16b, mask.16b}, [const_addr]
	movi		vzr.16b, #0

.Lblock:
	/* ============ segment 1: [0,512) seeded with wCRC ============ */
	ld1		{l0.16b, l1.16b, l2.16b, l3.16b}, [BUF]
	mov		t0.16b, vzr.16b
	fmov		t0s, wCRC
	eor		l0.16b, l0.16b, t0.16b
	add		pmull_ptr, BUF, #64
	add		crc_ptr, BUF, #1024		/* CRC region [1024,2048) */
	mov		wCRC, #0			/* chain state from 0 */
	/* 7 fold steps, each paired with 4 CRC bundles (64B + 64B) */
	.rept	7
	pf_step
	crc_bundle
	crc_bundle
	crc_bundle
	crc_bundle
	.endr
	/* 4 remaining bundles (64B) around the fold tail */
	crc_bundle
	crc_bundle
	fold512_tail
	crc_bundle
	crc_bundle
	mov		w11, l0.s[1]			/* r1 = barrett result */

	/* ============ segment 2: [512,1024) from state 0 ============ */
	add		BUF, BUF, #512
	ld1		{l0.16b, l1.16b, l2.16b, l3.16b}, [BUF]
	add		pmull_ptr, BUF, #64
	.rept	7
	pf_step
	crc_bundle
	crc_bundle
	crc_bundle
	crc_bundle
	.endr
	crc_bundle
	crc_bundle
	fold512_tail
	crc_bundle
	crc_bundle
	mov		w12, l0.s[1]			/* r2 */

	/* ============ merge (identical to mix_half) ============ */
	mov		t0.16b, vzr.16b
	fmov		t0s, w11
	pmull		t0.1q, t0.1d, foldc1.1d
	fmov		x6, t0d
	crc32cx		w6, wzr, x6			/* m1 = adv(r1, 1536) */
	mov		t0.16b, vzr.16b
	fmov		t0s, w12
	pmull		t0.1q, t0.1d, foldc2.1d
	fmov		x7, t0d
	crc32cx		w7, wzr, x7			/* m2 = adv(r2, 1024) */
	eor		wCRC, wCRC, w6
	eor		wCRC, wCRC, w7

	add		BUF, BUF, #1536			/* to block end (base+2048) */
	sub		LEN, LEN, #2048
	cmp		LEN, #2048
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
	/* 16-byte-aligned layout matching the 16-byte ld1 pairs below:
	   [0,16)  k1k2   [16,32) k3k4   [32,48) poly/mu
	   [48,64) k5(128b, hi=0)          [64,80) K1536(128b, hi=0)
	   [80,96) K1024(128b, hi=0)       [96,112) mask(128b: lo32 ones) */
	.octa		0x000000009e4addf800000000740eef02	/* k1k2 */
	.octa		0x000000014cd00bd600000000f20c0dfe	/* k3k4 */
	.octa		0x00000000dea713f10000000105ec76f0	/* poly/mu */
	.octa		0x000000000000000000000000dd45aab8	/* k5 */
	.octa		0x0000000000000000000000009ef68d35	/* K1536 */
	.octa		0x000000000000000000000000170076fa	/* K1024 */
	.octa		0x000000000000000000000000FFFFFFFF	/* mask: low 32 ones */

#ifndef __APPLE__
	.size		crc32c_mix_half_ilv, .-crc32c_mix_half_ilv
#endif
