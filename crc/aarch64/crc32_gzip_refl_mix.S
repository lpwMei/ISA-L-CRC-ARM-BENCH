/**********************************************************************
  Copyright(c) 2026 Contributors to ISA-L-CRC-ARM-BENCH All rights
  reserved.  Redistribution and use in source and binary forms, with or
  without modification, are permitted under the following
  conditions:  BSD-3-Clause, same as ISA-L upstream.

  crc32_gzip_refl_mix -- CRC32 (reflected, gzip poly) variant of the R4
  single-region balanced-interleaved hybrid.

  Ported from crc32c_mix_r4 (proven geometry on Cobalt 100):
    per 2048B block:
      [0,   1024)  NEON PMULL fold region (running crc xored in) -> r
      [1024,2048)  single CRC32X chain from state 0              -> chain
      merge: crc32x(0, r*K1024) ^ chain
    15 fold steps x (64B NEON + 3 CRC bundles) + fold_tail with 19
    CRC bundles issued around it.

  All fold/barrett constants are the shipped CRC32-poly constants from
  crc32_mix_default.S (k1k2/k3k4/k5k0/poly), merge constant K1024 is
  crc32_const[1] of the same file.  Algorithm verified by bit-level
  simulation: tools/verify_gzip_mix_sim.py (ALL PASS).

  API semantics identical to crc32_gzip_refl_3crc_fold:
    uint32_t crc32_gzip_refl_mix(uint32_t wCRC,
                                 const unsigned char *BUF,
                                 uint64_t LEN);
  (input and output CRC are complemented, i.e. ~ internally.)
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

	/* temps live ONLY in caller-saved SIMD regs (v0-v7, v16-v31) */
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

/* one 16B CRC bundle on the serial chain. */
.macro	crc_bundle
	ldp		cd0, cd1, [crc_ptr], #16
	crc32x		wCRC, wCRC, cd0
	crc32x		wCRC, wCRC, cd1
.endm

/* 128b->32b fold tail (same structure as crc32c_mix_r4 fold_tail,
 * constants swapped to the CRC32 poly pair from crc32_mix_default.S). */
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

	.align		6
	.global		crc32_gzip_refl_mix
#ifndef __APPLE__
	.type		crc32_gzip_refl_mix, %function
#endif
crc32_gzip_refl_mix:
	mvn		wCRC, wCRC
	cmp		LEN, 2047
	adr		const_addr, .Lconstants
	bls		.Ltail_entry

	ld1		{k1k2.16b, k3k4.16b, polyp.16b}, [const_addr], #48
	ld1		{k5k0.16b, foldc.16b}, [const_addr], #32
	ld1		{mask.16b}, [const_addr]
	movi		vzr.16b, #0

.Lblock:
	/* ==== fold region [0,1024): seed + first 64B ==== */
	ld1		{l0.16b, l1.16b, l2.16b, l3.16b}, [BUF]
	mov		t0.16b, vzr.16b
	fmov		t0s, wCRC
	eor		l0.16b, l0.16b, t0.16b
	add		pmull_ptr, BUF, #64
	add		crc_ptr, BUF, #1024
	mov		wCRC, #0
	sub		LEN, LEN, #2048

	/* 15 fold steps x (64B NEON + 3 CRC bundles) = 960B + 720B CRC */
	.rept	15
	pf_step
	crc_bundle
	crc_bundle
	crc_bundle
	.endr

	/* remaining 19 bundles (304B) issued around the tail */
	crc_bundle
	crc_bundle
	crc_bundle
	crc_bundle
	crc_bundle
	crc_bundle
	crc_bundle
	fold_tail
	crc_bundle
	crc_bundle
	crc_bundle
	crc_bundle
	crc_bundle
	crc_bundle
	crc_bundle
	crc_bundle
	crc_bundle
	crc_bundle
	crc_bundle
	crc_bundle
	mov		w11, l0.s[1]			/* r = barrett result */

	/* ==== merge: m = crc32x(0, r*K1024) ==== */
	mov		t0.16b, vzr.16b
	fmov		t0s, w11
	pmull		t0.1q, t0.1d, foldc.1d
	fmov		x6, t0d
	crc32x		w6, wzr, x6
	eor		wCRC, wCRC, w6

	add		BUF, BUF, #2048
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
	crc32x		wCRC, wCRC, data0
	crc32x		wCRC, wCRC, data1
	ldp		data0, data1, [BUF], #16
	crc32x		wCRC, wCRC, data2
	crc32x		wCRC, wCRC, data3
	ldp		data2, data3, [BUF], #16
	crc32x		wCRC, wCRC, data0
	crc32x		wCRC, wCRC, data1
	crc32x		wCRC, wCRC, data2
	crc32x		wCRC, wCRC, data3
	bge		.Lloop_64B
.Lloop_16B:
	cmp		LEN, 15
	bls		.Lless_16B
	ldp		data0, data1, [BUF], #16
	sub		LEN, LEN, #16
	cmp		LEN, 15
	crc32x		wCRC, wCRC, data0
	crc32x		wCRC, wCRC, data1
	bls		.Lless_16B
	ldp		data0, data1, [BUF], #16
	sub		LEN, LEN, #16
	cmp		LEN, 15
	crc32x		wCRC, wCRC, data0
	crc32x		wCRC, wCRC, data1
	bls		.Lless_16B
	ldp		data0, data1, [BUF], #16
	sub		LEN, LEN, #16
	crc32x		wCRC, wCRC, data0
	crc32x		wCRC, wCRC, data1
.Lless_16B:
	cmp		LEN, 7
	bls		.Lless_8B
	ldr		data0, [BUF], #8
	sub		LEN, LEN, #8
	crc32x		wCRC, wCRC, data0
.Lless_8B:
	cmp		LEN, 3
	bls		.Lless_4B
	ldr		wdata, [BUF], #4
	sub		LEN, LEN, #4
	crc32w		wCRC, wCRC, wdata
.Lless_4B:
	cmp		LEN, 1
	bls		.Lless_2B
	ldrh		wdata, [BUF], #2
	sub		LEN, LEN, #2
	crc32h		wCRC, wCRC, wdata
.Lless_2B:
	cbz		LEN, .Lfinish
	ldrb		wdata, [BUF]
	crc32b		wCRC, wCRC, wdata
.Lfinish:
	mvn		w0, wCRC
	ret

	.align		6
.Lconstants:
	/* 16-byte-aligned layout matching the 16-byte ld1 pairs:
	   [0,16) k1k2  [16,32) k3k4  [32,48) poly/mu
	   [48,64) k5   [64,80) K1024  [80,96) mask
	   All values are the shipped CRC32-poly constants from
	   crc32_mix_default.S. */
	.octa		0x00000001c6e415960000000154442bd4		/* k1k2 */
	.octa		0x00000000ccaa009e00000001751997d0		/* k3k4 */
	.octa		0x00000001db71064100000001f7011641		/* poly/mu */
	.octa		0x00000000000000000000000163cd6124		/* k5 */
	.octa		0x000000000000000000000000bbf2f6d6		/* K1024 */
	.octa		0x00000000FFFFFFFF		/* mask */

#ifndef __APPLE__
	.size		crc32_gzip_refl_mix, .-crc32_gzip_refl_mix
#endif
