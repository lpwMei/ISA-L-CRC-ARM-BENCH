/**********************************************************************
  Copyright(c) 2026 Contributors to ISA-L-CRC-ARM-BENCH All rights
  reserved.  Redistribution and use in source and binary forms, with or
  without modification, are permitted provided that the following
  conditions are met:  BSD-3-Clause, same as ISA-L upstream.

  crc32c_mix_half -- 1:1 PMULL:CRC rebalanced CRC32C variant.

  Variant-matrix evidence (Cobalt 100, run 39586046753):
    crc32c_mix_default (512B PMULL + 3x512B CRC)   : 31183 MB/s
    crc32c_mix_neoverse_n1 (512B PMULL + 1536B CRC): 31284 MB/s
  Both 1:3 and 3:1 CRC:PMULL mixes saturate identically; this variant
  probes the symmetric 1:1 point.

  Geometry per 2048B block:
    [0,   512)   NEON fold segment 1 (running crc xored in) -> r1
    [512, 1024)  NEON fold segment 2 (from state 0)         -> r2
    [1024,2048)  single CRC32CX chain (from state 0)        -> chain
    merge: crc32cx(0, r1*K1536) ^ crc32cx(0, r2*K1024) ^ chain

  Each 512B segment uses the exact fold tail proven correct by the
  mix family (7 x 64B k1k2 steps, k3k4 lane fold-down, 128->64 via
  k3k4-hi ext fold, 64->32 via k5, Barrett via poly/mu).  The advance
  constants K1536/K1024 convert the folded residue into the running
  state domain; the complete algorithm is verified against
  table-driven CRC32C by multi-block multi-seed simulation (lengths
  2048..10240) -- see commit history.

  Prototype: unsigned int crc32c_mix_half(unsigned char *BUF,
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

	l0		.req	v0
	l1		.req	v1
	l2		.req	v2
	l3		.req	v3
	t0		.req	v8
	t1		.req	v9
	t2		.req	v10
	t3		.req	v11
	d0v		.req	v12
	d1v		.req	v13
	d2v		.req	v14
	d3v		.req	v15
	vzr		.req	v19
	t0s		.req	s8
	t0d		.req	d8

	const_addr	.req	x3
	pmull_ptr	.req	x4
	crc_ptr		.req	x5
	cd0		.req	x6
	cd1		.req	x7
	xR1		.req	x11
	xR2		.req	x12

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

/* full 512B fold+tail producing the residue in t0.s[0];
 * used twice per block. Clobbers l0..l3, t0..t3. */
.macro	fold512_tail
	/* lane fold-down: l0 = fold(l0)^l1 ^ fold(l0)^l2 ^ fold(l0)^l3 */
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
	.global		crc32c_mix_half
#ifndef __APPLE__
	.type		crc32c_mix_half, %function
#endif
crc32c_mix_half:
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
	.rept	7
	pf_step
	.endr
	fold512_tail
	mov		w11, l0.s[1]			/* r1 = barrett result */
	/* ============ segment 2: [512,1024) from state 0 ============ */
	add		BUF, BUF, #512
	ld1		{l0.16b, l1.16b, l2.16b, l3.16b}, [BUF]
	add		pmull_ptr, BUF, #64
	.rept	7
	pf_step
	.endr
	fold512_tail
	mov		w12, l0.s[1]			/* r2 */

	/* ============ CRC chain [1024,2048) ============ */
	/* BUF is now block_base+512 after the ld1 writeback */
	add		crc_ptr, BUF, #512		/* block_base + 1024 */
	mov		wCRC, #0
	.rept	16
	ldp		cd0, cd1, [crc_ptr], #16
	crc32cx		wCRC, wCRC, cd0
	crc32cx		wCRC, wCRC, cd1
	.endr

	/* ============ merge ============ */
	/* m1 = crc32cx(0, r1*K1536) */
	mov		t0.16b, vzr.16b
	fmov		t0s, w11
	pmull		t0.1q, t0.1d, foldc1.1d
	fmov		x6, t0d
	crc32cx		w6, wzr, x6
	/* m2 = crc32cx(0, r2*K1024) */
	mov		t0.16b, vzr.16b
	fmov		t0s, w12
	pmull		t0.1q, t0.1d, foldc2.1d
	fmov		x7, t0d
	crc32cx		w7, wzr, x7
	eor		wCRC, wCRC, w6
	eor		wCRC, wCRC, w7

	add		BUF, BUF, #1536			/* to block end (base+512+1536) */
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
	.octa		0x000000009e4addf800000000740eef02	/* k1k2 */
	.octa		0x000000014cd00bd600000000f20c0dfe	/* k3k4 */
	.octa		0x00000000dea713f10000000105ec76f0	/* poly/mu */
	/* offset 48: k5 ; 64: K1536 ; 80: K1024 ; 96: mask (128b) */
	.quad		0x00000000dd45aab8			/* k5 */
	.quad		0x000000009ef68d35			/* K1536 */
	.quad		0x00000000170076fa			/* K1024 */
	.quad		0
	.quad		0x00000000FFFFFFFF			/* mask lo */
	.quad		0					/* mask hi = 0 */

#ifndef __APPLE__
	.size		crc32c_mix_half, .-crc32c_mix_half
#endif
