/**********************************************************************
  Copyright(c) 2026 Contributors to ISA-L-CRC-ARM-BENCH All rights
  reserved.  Redistribution and use in source and binary forms, with or
  without modification, are permitted provided that the following
  conditions are met:  BSD-3-Clause, same as ISA-L upstream.

  crc32c_mix_half -- experimental 1:1 rebalanced CRC32C variant.

  Variant-matrix evidence (Cobalt 100, run 39586046753):
    crc32c_mix_default    (512B PMULL + 3x512B CRC) : 31183 MB/s
    crc32c_mix_neoverse_n1 (1536B PMULL + 1x512B CRC): 31284 MB/s
  1:3 and 3:1 splits saturate identically ~31.2 GB/s; this variant
  probes the symmetric 1:1 point:

    per 2048B block:
      [0,   1024)  NEON PMULL fold region (running crc xored into
                   the first 4 bytes), folded down + Barrett-reduced
                   to a 32-bit partial
      [1024,2048)  single CRC32CX chain from state 0
    merge: crc32cx(0, r_pm * K1024) XOR crc_chain
           (K1024 = 0x170076fa advances a state by 1024 zero bytes)

  The complete algorithm (fold chain, constants, merge) is verified
  against table-driven CRC32C by multi-block multi-seed simulation
  (lengths 2048..10240) prior to writing this file.

  Self-contained: does not include crc32_common_mix_neoverse_n1.S.

  Prototype: unsigned int crc32c_mix_half(unsigned char *BUF,
                                          uint64_t LEN,
                                          unsigned int CRC);
**********************************************************************/

	.text
	.align		6
	.arch		armv8-a+crypto+crc

/* ---------------- register names ---------------- */
	k1k2		.req	v20
	k3k4		.req	v21
	polyp		.req	v22
	k5k0		.req	v23
	mask		.req	v24
	fold_poly	.req	v25

	l0		.req	v0
	l1		.req	v1
	l2		.req	v2
	l3		.req	v3
	ldata0		.req	v16
	ldata1		.req	v17
	ldata2		.req	v18
	ldata3		.req	v19
	t0		.req	v4
	t1		.req	v5
	t2		.req	v6
	t3		.req	v7
	t4		.req	v8
	t5		.req	v9
	vzr		.req	v26

	const_addr	.req	x3
	pmull_ptr	.req	x4
	crc_ptr		.req	x5
	cd0		.req	x6
	cd1		.req	x7
	cd2		.req	x9
	cd3		.req	x10
	xPmull		.req	x11

	BUF		.req	x0
	LEN		.req	x1
	wCRC		.req	w2
	CRC		.req	x2

	data0		.req	x4
	data1		.req	x5
	data2		.req	x6
	data3		.req	x7
	wdata		.req	w4

/* pure-NEON fold step: 64B in, no CRC instructions */
.macro	pf_step
	ld1		{ldata0.16b, ldata1.16b, ldata2.16b, ldata3.16b}, [pmull_ptr], #64
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
	eor		l0.16b, l0.16b, ldata0.16b
	eor		l1.16b, l1.16b, ldata1.16b
	eor		l2.16b, l2.16b, ldata2.16b
	eor		l3.16b, l3.16b, ldata3.16b
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
	ld1		{k5k0.8b, mask.8b, fold_poly.8b}, [const_addr]
	movi		vzr.16b, #0

.Lblock:
	/* ---- seed + first 64B of PMULL region ---- */
	ld1		{l0.16b, l1.16b, l2.16b, l3.16b}, [BUF]
	mov		t0.16b, vzr.16b
	fmov		t0.s, wCRC
	eor		l0.16b, l0.16b, t0.16b	/* crc into first 4B */
	add		pmull_ptr, BUF, #64
	add		crc_ptr, BUF, #1024
	mov		wCRC, #0
	sub		LEN, LEN, #2048
	cmp		LEN, #2048

	/* 15 x 64B pure-NEON fold steps: 64 + 15*64 = 1024B */
	.rept	15
	pf_step
	.endr

	/* ---- single CRC chain [1024,2048): 16 x 32B LDP pairs ---- */
	.rept	16
	ldp		cd0, cd1, [crc_ptr], #16
	crc32cx		wCRC, wCRC, cd0
	crc32cx		wCRC, wCRC, cd1
	.endr

	/* ---- fold 4x128b accumulators down (k3k4) ---- */
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

	/* Barrett reduce */
	and		t1.16b, l0.16b, mask.16b
	ext		t1.16b, vzr.16b, t1.16b, #8
	pmull2		t1.1q, t1.2d, polyp.2d
	and		t1.16b, t1.16b, mask.16b
	pmull		t1.1q, t1.1d, polyp.1d
	eor		l0.16b, l0.16b, t1.16b

	/* merge: advance r_pm by 1024 zero bytes then crc32cx */
	mov		t0.16b, vzr.16b
	mov		t0.s[0], l0.s[1]
	pmull		t0.1q, t0.1d, fold_poly.1d
	add		BUF, BUF, #2048
	fmov		xPmull, t0.d
	crc32cx		w11, wzr, xPmull
	eor		wCRC, w11, wCRC
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

.Lconstants:
	.octa		0x000000009e4addf800000000740eef02	/* k1k2 */
	.octa		0x000000014cd00bd600000000f20c0dfe	/* k3k4 */
	.octa		0x00000000dea713f10000000105ec76f0	/* poly(mu) */
	.quad		0x00000000dd45aab8			/* k5 */
	.quad		0x00000000FFFFFFFF			/* mask */
	.quad		0x00000000170076fa			/* K1024 */

#ifndef __APPLE__
	.size		crc32c_mix_half, .-crc32c_mix_half
#endif
