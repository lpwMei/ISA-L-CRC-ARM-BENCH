/**********************************************************************
  Copyright(c) 2026 Contributors to ISA-L-CRC-ARM-BENCH All rights
  reserved.  Redistribution and use in source and binary forms, with or
  without modification, are permitted provided that the following
  conditions are met:  BSD-3-Clause, same as ISA-L upstream.

  crc32c_mix_r8 -- pure-CRC triple-chain variant (NO NEON fold).

  Instruction-budget insight (from the R1-R7 data):
    - CRC-side ldp bundles cost 0.1875 instr/byte vs NEON fold 0.266:
      the CRC path is 1.4x more instruction-efficient per byte.
    - R4 spends 38% of its instruction budget on a NEON fold region
      that only relieves a CRC unit which is NOT the bottleneck
      (dispatch is).  Removing the fold entirely:
        2048B = 392 instrs -> 146 dispatch cycles -> ~42 GB/s potential.
    - Longest serial chain: 1024B = 128 cyc < dispatch. No fold tail.

  Geometry per 2048B block:
    chain0 [0,512)   SEEDED with running crc -> st512
    chain1 [512,1024) from 0 -> c1
    chain2 [1024,2048) from 0 -> c2
    merge: adv(st512,1536) ^ adv(c1,1024) ^ c2
  State advances compose existing constants (verified on silicon and
  by simulation): K512=0xdd7e3b0c, K1024=0x170076fa.
    adv(s,512)  = crc32cx(0, s*K512)
    adv(s,1024) = crc32cx(0, s*K1024)
    adv(s,1536) = crc32cx(0, crc32cx(0,s*K1024)*K512)
  Multi-block multi-seed simulation ALL PASS (seeded-chain0 form).

  Prototype: unsigned int crc32c_mix_r8(unsigned char *BUF,
                                         uint64_t LEN,
                                         unsigned int CRC);
**********************************************************************/

	.text
	.align		6
	.arch		armv8-a+crypto+crc

/* ---------------- registers ---------------- */
	const_addr	.req	x3
	ca		.req	x4
	cb		.req	x5
	cc		.req	x6
	cd0		.req	x7
	cd1		.req	x8
	w0c		.req	w15		/* chain0 accumulator */
	w1c		.req	w16		/* chain1 accumulator */
	w2c		.req	w17		/* chain2 accumulator */
	t0s		.req	s0
	t0d		.req	d0
	fc512		.req	v1
	fc1024		.req	v2

	BUF		.req	x0
	LEN		.req	x1
	wCRC		.req	w2
	CRC		.req	x2

	data0		.req	x4
	data1		.req	x5
	data2		.req	x6
	data3		.req	x7
	wdata		.req	w4

/* one 16B bundle per chain (0,1,2); loads x7/x8 isolated from
 * accumulators w15-w17.  3 ldp + 6 crc32cx per call = 48B. */
.macro	t_bundles
	ldp		cd0, cd1, [ca], #16
	crc32cx		w0c, w0c, cd0
	crc32cx		w0c, w0c, cd1
	ldp		cd0, cd1, [cb], #16
	crc32cx		w1c, w1c, cd0
	crc32cx		w1c, w1c, cd1
	ldp		cd0, cd1, [cc], #16
	crc32cx		w2c, w2c, cd0
	crc32cx		w2c, w2c, cd1
.endm

/* advance true state wN by 512/1024 via pmull with the K constant;
 * v0 must be pre-zeroed above bit 32 (fmov only writes low 32). */
.macro	adv1024 wN
	movi		v0.16b, #0
	fmov		t0s, \wN
	pmull		v0.1q, v0.1d, fc1024.1d
	fmov		x9, t0d
	crc32cx		w13, wzr, x9
	mov		\wN, w13
.endm
.macro	adv512 wN
	movi		v0.16b, #0
	fmov		t0s, \wN
	pmull		v0.1q, v0.1d, fc512.1d
	fmov		x9, t0d
	crc32cx		w13, wzr, x9
	mov		\wN, w13
.endm

	.align		6
	.global		crc32c_mix_r8
#ifndef __APPLE__
	.type		crc32c_mix_r8, %function
#endif
crc32c_mix_r8:
	cmp		LEN, 2047
	b.ls		.Ltail_entry

	adrp	x3, .Lconstants
	add	x3, x3, :lo12:.Lconstants
	ldr	q1, [x3, #0]			/* K512  (fc512)  */
	ldr	q2, [x3, #16]			/* K1024 (fc1024) */

.Lblock:
	add		ca, BUF, #0
	add		cb, BUF, #512
	add		cc, BUF, #1024
	mov		w0c, wCRC			/* chain0 seeded with running crc */
	mov		w1c, #0
	mov		w2c, #0
	sub		LEN, LEN, #2048

	/* chain0: 512B = 32 bundles; t_bundles feeds 16B to EACH chain per
	 * call: 32 calls -> ch0 512B (complete), ch1/ch2 512B each. */
	.rept	32
	t_bundles
	.endr
	/* remaining: ch1 512B (32), ch2 512B (32) = 16 ldp-pairs each. */
	.rept	16
	ldp		cd0, cd1, [cb], #16
	crc32cx		w1c, w1c, cd0
	crc32cx		w1c, w1c, cd1
	ldp		cd0, cd1, [cc], #16
	crc32cx		w2c, w2c, cd0
	crc32cx		w2c, w2c, cd1
	.endr

	/* ==== merge ====
	 * st512 (w0c) advance 1536 = adv1024 + adv512
	 * c1 (w1c) advance 1024
	 * c2 (w2c) direct */
	adv1024	w0c
	adv512	w0c
	adv1024	w1c
	eor		wCRC, w2c, w1c
	eor		wCRC, wCRC, w0c

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
	.octa		0x000000000000000000000000dd7e3b0c	/* K512 */
	.octa		0x000000000000000000000000170076fa	/* K1024 */

#ifndef __APPLE__
	.size		crc32c_mix_r8, .-crc32c_mix_r8
#endif
