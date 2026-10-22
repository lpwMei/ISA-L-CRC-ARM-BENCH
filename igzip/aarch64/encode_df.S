/**********************************************************************
  Copyright(c) 2019 Arm Corporation All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted under the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Arm Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING NEGLIGENCE OR
  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**********************************************************************/

#include "../include/aarch64_label.h"

	.arch armv8-a+crc
	.text
	.align	2
#include "lz0a_const_aarch64.h"
#include "data_struct_aarch64.h"
#include "huffman_aarch64.h"
#include "bitbuf2_aarch64.h"
#include "stdmac_aarch64.h"

/*
declare Macros
*/

.macro	declare_generic_reg name:req,reg:req,default:req
	\name		.req	\default\reg
	w_\name		.req	w\reg
	x_\name		.req	x\reg
.endm

	.global	cdecl(encode_deflate_icf_aarch64)
#ifndef __APPLE__
	.type	encode_deflate_icf_aarch64, %function
#endif

/*
	struct deflate_icf *encode_deflate_icf_base(struct deflate_icf *next_in,
				struct deflate_icf *end_in, struct BitBuf2 *bb,
				struct hufftables_icf *hufftables)

	Rewritten: the whole BitBuf2 state (m_bits / m_bit_count / m_out_buf)
	stays in registers for the duration of the loop; per token we issue
	ONE 8-byte store to the output window (same as the C flush_bits),
	instead of the previous 6 stores + 3 reloads through the bb struct.

	Per-token bit budget (all from table entries): ll len (incl. packed
	extra) <= 20, dist len <= 15, dist extra <= 13, P < 8  =>  < 64 bits,
	so a single merged OR-merge into the 64-bit m_bits is always safe.
*/

	// parameters
	declare_generic_reg	next_in,		0,x
	declare_generic_reg	end_in,			1,x
	declare_generic_reg	bb,			2,x
	declare_generic_reg	hufftables,		3,x

	// resident bitbuf state
	declare_generic_reg	bb_out_buf,		9,x
	declare_generic_reg	bb_out_end,		10,x
	declare_generic_reg	bb_bit_count,		11,w
	declare_generic_reg	bb_bits,		12,x

	// per-token temporaries
	declare_generic_reg	code_and_extra,		8,x
	declare_generic_reg	dist_extra,		6,x
	declare_generic_reg	dist_lit_table,		7,x
	declare_generic_reg	code_sum,		4,x
	declare_generic_reg	count_sum,		13,x
	declare_generic_reg	d_length,		14,x
	declare_generic_reg	l_length,		15,x
	declare_generic_reg	d_extra_bit_count,	16,x
	declare_generic_reg	tmp0,			17,x
	declare_generic_reg	tmp1,			5,x

// bit buffer offset
.equ	offset_m_bits, 0
.equ	offset_m_bit_count, 8
.equ	offset_m_out_buf, 16
.equ	offset_m_out_end, 24

cdecl(encode_deflate_icf_aarch64):
	cmp	next_in, end_in
	bcs	.done

	ldp	bb_out_buf, bb_out_end, [bb, offset_m_out_buf]
	cmp	bb_out_end, bb_out_buf
	bcc	.done

	ldr	bb_bit_count, [bb, offset_m_bit_count]
	ldr	bb_bits, [bb, offset_m_bits]
	b	.loop_start

	.align 3
.loop_start:
	// is_full(bb): m_out_buf > m_out_end
	cmp	bb_out_buf, bb_out_end
	bhi	.done

	// ---- load icf token ----
	ldrh	w_code_and_extra, [next_in]
	ldrh	w_dist_extra, [next_in, 2]
	ldr	w_dist_lit_table, [next_in]
	add	next_in, next_in, 4

	and	code_and_extra, code_and_extra, 1023		// lit_len
	ubfx	x_dist_extra, x_dist_extra, 3, 13		// dist_extra
	ubfx	x_dist_lit_table, x_dist_lit_table, 10, 9	// lit_dist

	// ---- look up ll entry: hufftables[(lit_len + 31)] ----
	add	code_and_extra, code_and_extra, 31
	add	x_tmp0, hufftables, code_and_extra, lsl 2
	ldr	w_code_and_extra, [hufftables, code_and_extra, lsl 2]
	ldrb	w_l_length, [x_tmp0, 3]
	and	code_and_extra, code_and_extra, 0xffffff		// ll code+extra

	// ---- look up dist entry ----
	lsl	x_dist_lit_table, x_dist_lit_table, 2
	add	x_d_extra_bit_count, hufftables, x_dist_lit_table
	ldrh	w_code_sum, [hufftables, x_dist_lit_table]
	ldrb	w_d_length, [x_d_extra_bit_count, 3]
	ldrb	w_d_extra_bit_count, [x_d_extra_bit_count, 2]

	// ---- merge: ll_code at 0, dist_code at ll_len, extra at ll_len+d_len ----
	add	w_l_length, w_l_length, bb_bit_count		// P + ll_len
	lsl	x_tmp0, code_and_extra, x_bb_bit_count		// ll_code << P
	add	count_sum, x_d_length, x_l_length		// P+ll_len+d_len
	lsl	x_code_sum, x_code_sum, x_l_length		// dist_code << (P+ll_len)
	orr	x_code_sum, x_code_sum, x_tmp0
	add	w_count_sum, w_d_extra_bit_count, w_count_sum	// + ebc = new count
	lsl	x_dist_extra, x_dist_extra, x_count_sum		// extra << (P+ll_len+d_len)
	orr	x_dist_extra, x_dist_extra, bb_bits		// m_bits |= merged
	orr	x_tmp1, x_code_sum, x_dist_extra

	// ---- flush_bits equivalent: store window, advance, shift ----
	str	x_tmp1, [bb_out_buf]				// store 8B window
	and	w_tmp0, w_count_sum, -8				// whole bits
	lsr	bb_bits, x_tmp1, x_tmp0				// residue (before tmp1 reuse)
	lsr	w_tmp1, w_count_sum, 3				// whole bytes
	add	bb_out_buf, bb_out_buf, x_tmp1, uxtw
	sub	bb_bit_count, w_count_sum, w_tmp0

	cmp	end_in, next_in
	bhi	.loop_start

.done:
	str	bb_bits, [bb, offset_m_bits]
	str	w_bb_bit_count, [bb, offset_m_bit_count]
	str	bb_out_buf, [bb, offset_m_out_buf]
	mov	x0, next_in
	ret
#ifndef __APPLE__
	.size	encode_deflate_icf_aarch64, .-encode_deflate_icf_aarch64
#endif
