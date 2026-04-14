#include "lpc.h"

namespace cst
{
	// Zigzag - Table 8-12
	const int zigzag[16] = {
		0,  1,  4,  8,
		5,  2,  3,  6,
		9, 12, 13, 10,
		7, 11, 14, 15
	};
}

void reorder_zigzag(int16_t *dst, const int16_t *src, int coeff_count)
{
	if (coeff_count == 16)
	{
		for (int i = 0; i < 16; i++)
			dst[i] = src[cst::zigzag[i]];
	}
	else
	{
		for (int i = 0; i < coeff_count; i++)
			dst[i] = src[i];
	}
}

void reorder_linear(int16_t *dst, const int16_t *src, int coeff_count)
{
	if (coeff_count == 16)
	{
		for (int i = 0; i < 16; i++)
			dst[cst::zigzag[i]] = src[i];
	}
	else
	{
		for (int i = 0; i < coeff_count; i++)
			dst[i] = src[i];
	}
}

// Table 9-32 – Specification of ctxBlockCat for the different blocks 
enum
{
	LUMA_DC_BLOCK = 0,
	LUMA_AC_BLOCK = 1,
	LUMA_BLOCK = 2,
	CHROMA_DC_BLOCK = 3,
	CHROMA_AC_BLOCK = 4,
};

namespace cst
{
	// Table 9-32
	const int coeff_per_category[] = { 16, 15, 16, 4, 15 };

	// Table 9-30 
	const int significant_coeff_flag_offset[] = { 0, 15, 29, 44, 47 };
	const int last_significant_coeff_flag_offset[] = { 0, 15, 29, 44, 47 };
	const int coeff_abs_level_minus1_offset[] = { 0, 10, 20, 30, 39 };
}

void encode_mb_type(const predicted_macroblock_t &pred, const neighbour_ctx_t &neighbours, cabac_coder_t *cabac)
{
	const int ctx = CTX_MB_TYPE_I_START;

	int cond_a = !neighbours.top.valid || (*neighbours.top.type == MB_TYPE_4x4) ? 0 : 1;
	int cond_b = !neighbours.left.valid || (*neighbours.left.type == MB_TYPE_4x4) ? 0 : 1;
	int ctx_inc = cond_a + cond_b;

	cabac->encode_bit(pred.type == MB_TYPE_16x16, ctx + ctx_inc);
	if (pred.type == MB_TYPE_16x16)
	{
		LPC_ASSERT(pred.cbp_luma == 0 || pred.cbp_luma == 15);
		LPC_ASSERT(pred.cbp_chroma <= 2);

		cabac->encode_bit(pred.cbp_luma == 15, ctx + 3);

		{
			cabac->encode_bit(pred.cbp_chroma != 0, ctx + 4);
			if (pred.cbp_chroma != 0)
				cabac->encode_bit(pred.cbp_chroma == 2, ctx + 5);
		}

		cabac->encode_bit(uint32_t(pred.mode_luma) & 1, ctx + 6);
		cabac->encode_bit(uint32_t(pred.mode_luma) & 2, ctx + 7);
	}
}

void decode_mb_type(predicted_macroblock_t *pred, const neighbour_ctx_t &neighbours, cabac_coder_t *cabac)
{
	const int ctx = CTX_MB_TYPE_I_START;

	int cond_a = neighbours.top.valid && (*neighbours.top.type != MB_TYPE_4x4) ? 1 : 0;
	int cond_b = neighbours.left.valid && (*neighbours.left.type != MB_TYPE_4x4) ? 1 : 0;
	int ctx_inc = cond_a + cond_b;

	pred->type = cabac->decode_bit(ctx + ctx_inc) ? MB_TYPE_16x16 : MB_TYPE_4x4;
	if (pred->type == MB_TYPE_16x16)
	{
		pred->cbp_luma = cabac->decode_bit(ctx + 3) ? 15 : 0;

		{
			if (cabac->decode_bit(ctx + 4) == 0)
				pred->cbp_chroma = 0;
			else
				pred->cbp_chroma = cabac->decode_bit(ctx + 5) ? 2 : 1;
		}

		int bit1 = cabac->decode_bit(ctx + 6);
		int bit2 = cabac->decode_bit(ctx + 7);
		pred->mode_luma = intra_mode_t(bit2 * 2 + bit1);
	}
}

void encode_luma_mode(intra_mode_t mode, intra_mode_t predicted_mode, cabac_coder_t *cabac)
{
	const int ctx = CTX_PREV_INTRA_PRED_FLAG_START;

	bool match_pred = (mode == predicted_mode);
	cabac->encode_bit(!match_pred, ctx + 0);

	if (!match_pred)
	{
		int mode_int = (mode < predicted_mode) ? mode : mode - 1;

		for (int b = 0; b < 7; b++)
			cabac->encode_bit(mode_int & (1 << b), ctx + 1);
	}
}

void decode_luma_mode(intra_mode_t *mode, intra_mode_t predicted_mode, cabac_coder_t *cabac)
{
	const int ctx = CTX_PREV_INTRA_PRED_FLAG_START;

	bool match_pred = !cabac->decode_bit(ctx + 0);

	if (!match_pred)
	{
		int mode_int = 0;
		for (int b = 0; b < 7; b++)
			mode_int |= (cabac->decode_bit(ctx + 1) << b);

		*mode = (intra_mode_t)(mode_int < predicted_mode ? mode_int : mode_int + 1);
	}
	else
	{
		*mode = predicted_mode;
	}
}

void encode_chroma_mode(intra_mode_t mode, const neighbour_ctx_t &neighbours, cabac_coder_t *cabac)
{
	const int ctx = CTX_INTRA_CHROMA_PRED_START;

	int cond_a = neighbours.top.valid && (*neighbours.top.mode_chroma != INTRA_DC) ? 1 : 0;
	int cond_b = neighbours.left.valid && (*neighbours.left.mode_chroma != INTRA_DC) ? 1 : 0;
	int ctx_inc = cond_a + cond_b;

	// Truncated unary
	cabac->encode_bit(mode != 0, ctx + ctx_inc);
	if (mode != 0)
	{
		ctx_inc = 3;
		for (int i = 1; i < mode; i++)
			cabac->encode_bit(1, ctx + ctx_inc);
		cabac->encode_bit(0, ctx + ctx_inc);
	}
}

void decode_chroma_mode(intra_mode_t *mode, const neighbour_ctx_t &neighbours, cabac_coder_t *cabac)
{
	const int ctx = CTX_INTRA_CHROMA_PRED_START;

	int cond_a = neighbours.top.valid && (*neighbours.top.mode_chroma != INTRA_DC) ? 1 : 0;
	int cond_b = neighbours.left.valid && (*neighbours.left.mode_chroma != INTRA_DC) ? 1 : 0;
	int ctx_inc = cond_a + cond_b;

	// Truncated unary
	int mode_int = 0;
	if (cabac->decode_bit(ctx + ctx_inc) != 0)
	{
		mode_int = 1;
		ctx_inc = 3;
		while (cabac->decode_bit(ctx + ctx_inc) && mode_int < INTRA_MODE_COUNT)
			mode_int++;
	}

	*mode = (intra_mode_t)mode_int;
}

void encode_qp_delta(int8_t qp_delta, const neighbour_ctx_t &neighbours, cabac_coder_t *cabac)
{
	const int ctx = CTX_MB_QP_DELTA_START;

	int ctx_inc = neighbours.prev_qp_delta != 0 ? 1 : 0;
	cabac->encode_bit(qp_delta != 0, ctx + ctx_inc);

	if (qp_delta != 0)
	{
		int coded_value = (qp_delta > 0) ? qp_delta * 2 - 1 : -qp_delta * 2;

		ctx_inc = 2;
		cabac->encode_bit(coded_value != 1, ctx + ctx_inc);
		if (coded_value != 1)
		{
			coded_value -= 2;

			ctx_inc = 3;
			while (coded_value-- > 0)
				cabac->encode_bit(1, ctx + ctx_inc);
			cabac->encode_bit(0, ctx + ctx_inc);
		}
	}
}

void decode_qp_delta(int8_t *qp_delta, const neighbour_ctx_t &neighbours, cabac_coder_t *cabac)
{
	const int ctx = CTX_MB_QP_DELTA_START;

	int ctx_inc = neighbours.prev_qp_delta != 0 ? 1 : 0;
	if (cabac->decode_bit(ctx + ctx_inc) == 0)
	{
		*qp_delta = 0;
	}
	else
	{
		int coded_value = 1;

		ctx_inc = 2;
		if (cabac->decode_bit(ctx + ctx_inc) != 0)
		{
			coded_value++;

			ctx_inc = 3;
			while (cabac->decode_bit(ctx + ctx_inc) != 0 && coded_value < (26 * 2))
				coded_value++;
		}

		if (coded_value & 1)
			*qp_delta = (coded_value >> 1) + 1;
		else
			*qp_delta = -(coded_value >> 1);
	}
}

void encode_coeff_abs(int coeff_abs, int block_category, int numDecodAbsLevelEq1, int numDecodAbsLevelGt1, cabac_coder_t *cabac)
{
	const int ctx = CTX_COEFF_ABS_LEVEL_START + cst::coeff_abs_level_minus1_offset[block_category];

	int ctx_inc = 0;
	if (numDecodAbsLevelGt1 == 0)
		ctx_inc = min(4, 1 + numDecodAbsLevelEq1);

	int coeff = coeff_abs;
	int prefix = 0;
	while (coeff-- > 0 && prefix++ < 14)
	{
		cabac->encode_bit(coeff > 0, ctx + ctx_inc);

		ctx_inc = 5 + min(4, numDecodAbsLevelGt1);
	}

	if (prefix > 14)
	{
		int k = 0;
		while (coeff >= (1 << k))
		{
			cabac->encode_bypass(true);
			coeff -= (1 << k);
			k++;
		}
		cabac->encode_bypass(false);

		while (k--)
			cabac->encode_bypass((coeff >> k) & 1);
	}
}

int decode_coeff_abs(int block_category, int numDecodAbsLevelEq1, int numDecodAbsLevelGt1, cabac_coder_t *cabac)
{
	const int ctx = CTX_COEFF_ABS_LEVEL_START + cst::coeff_abs_level_minus1_offset[block_category];

	int ctx_inc = 0;
	if (numDecodAbsLevelGt1 == 0)
		ctx_inc = min(4, 1 + numDecodAbsLevelEq1);

	int prefix = 0;
	while (prefix < 14)
	{
		if (cabac->decode_bit(ctx + ctx_inc) == 0)
			break;
		prefix++;

		ctx_inc = 5 + min(4, numDecodAbsLevelGt1);
	}

	int coef = prefix;
	if (prefix >= 14)
	{
		int suffix = 0;
		int k = 0;

		while (cabac->decode_bypass() && k < 32)
			k++;

		for (int i = 0; i < k; i++)
			suffix = (suffix << 1) | int(cabac->decode_bypass());

		suffix += (1 << k) - 1;

		coef = 14 + suffix;
	}

	return coef + 1;
}

void encode_residual_block(int block_category, const int16_t *residuals, int num_residuals, cabac_coder_t *cabac)
{
	const int max_coeffs = cst::coeff_per_category[block_category];
	const int offset = num_residuals - max_coeffs;
	int ctx_inc;

	int16_t resid_scan[16];
	reorder_zigzag(resid_scan, residuals, num_residuals);

	// Find last non zero coefficient
	int coeff_count = 0;
	for (int i = max_coeffs - 1; i >= 0; i--)
	{
		int16_t value = resid_scan[offset + i];
		if (value != 0)
		{
			coeff_count = i + 1;
			break;
		}
	}

	if (coeff_count == 0)
	{
		for (int i = 0; i < max_coeffs; i++)
		{
			ctx_inc = offset + i + cst::significant_coeff_flag_offset[block_category];
			cabac->encode_bit(false, CTX_SIG_COEFF_FLAG_START + ctx_inc);
		}
		return;
	}

	for (int i = 0; i < coeff_count; i++)
	{
		int16_t value = resid_scan[offset + i];
		bool is_significant = (value != 0);
		bool is_last = (i == coeff_count - 1);

		ctx_inc = offset + i + cst::significant_coeff_flag_offset[block_category];
		cabac->encode_bit(is_significant, CTX_SIG_COEFF_FLAG_START + ctx_inc);
		if (!is_significant)
			continue;

		ctx_inc = offset + i + cst::last_significant_coeff_flag_offset[block_category];
		cabac->encode_bit(is_last, CTX_LAST_SIG_COEFF_START + ctx_inc);
		if (is_last)
			break;
	}

	int numDecodAbsLevelEq1 = 0;
	int numDecodAbsLevelGt1 = 0;

	for (int i = coeff_count - 1; i >= 0; i--)
	{
		int16_t value = resid_scan[offset + i];
		if (value == 0)
			continue;

		int coeff_abs = value > 0 ? value : -value;
		bool coeff_sign = value > 0 ? false : true;

		encode_coeff_abs(coeff_abs, block_category, numDecodAbsLevelEq1, numDecodAbsLevelGt1, cabac);
		cabac->encode_bypass(coeff_sign);

		if (coeff_abs == 1)
			numDecodAbsLevelEq1++;
		else
			numDecodAbsLevelGt1++;
	}
}

void decode_residual_block(int block_category, int16_t *residuals, int num_residuals, cabac_coder_t *cabac)
{
	const int max_coeffs = cst::coeff_per_category[block_category];
	const int offset = num_residuals - max_coeffs;
	int coeff_count = max_coeffs;
	int ctx_inc;

	int16_t resid_scan[16];

	bool significant_coeff[16];
	for (int i = 0; i < max_coeffs; i++)
	{
		ctx_inc = offset + i + cst::significant_coeff_flag_offset[block_category];
		significant_coeff[i] = cabac->decode_bit(CTX_SIG_COEFF_FLAG_START + ctx_inc);
		if (!significant_coeff[i])
			continue;

		ctx_inc = offset + i + cst::last_significant_coeff_flag_offset[block_category];
		if (cabac->decode_bit(CTX_LAST_SIG_COEFF_START + ctx_inc))
		{
			coeff_count = i + 1;
			for (int j = coeff_count; j < max_coeffs; j++)
				resid_scan[offset + j] = 0;
			break;
		}
	}

	int numDecodAbsLevelEq1 = 0;
	int numDecodAbsLevelGt1 = 0;

	for (int i = coeff_count - 1; i >= 0; i--)
	{
		if (!significant_coeff[i])
		{
			resid_scan[offset + i] = 0;
			continue;
		}

		int coeff_abs = decode_coeff_abs(block_category, numDecodAbsLevelEq1, numDecodAbsLevelGt1, cabac);
		int coeff_sign = cabac->decode_bypass() ? -1 : 1;
		resid_scan[offset + i] = coeff_sign * coeff_abs;

		if (coeff_abs == 1)
			numDecodAbsLevelEq1++;
		else
			numDecodAbsLevelGt1++;
	}

	reorder_linear(residuals, resid_scan, num_residuals);
}

void predicted_macroblock_t::compute_cbp_flags(const mb_residuals_t &residuals)
{
	if (type == MB_TYPE_4x4)
	{
		cbp_chroma = 2;
	}
	else
	{
		cbp_luma = 0;
		cbp_chroma = 0;

		for (int b = 0; b < 16; b++)
		{
			for (int i = 1; i < 16; i++)
			{
				if (residuals.luma[b].val[i] != 0)
				{
					cbp_luma = 15;
					goto end_luma;
				}
			}
		}

		end_luma:

		for (int plane = 0; plane < 2; plane++)
		{
			for (int b = 0; b < 4; b++)
			{
				for (int i = 1; i < 16; i++)
				{
					if (residuals.chroma_ac[plane][b].val[i] != 0)
					{
						cbp_chroma = 2;
						goto end_chroma;
					}
				}
			}

			for (int i = 0; i < 4; i++)
			{
				if (residuals.chroma_dc[plane].val[i] != 0)
				{
					cbp_chroma = 1;
					goto end_chroma;
				}
			}
		}

		end_chroma: ;
	}
}

void predicted_macroblock_t::encode_mb(const neighbour_ctx_t &neighbours, const mb_residuals_t &residuals,
		cabac_coder_t *cabac) const
{
	encode_mb_type(*this, neighbours, cabac);

	/// Modes

	// Luma
	if (type == MB_TYPE_4x4)
	{
		for (int i = 0; i < LUMA_BLOCK_COUNT; i++)
		for (int j = 0; j < LUMA_BLOCK_COUNT; j++)
		{
			int mode_top = neighbours.top.valid ? neighbours.top.modes_luma[i] : INTRA_MODE_COUNT;
			int mode_left = neighbours.left.valid ? neighbours.left.modes_luma[j] : INTRA_MODE_COUNT;
			intra_mode_t predicted_mode = (intra_mode_t)min(mode_top, mode_left);

			int idx = i * LUMA_BLOCK_COUNT + j;
			encode_luma_mode(modes_luma[idx], predicted_mode, cabac);

			LPC_DEBUG_ONLY(if (STATS) STATS->num_block_match_pred += (modes_luma[idx] == predicted_mode));
		}
	}

	// Chroma

	encode_chroma_mode(mode_chroma, neighbours, cabac);

	/// QP delta

	#if LPC_ADAPTIVE_QP
	encode_qp_delta(qp_delta, neighbours, cabac);
	#endif

	/// Residuals

	// Luma
	if (type == MB_TYPE_4x4)
	{
		for (int i = 0; i < LUMA_BLOCK_COUNT * LUMA_BLOCK_COUNT; i++)
			encode_residual_block(LUMA_BLOCK, residuals.luma[i].val, 16, cabac);
	}
	else // 16x16
	{
		encode_residual_block(LUMA_DC_BLOCK, residuals.luma_dc.val, 16, cabac);
		if (cbp_luma != 0)
		{
			for (int i = 0; i < LUMA_BLOCK_COUNT * LUMA_BLOCK_COUNT; i++)
				encode_residual_block(LUMA_AC_BLOCK, residuals.luma[i].val, 16, cabac);
		}
	}

	// Chroma
	{
		if (cbp_chroma & 3)
		{
			for (int plane = 0; plane < 2; plane++)
				encode_residual_block(CHROMA_DC_BLOCK, residuals.chroma_dc[plane].val, 4, cabac);
		}

		if (cbp_chroma & 2)
		{
			for (int plane = 0; plane < 2; plane++)
			{
				for (int i = 0; i < 2 * 2; i++)
					encode_residual_block(CHROMA_AC_BLOCK, residuals.chroma_ac[plane][i].val, 16, cabac);
			}
		}
	}
}

void predicted_macroblock_t::decode_mb(const neighbour_ctx_t &neighbours, mb_residuals_t *residuals,
		cabac_coder_t *cabac)
{
	decode_mb_type(this, neighbours, cabac);

	/// Modes

	// Luma
	if (type == MB_TYPE_4x4)
	{
		cbp_chroma = 2;

		for (int i = 0; i < LUMA_BLOCK_COUNT; i++)
		for (int j = 0; j < LUMA_BLOCK_COUNT; j++)
		{
			int mode_top = neighbours.top.valid ? neighbours.top.modes_luma[i] : INTRA_MODE_COUNT;
			int mode_left = neighbours.left.valid ? neighbours.left.modes_luma[j] : INTRA_MODE_COUNT;
			intra_mode_t predicted_mode = (intra_mode_t)min(mode_top, mode_left);

			int idx = i * LUMA_BLOCK_COUNT + j;
			decode_luma_mode(&modes_luma[idx], predicted_mode, cabac);
		}
	}

	// Chroma

	decode_chroma_mode(&mode_chroma, neighbours, cabac);

	/// QP delta

	#if LPC_ADAPTIVE_QP
	decode_qp_delta(&qp_delta, neighbours, cabac);
	qp += qp_delta;
	#endif

	/// Residuals

	// Luma
	if (type == MB_TYPE_4x4)
	{
		for (int i = 0; i < LUMA_BLOCK_COUNT * LUMA_BLOCK_COUNT; i++)
			decode_residual_block(LUMA_BLOCK, residuals->luma[i].val, 16, cabac);
	}
	else // 16x16
	{
		decode_residual_block(LUMA_DC_BLOCK, residuals->luma_dc.val, 16, cabac);
		for (int i = 0; i < LUMA_BLOCK_COUNT * LUMA_BLOCK_COUNT; i++)
		{
			if (cbp_luma != 0)
				decode_residual_block(LUMA_AC_BLOCK, residuals->luma[i].val, 16, cabac);
			else
			{
				for (int c = 1; c < 16; c++)
					residuals->luma[i].val[c] = 0;
			}
		}
	}

	// Chroma
	{
		for (int plane = 0; plane < 2; plane++)
		{
			if (cbp_chroma & 3)
				decode_residual_block(CHROMA_DC_BLOCK, residuals->chroma_dc[plane].val, 4, cabac);
			else
			{
				for (int c = 0; c < 4; c++)
					residuals->chroma_dc[plane].val[c] = 0;
			}
		}

		for (int plane = 0; plane < 2; plane++)
		{
			for (int i = 0; i < 2 * 2; i++)
			{
				if (cbp_chroma & 2)
					decode_residual_block(CHROMA_AC_BLOCK, residuals->chroma_ac[plane][i].val, 16, cabac);
				else
				{
					for (int c = 1; c < 16; c++)
						residuals->chroma_ac[plane][i].val[c] = 0;
				}
			}
		}
	}
}
