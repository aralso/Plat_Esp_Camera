#include "lpc.h"

/// MODE SELECTION

inline uint32_t find_mode_luma_blocks(const macroblock_t &orig,
		const uint8_t *top, const uint8_t *left,
		predicted_macroblock_t *predicted)
{
	uint32_t total_cost = 0;

	for (int block_i = 0; block_i < LUMA_BLOCK_COUNT; block_i++)
	{
		for (int block_j = 0; block_j < LUMA_BLOCK_COUNT; block_j++)
		{
			// Find the best prediction mode for this block
			int block_idx = block_i * LUMA_BLOCK_COUNT + block_j;
			auto &block = orig.luma[block_idx];
			auto *pred_block = &predicted->mb.luma[block_idx];
			auto *pred_mode = &predicted->modes_luma[block_idx];

			const uint8_t *block_top = top ? &top[block_i * LUMA_BLOCK_SIZE] : NULL;
			const uint8_t *block_left = left ? &left[block_j * LUMA_BLOCK_SIZE] : NULL;

			uint32_t pred_cost = find_mode_luma_4x4(block,
					block_top, block_left, pred_block, pred_mode);
			total_cost += pred_cost;
		}
	}

	return total_cost;
}

void predicted_macroblock_t::select_intra_modes(const macroblock_t &orig, const neighbour_ctx_t &neighbours)
{
	const neighbour_t &top = neighbours.top;
	const neighbour_t &left = neighbours.left;

	// Luma 16x16
	{
		const uint8_t *orig_luma = (uint8_t*)orig.luma;
		uint8_t *predicted_luma = (uint8_t*)mb.luma;
		uint32_t cost_16x16 = find_mode_luma_16x16(orig_luma,
				top.get_luma(), left.get_luma(),
				predicted_luma, &mode_luma);

		type = (cost_16x16 > 200 * uint32_t(qp)) ? MB_TYPE_4x4 : MB_TYPE_16x16;
		//type = MB_TYPE_4x4;
		//type = MB_TYPE_16x16;
	}

	// Luma 4x4
	if (type == MB_TYPE_4x4)
	{
		find_mode_luma_blocks(orig, top.get_luma(), left.get_luma(), this);
	}

	// Chroma
	{
		uint32_t cost_chroma = find_mode_chroma(orig, top, left, &mb, &mode_chroma);
	}
}

/// INTRA PREDICTION

void predict_luma_blocks(const intra_mode_t *modes,
		const uint8_t *top, const uint8_t *left, luma_block_t *blocks)
{
	for (int block_i = 0; block_i < LUMA_BLOCK_COUNT; block_i++)
	{
		for (int block_j = 0; block_j < LUMA_BLOCK_COUNT; block_j++)
		{
			int block_idx = block_i * LUMA_BLOCK_COUNT + block_j;
			intra_mode_t mode = modes[block_idx];
			luma_block_t *block = blocks + block_idx;

			const uint8_t *block_top = top ? &top[block_i * LUMA_BLOCK_SIZE] : NULL;
			const uint8_t *block_left = left ? &left[block_j * LUMA_BLOCK_SIZE] : NULL;

			predict_luma_4x4(*block, mode, block_top, block_left, block);
		}
	}
}

void predicted_macroblock_t::predict(const neighbour_ctx_t &neighbours)
{
	const neighbour_t &top = neighbours.top;
	const neighbour_t &left = neighbours.left;

	if (type == MB_TYPE_16x16)
	{
		uint8_t *block = (uint8_t*)mb.luma;
		predict_luma_16x16(block, mode_luma, top.get_luma(), left.get_luma(), block);
	}
	else
	{
		predict_luma_blocks(modes_luma, top.get_luma(), left.get_luma(), mb.luma);
	}

	predict_chroma(mb.chroma_u, mode_chroma, top.get_chroma_u(), left.get_chroma_u(),
			&mb.chroma_u);
	predict_chroma(mb.chroma_v, mode_chroma, top.get_chroma_v(), left.get_chroma_v(),
			&mb.chroma_v);
}

/// ADAPTIVE QP

uint32_t compute_variance(const luma_block_t &block)
{
	uint32_t mean = 0;
	uint32_t var = 0;

	// TODO: do we have the precision to do it in 1 pass ?

	// Compute mean
	for (int i = 0; i < LUMA_BLOCK_SIZE*LUMA_BLOCK_SIZE; i++)
		mean += block.Y[i];
	mean /= (LUMA_BLOCK_SIZE*LUMA_BLOCK_SIZE);

	// Compute variance
	for (int i = 0; i < LUMA_BLOCK_SIZE*LUMA_BLOCK_SIZE; i++)
	{
		int32_t diff = (int32_t)block.Y[i] - mean;
		var += diff * diff;
	}

	return var / (LUMA_BLOCK_SIZE*LUMA_BLOCK_SIZE);
}

uint32_t compute_variance(const macroblock_t &mb)
{
	uint32_t var = 0;
	for (int b = 0; b < LUMA_BLOCK_COUNT*LUMA_BLOCK_COUNT; b++)
		var += compute_variance(mb.luma[b]);

	return var / (LUMA_BLOCK_COUNT*LUMA_BLOCK_COUNT);
}

int compute_qp_delta(const macroblock_t &mb, float log_var_avg)
{
	float bias = 0.0f;
	uint32_t variance = compute_variance(mb);
	float log_var = logf(variance + 1.0f) - bias;

	// Only increase quantization when we are below the avg variance
	if (log_var >= log_var_avg)
		return 0;

	float strength = 5.0f;
	return int((log_var_avg - log_var) * strength);
}

void predicted_macroblock_t::set_qp_delta(int8_t value)
{
#if LPC_ADAPTIVE_QP
	qp_delta = min(max(-26, (int)value), 25);
	qp_delta = min(max(0, (int)qp + qp_delta), QP_MAX) - qp;

	LPC_ASSERT((int)qp + qp_delta >= 0);
	LPC_ASSERT((int)qp + qp_delta <= QP_MAX);
	qp += qp_delta;
#endif
}

/// RESIDUALS

void predicted_macroblock_t::build_residuals(const macroblock_t &orig, mb_residuals_t *residuals) const
{
	// Luma
	if (type == MB_TYPE_4x4)
	{
		for (int block_i = 0; block_i < LUMA_BLOCK_COUNT; block_i++)
		{
			for (int block_j = 0; block_j < LUMA_BLOCK_COUNT; block_j++)
			{
				int block_idx = block_i * LUMA_BLOCK_COUNT + block_j;
				auto &block = orig.luma[block_idx];
				auto *pred_block = &mb.luma[block_idx];

				// Compute the residuals
				for (int i = 0; i < 4 * 4; i++)
					residuals->luma[block_idx].val[i] = block.Y[i] - pred_block->Y[i];

				residuals->luma[block_idx].transform();
				residuals->luma[block_idx].quantize(qp);
			}
		}
	}
	else // 16x16
	{
		for (int block_i = 0; block_i < LUMA_BLOCK_COUNT; block_i++)
		{
			for (int block_j = 0; block_j < LUMA_BLOCK_COUNT; block_j++)
			{
				int block_idx = block_i * LUMA_BLOCK_COUNT + block_j;
				auto &block = orig.luma[block_idx];
				auto *pred_block = &mb.luma[block_idx];

				// Compute the residuals
				for (int i = 0; i < 4 * 4; i++)
					residuals->luma[block_idx].val[i] = block.Y[i] - pred_block->Y[i];

				// Transform and move DC to their own block
				residuals->luma[block_idx].transform();
				residuals->luma_dc.val[block_idx] = residuals->luma[block_idx].val[0];
				residuals->luma[block_idx].quantize(qp);
			}
		}

		residuals->luma_dc.dc_transform();
		residuals->luma_dc.dc_quantize(qp);
	}

	// Chroma
	{
		uint8_t qpc = cst::compute_qp_chroma(qp + qp_chroma_offset);

		for (int block_i = 0; block_i < 2; block_i++)
		{
			for (int block_j = 0; block_j < 2; block_j++)
			{
				int block_idx = block_i * 2 + block_j;
				int block_offset = (block_i * 4) * 8 + (block_j * 4);

				// Compute the residuals
				for (int i = 0; i < 4; i++)
				{
					for (int j = 0; j < 4; j++)
					{
						int c_idx = block_offset + i * 8 + j;
						int r_idx = i * 4 + j;

						int orig_u = orig.chroma_u.C[c_idx];
						int pred_u = mb.chroma_u.C[c_idx];
						residuals->chroma_ac[0][block_idx].val[r_idx] = orig_u - pred_u;

						int orig_v = orig.chroma_v.C[c_idx];
						int pred_v = mb.chroma_v.C[c_idx];
						residuals->chroma_ac[1][block_idx].val[r_idx] = orig_v - pred_v;
					}
				}

				for (int plane = 0; plane < 2; plane++)
				{
					// Transform and move DC to their own block
					residuals->chroma_ac[plane][block_idx].transform();
					residuals->chroma_dc[plane].val[block_idx] = residuals->chroma_ac[plane][block_idx].val[0];
					residuals->chroma_ac[plane][block_idx].quantize(qpc);
				}
			}
		}

		for (int plane = 0; plane < 2; plane++)
		{
			residuals->chroma_dc[plane].transform();
			residuals->chroma_dc[plane].quantize(qpc);
		}
	}
}

void predicted_macroblock_t::add_residuals(mb_residuals_t &residuals)
{
	// Luma
	if (type == MB_TYPE_4x4)
	{
		for (int block_i = 0; block_i < LUMA_BLOCK_COUNT; block_i++)
		{
			for (int block_j = 0; block_j < LUMA_BLOCK_COUNT; block_j++)
			{
				int block_idx = block_i * LUMA_BLOCK_COUNT + block_j;
				auto &block = mb.luma[block_idx];
				auto &resid = residuals.luma[block_idx];

				// Inverse transform
				resid.inverse_quantize(qp);
				resid.inverse_transform();

				// Add residuals to prediction
				for (int i = 0; i < 4 * 4; i++)
					block.Y[i] = clamp8(block.Y[i] + resid.val[i]);
			}
		}
	}
	else // 16x16
	{
		// Dequantize
		residuals.luma_dc.dc_inverse_quantize(qp);
		residuals.luma_dc.dc_inverse_transform();

		for (int block_i = 0; block_i < LUMA_BLOCK_COUNT; block_i++)
		{
			for (int block_j = 0; block_j < LUMA_BLOCK_COUNT; block_j++)
			{
				int block_idx = block_i * LUMA_BLOCK_COUNT + block_j;
				auto &block = mb.luma[block_idx];
				auto &resid = residuals.luma[block_idx];

				// Restore the DC and transform
				resid.inverse_quantize(qp);
				resid.val[0] = residuals.luma_dc.val[block_idx];
				resid.inverse_transform();

				// Add residuals to prediction
				for (int i = 0; i < 4 * 4; i++)
					block.Y[i] = clamp8(block.Y[i] + resid.val[i]);
			}
		}
	}

	// Chroma
	{
		uint8_t qpc = cst::compute_qp_chroma(qp + qp_chroma_offset);

		for (int plane = 0; plane < 2; plane++)
		{
			residuals.chroma_dc[plane].inverse_quantize(qpc);
			residuals.chroma_dc[plane].inverse_transform();
		}

		for (int block_i = 0; block_i < 2; block_i++)
		{
			for (int block_j = 0; block_j < 2; block_j++)
			{
				int block_idx = block_i * 2 + block_j;
				int block_offset = (block_i * 4) * 8 + (block_j * 4);

				// Restore the DC and transform
				for (int plane = 0; plane < 2; plane++)
				{
					auto &resid = residuals.chroma_ac[plane][block_idx];

					resid.inverse_quantize(qpc);
					resid.val[0] = residuals.chroma_dc[plane].val[block_idx];
					resid.inverse_transform();
				}

				// Add residuals to prediction
				for (int i = 0; i < 4; i++)
				{
					for (int j = 0; j < 4; j++)
					{
						int c_idx = block_offset + i * 8 + j;
						int r_idx = i * 4 + j;

						int resid_u = residuals.chroma_ac[0][block_idx].val[r_idx];
						mb.chroma_u.C[c_idx] = clamp8(mb.chroma_u.C[c_idx] + resid_u);

						int resid_v = residuals.chroma_ac[1][block_idx].val[r_idx];
						mb.chroma_v.C[c_idx] = clamp8(mb.chroma_v.C[c_idx] + resid_v);
					}
				}
			}
		}
	}
}
