#include "lpc.h"

// Ideally turn that on but increases size currently
#define USE_PRED_MODE 0

#if USE_PRED_MODE
#define PRED_ENCODE(x) x
#define PRED_DECODE(x) x
#else
#define PRED_ENCODE(x) (match_pred = false)
#define PRED_DECODE(x) false
#endif

struct predicted_macroblock_t
{
	macroblock_t mb;
	union
	{
		intra_mode_t modes_luma[LUMA_BLOCK_COUNT*LUMA_BLOCK_COUNT]; // 4x4
		intra_mode_t mode_luma; // 16x16
	};
	intra_mode_t mode_chroma;

	mb_type_t type;
	uint8_t qp, qpc;
	uint32_t cost_threshold;

	// Intra mode selection
	void select_intra_modes(const macroblock_t &orig,
			const neighbour_t &top, const neighbour_t &left);

	void predict(const neighbour_t &top, const neighbour_t &left);

	// IO
	static const int LUMA_4x4_MODE_BITS = 2;
	static const int LUMA_16x16_MODE_BITS = 2;
	static const int CHROMA_MODE_BITS = 2;

	void encode_mode(lpc_cabac_t *cabac, intra_mode_t mode, int bit_count)
	{
		LPC_ASSERT((mode & ~((1 << bit_count) - 1)) == 0);

		for (int b = 0; b < bit_count; b++)
			cabac->encode_bit(mode & (1 << b));
	}

	intra_mode_t decode_mode(lpc_cabac_t *cabac, int bit_count)
	{
		int mode = 0;
		for (int b = 0; b < bit_count; b++)
			mode |= cabac->decode_bit() << b;
		return (intra_mode_t)mode;
	}

	void encode_modes(lpc_cabac_t *cabac, const neighbour_ctx_t &neighbours)
	{
		bool is_4x4 = type == MB_TYPE_4x4 ? true : false;
		cabac->encode_bit(is_4x4);

		// Luma
		if (is_4x4)
		{
			for (int i = 0; i < LUMA_BLOCK_COUNT; i++)
			for (int j = 0; j < LUMA_BLOCK_COUNT; j++)
			{
				int idx = i * LUMA_BLOCK_COUNT + j;
				intra_mode_t mode = modes_luma[idx];

				bool match_pred = (mode == neighbours.get_predicted_luma_mode(i, j));
				LPC_DEBUG_ONLY(STATS->num_block_match_pred[STAT_LUMA_4x4] += match_pred);
				PRED_ENCODE(cabac->encode_bit(!match_pred));

				if (!match_pred)
					encode_mode(cabac, mode, LUMA_4x4_MODE_BITS);
			}
		}
		else // 16x16
		{
			bool match_pred = (mode_luma == neighbours.get_predicted_luma_mode(0, 0));
			LPC_DEBUG_ONLY(STATS->num_block_match_pred[STAT_LUMA_16x16] += match_pred);
			PRED_ENCODE(cabac->encode_bit(!match_pred));

			if (!match_pred)
				encode_mode(cabac, mode_luma, LUMA_16x16_MODE_BITS);
		}

		// Chroma
		{
			bool match_pred = (mode_chroma == neighbours.get_predicted_chroma_mode());
			LPC_DEBUG_ONLY(STATS->num_block_match_pred[STAT_CHROMA] += match_pred);
			PRED_ENCODE(cabac->encode_bit(!match_pred));

			if (!match_pred)
				encode_mode(cabac, mode_chroma, CHROMA_MODE_BITS);
		}
	}

	void decode_modes(lpc_cabac_t *cabac, const neighbour_ctx_t &neighbours)
	{
		bool is_4x4 = cabac->decode_bit();
		type = is_4x4 ? MB_TYPE_4x4 : MB_TYPE_16x16;

		// Luma
		if (is_4x4)
		{
			for (int i = 0; i < LUMA_BLOCK_COUNT; i++)
			for (int j = 0; j < LUMA_BLOCK_COUNT; j++)
			{
				int idx = i * LUMA_BLOCK_COUNT + j;

				bool match_pred = PRED_DECODE(cabac->decode_bit() == 0);
				if (match_pred)
					modes_luma[idx] = neighbours.get_predicted_luma_mode(i, j);
				else
					modes_luma[idx] = decode_mode(cabac, LUMA_4x4_MODE_BITS);
			}
		}
		else // 16x16
		{
			bool match_pred = PRED_DECODE(cabac->decode_bit() == 0);
			if (match_pred)
				mode_luma = neighbours.get_predicted_luma_mode(0, 0);
			else
				mode_luma = decode_mode(cabac, LUMA_16x16_MODE_BITS);
		}

		// Chroma
		{
			bool match_pred = PRED_DECODE(cabac->decode_bit() == 0);
			if (match_pred)
				mode_chroma = neighbours.get_predicted_chroma_mode();
			else
				mode_chroma = decode_mode(cabac, CHROMA_MODE_BITS);
		}
	}

	LPC_DEBUG_ONLY(void print() const);
};

/// MODE SELECTION

void find_mode_luma_blocks(const macroblock_t &orig,
		const uint8_t *top, const uint8_t *left,
		predicted_macroblock_t *predicted)
{
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

			#ifdef LPC_DEBUG
			if (STATS && pred_cost < predicted->cost_threshold)
				STATS->num_block_below_threshold[STAT_LUMA_4x4]++;
			#endif
		}
	}
}

void predicted_macroblock_t::select_intra_modes(const macroblock_t &orig,
		const neighbour_t &top, const neighbour_t &left)
{
	// Evaluate 16x16 cost
	{
		const uint8_t *orig_luma = (uint8_t*)orig.luma;
		uint8_t *predicted_luma = (uint8_t*)mb.luma;
		uint32_t cost_16x16 = find_mode_luma_16x16(orig_luma,
				top.get_luma(), left.get_luma(), predicted_luma, &mode_luma);

		#ifdef LPC_DEBUG
		if (STATS && cost_16x16 < cost_threshold)
			STATS->num_block_below_threshold[STAT_LUMA_16x16]++;
		#endif

		//cost_16x16 = -1; // Force MB_TYPE_4x4
		//cost_16x16 = 0; // Force MB_TYPE_16x16
		type = cost_16x16 > 50 * qp ? MB_TYPE_4x4 : MB_TYPE_16x16;
	}

	if (type == MB_TYPE_4x4)
	{
		find_mode_luma_blocks(orig, top.get_luma(), left.get_luma(), this);
	}

	uint32_t cost_chroma = find_mode_chroma(orig, top, left, &mb, &mode_chroma);

	#ifdef LPC_DEBUG
	if (STATS && cost_chroma < cost_threshold)
		STATS->num_block_below_threshold[STAT_CHROMA]++;
	#endif
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

void predicted_macroblock_t::predict(const neighbour_t &top, const neighbour_t &left)
{
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

/// NEIGHBOUR UPDATE

void neighbour_ctx_t::update_data(const predicted_macroblock_t &predicted)
{
	const macroblock_t &mb = predicted.mb;

	// Update corner data that might have been skipped
	// See comments below
	if (left.valid && top.valid)
	{
		left.luma[-1] = left_luma[0];
		left.chroma_u[-1] = left_chroma_u[0];
		left.chroma_v[-1] = left_chroma_v[0];
	}

	// Luma
	for (int i = 0; i < MB_SIZE; i++)
	{
		int block_i = i / 4;
		int pos_i = i % 4;

		// Data
		if (left.valid && i == MB_SIZE - 1)
		{
			// This will be used as corner data for the next macroblock
			// so don't overwrite it, but cache the value
			left_luma[0] = mb.luma[3*4+block_i].Y[3*4+pos_i];

			// Also make it available from the top row
			top.luma[-1] = left.luma[i];
		}
		else
		{
			left.luma[i] = mb.luma[3*4+block_i].Y[3*4+pos_i];
		}

		top.luma[i] = mb.luma[block_i*4+3].Y[pos_i*4+3];

		// Modes
		if (predicted.type == MB_TYPE_4x4)
		{
			for (int b = 0; b < LUMA_BLOCK_COUNT; b++)
			{
				left.modes_luma[b] = predicted.modes_luma[b];
				top.modes_luma[b] = predicted.modes_luma[b];
			}
		}
		else
		{
			for (int b = 0; b < LUMA_BLOCK_COUNT; b++)
			{
				left.modes_luma[b] = predicted.mode_luma;
				top.modes_luma[b] = predicted.mode_luma;
			}
		}
	}

	// Chroma
	for (int i = 0; i < MB_SIZE/2; i++)
	{
		// Data
		if (left.valid && i == (MB_SIZE/2) - 1)
		{
			left_chroma_u[0] = mb.chroma_u.C[7*8+i];
			left_chroma_v[0] = mb.chroma_v.C[7*8+i];

			top.chroma_u[-1] = left.chroma_u[i];
			top.chroma_v[-1] = left.chroma_v[i];
		}
		else
		{
			left.chroma_u[i] = mb.chroma_u.C[7*8+i];
			left.chroma_v[i] = mb.chroma_v.C[7*8+i];
		}

		top.chroma_u[i] = mb.chroma_u.C[i*8+7];
		top.chroma_v[i] = mb.chroma_v.C[i*8+7];

		// Modes
		*left.mode_chroma = predicted.mode_chroma;
		*top.mode_chroma = predicted.mode_chroma;
	}

	top.validate();
}
