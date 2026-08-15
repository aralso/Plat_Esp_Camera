/// CABAC CONTEXT

// Table 9-11
enum
{
	CTX_MB_TYPE_I_START = 3,				// I slice mb_type (ctxIdx 3-13)
	CTX_MB_TYPE_P_START = 14,				// P slice mb_type prefix (ctxIdx 14-16)
	CTX_MB_TYPE_P_SUFFIX = 17,				// P slice mb_type I-suffix (ctxIdx 17-22)
	CTX_MB_QP_DELTA_START = 60,				// mb_qp_delta (ctxIdx 60-63)
	CTX_INTRA_CHROMA_PRED_START = 64,		// intra_chroma_pred_mode (ctxIdx 64-67)
	CTX_PREV_INTRA_PRED_FLAG_START = 68,	// prev_intra*_pred_mode_flag (ctxIdx 68-72)
	CTX_CODED_BLOCK_PATTERN_START = 73,		// coded_block_pattern (ctxIdx 73-84)
	CTX_CODED_BLOCK_FLAG_START = 85,		// coded_block_flag (ctxIdx 85-104)
	CTX_SIG_COEFF_FLAG_START = 105,			// significant_coeff_flag (ctxIdx 105-165)
	CTX_LAST_SIG_COEFF_START = 166,			// last_significant_coeff_flag (ctxIdx 166-226)
	CTX_COEFF_ABS_LEVEL_START = 227,		// coeff_abs_level_minus1 (ctxIdx 227-275)

	CTX_MB_TYPE_SI_START = 0,				// SI slice mb_type (ctxIdx 0-2)
	CTX_MB_TYPE_B_START = 27,				// B slice mb_type prefix (ctxIdx 27-31)
	CTX_MB_TYPE_B_SUFFIX = 32,				// B slice mb_type I-suffix (ctxIdx 32-37)
	CTX_SUB_MB_TYPE_P_START = 21,			// P slice sub_mb_type (ctxIdx 21-23)
	CTX_SUB_MB_TYPE_B_START = 36,			// B slice sub_mb_type (ctxIdx 36-39)
	CTX_MVD_START = 40,						// mvd (ctxIdx 40-46)
	CTX_REF_IDX_START = 54,					// ref_idx (ctxIdx 54-59)

	CTX_COUNT = 276
};

struct cabac_ctx_t
{
	uint8_t state : 7;
	bool mps : 1;
};

/// ARITHMETIC CODER

#include "lpc_cabac_constants.inl"

struct cabac_coder_t
{
public:
	cabac_coder_t(lpc_stream_out_t *stream, int qp) // Encoder
	{
		stream_out = stream;
		init_contexts(qp);
		reset_e();
	}

	cabac_coder_t(lpc_stream_in_t *stream, int qp) // Decoder
	{
		stream_in = stream;
		init_contexts(qp);
		reset_d();
	}

	void encode_bytes(const uint8_t *bytes, size_t byte_count, int context);
	void encode_bit(bool bin, int context);
	void encode_bypass(bool bin);
	void encode_terminate(bool bin);

	void decode_bytes(uint8_t *bytes, size_t byte_count, int context);
	bool decode_bit(int context);
	bool decode_bypass();
	bool decode_terminate();

private:
	void init_contexts(int qp);
	void reset_e();
	void reset_d();

	void put_bit(bool bit);
	void renorm_e();
	void flush();

	bool decode_decision(int context);
	void renorm_d();

private:
	// Common
	cabac_ctx_t contexts[CTX_COUNT];
	uint32_t codIRange;

	union
	{
		// Encoding
		struct
		{
			uint32_t codILow : 31;
			uint32_t firstBitFlag : 1;
			uint32_t bitsOutstanding;
			LPC_DEBUG_ONLY(uint32_t symCnt);

			lpc_stream_out_t *stream_out;
		};

		// Decoding
		struct
		{
			uint32_t codIOffset;
			lpc_stream_in_t *stream_in;
		};
	};
};

void cabac_coder_t::init_contexts(int qp)
{
	// Section 9.3.1.1
	for (int i = 0; i < CTX_COUNT; i++)
	{
		int m = cst::g_ctx_init[i][0];
		int n = cst::g_ctx_init[i][1];

		int preCtxState = ((m * qp) >> 4) + n;

		if (preCtxState >= 64)
		{
			preCtxState = min(126, preCtxState);
			contexts[i].state = (preCtxState - 64);
			contexts[i].mps = true;
		}
		else
		{
			preCtxState = max(1, preCtxState);
			contexts[i].state = (63 - preCtxState);
			contexts[i].mps = false;
		}
	}
}

void cabac_coder_t::reset_e()
{
	codILow = 0;
	codIRange = 0x01FE;
	firstBitFlag = true;
	bitsOutstanding = 0;
	LPC_DEBUG_ONLY(symCnt = 0);
}

void cabac_coder_t::reset_d()
{
#if LPC_USE_CABAC == 0
	return;
#endif

	codIRange = 0x01FE;

	codIOffset = 0;
	for (int b = 0; b < 9; b++)
		codIOffset = (codIOffset << 1) | uint32_t(stream_in->read_bit());
}

/// Public API

void cabac_coder_t::encode_bytes(const uint8_t *bytes, size_t byte_count, int context)
{
	for (size_t i = 0; i < byte_count; i++)
	{
		uint8_t byte = bytes[i];
		for (int b = 0; b < 8; b++)
		{
			bool bit = (byte >> (7-b)) & 1;
			encode_bit(bit, context);
		}
	}
}

void cabac_coder_t::encode_bit(bool bin, int context)
{
	PROFILER_SCOPE(CABAC_ENCODE);

#if LPC_USE_CABAC == 0
	return stream_out->write_bit(bin);
#endif

	LPC_ASSERT(context < CTX_COUNT);

	cabac_ctx_t &ctx = contexts[context];

	int qCodIRangeIdx = (codIRange >> 6) & 3;
	int codIRangeLPS = cst::g_range_lps[ctx.state][qCodIRangeIdx];
	codIRange -= codIRangeLPS;

	if (bin == ctx.mps)
	{
		ctx.state = cst::g_next_state_mps[ctx.state];
	}
	else
	{
		codILow += codIRange;
		codIRange = codIRangeLPS;

		if (ctx.state == 0)
		{
			ctx.mps = bin;
		}

		ctx.state = cst::g_next_state_lps[ctx.state];
	}

	renorm_e();
	LPC_DEBUG_ONLY(symCnt++);
}

void cabac_coder_t::encode_bypass(bool bin)
{
	PROFILER_SCOPE(CABAC_BYPASS);

#if LPC_USE_CABAC == 0
	return stream_out->write_bit(bin);
#endif

	codILow <<= 1;

	if (bin)
	{
		codILow += codIRange;
	}

	if (codILow >= 0x400)
	{
		put_bit(1);
		codILow -= 0x400;
	}
	else if (codILow >= 0x200)
	{
		codILow -= 0x200;
		bitsOutstanding++;
	}
	else
	{
		put_bit(0);
	}

	LPC_DEBUG_ONLY(symCnt++);
}

void cabac_coder_t::encode_terminate(bool bin)
{
#if LPC_USE_CABAC == 0
	return stream_out->write_bit(bin);
#endif

	codIRange -= 2;

	if (bin)
	{
		codILow += codIRange;
		flush();
	}
	else
	{
		renorm_e();
	}

	LPC_DEBUG_ONLY(symCnt++);
}

void cabac_coder_t::decode_bytes(uint8_t *bytes, size_t byte_count, int context)
{
	for (size_t i = 0; i < byte_count; i++)
	{
		uint8_t byte = 0;
		for (int b = 0; b < 8; b++)
		{
			bool bit = decode_bit(context);
			byte |= (bit << (7-b));
		}

		bytes[i] = byte;
	}
}

bool cabac_coder_t::decode_bit(int context)
{
#if LPC_USE_CABAC == 0
	return stream_in->read_bit();
#endif

	LPC_ASSERT(context < CTX_COUNT);

	if (context == CTX_COUNT)
	{
		return decode_terminate();
	}
	else
	{
		return decode_decision(context);
	}
}

bool cabac_coder_t::decode_bypass()
{
#if LPC_USE_CABAC == 0
	return stream_in->read_bit();
#endif

	codIOffset = (codIOffset << 1) | uint32_t(stream_in->read_bit());

	if (codIOffset >= codIRange)
	{
		codIOffset -= codIRange;
		return true;
	}

	return false;
}

bool cabac_coder_t::decode_terminate()
{
#if LPC_USE_CABAC == 0
	return stream_in->read_bit();
#endif

	codIRange -= 2;

	if (codIOffset >= codIRange)
		return true;

	renorm_d();
	return false;
}

/// Encode utility

void cabac_coder_t::put_bit(bool bit)
{
	if (firstBitFlag != false)
	{
		firstBitFlag = false;
	}
	else
	{
		stream_out->write_bit(bit);
	}

	while (bitsOutstanding > 0)
	{
		stream_out->write_bit(!bit);
		bitsOutstanding--;
	}
}

void cabac_coder_t::renorm_e()
{
	while (codIRange < 0x100)
	{
		if (codILow >= 0x200)
		{
			codILow -= 0x200;
			put_bit(true);
		}
		else if (codILow >= 0x100)
		{
			codILow -= 0x100;
			bitsOutstanding++;
		}
		else
		{
			put_bit(false);
		}

		codIRange <<= 1;
		codILow <<= 1;
	}
}

void cabac_coder_t::flush()
{
	codIRange = 2;
	renorm_e();
	put_bit((codILow >> 9) & 1);
	put_bit((codILow >> 8) & 1);
	put_bit(1);
}

/// Decode utility

bool cabac_coder_t::decode_decision(int context)
{
	cabac_ctx_t &ctx = contexts[context];

	int qCodIRangeIdx = (codIRange >> 6) & 3;
	int codIRangeLPS = cst::g_range_lps[ctx.state][qCodIRangeIdx];
	codIRange -= codIRangeLPS;

	bool bit;
	if (codIOffset >= codIRange) // LPS
	{
		bit = !ctx.mps;
		codIOffset -= codIRange;
		codIRange = codIRangeLPS;

		if (ctx.state == 0)
		{
			ctx.mps = bit;
		}

		ctx.state = cst::g_next_state_lps[ctx.state];
	}
	else // MPS
	{
		bit = ctx.mps;
		ctx.state = cst::g_next_state_mps[ctx.state];
	}

	renorm_d();
	return bit;
}

void cabac_coder_t::renorm_d()
{
	while (codIRange < 0x100)
	{
		codIRange <<= 1;
		codIOffset = (codIOffset << 1) | uint32_t(stream_in->read_bit());
	}
}
