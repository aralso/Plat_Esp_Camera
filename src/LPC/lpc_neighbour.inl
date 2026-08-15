#include "lpc.h"

struct neighbour_t
{
	bool valid;

	mb_type_t *type;

	uint8_t *luma;
	uint8_t *chroma_u;
	uint8_t *chroma_v;

	intra_mode_t *modes_luma;
	intra_mode_t *mode_chroma;

	void init(uint8_t *Y, uint8_t *Cb, uint8_t *Cr,
			intra_mode_t *modes_Y, intra_mode_t *modes_CbCr,
			mb_type_t *mb_type,
			int offset = 0)
	{
		// The +1 is to allow sampling previous neighbour using
		// a negative index like in the h264 standard
		luma = Y + (offset * MB_SIZE) + 1;
		chroma_u = Cb + (offset * MB_SIZE / 2) + 1;
		chroma_v = Cr + (offset * MB_SIZE / 2) + 1;

		modes_luma = modes_Y + (offset * LUMA_BLOCK_COUNT);
		mode_chroma = modes_CbCr + offset;

		type = mb_type + offset;
	}

	void validate() { valid = true; }
	void invalidate() { valid = false; }

	const uint8_t *get_luma()     const { return valid ? luma : nullptr; }
	const uint8_t *get_chroma_u() const { return valid ? chroma_u : nullptr; }
	const uint8_t *get_chroma_v() const { return valid ? chroma_v : nullptr; }

	intra_mode_t get_mode_luma(int i, intra_mode_t invalid_val = INTRA_MODE_COUNT) const
	{ return valid && *type != MB_TYPE_P ? modes_luma[i] : invalid_val; }

	intra_mode_t get_mode_chroma(intra_mode_t invalid_val = INTRA_MODE_COUNT) const
	{ return valid && *type != MB_TYPE_P ? *mode_chroma : invalid_val; }
};

struct neighbour_ctx_t
{
	uint8_t top_luma[MB_SIZE + 1];
	uint8_t top_chroma_u[MB_SIZE/2 + 1];
	uint8_t top_chroma_v[MB_SIZE/2 + 1];
	intra_mode_t top_luma_modes[LUMA_BLOCK_COUNT];
	intra_mode_t top_chroma_mode;
	mb_type_t top_type;

	uint8_t *left_luma;
	uint8_t *left_chroma_u;
	uint8_t *left_chroma_v;
	intra_mode_t *left_luma_modes;
	intra_mode_t *left_chroma_modes;
	mb_type_t *left_types;

	int num_mb_y;
	uint8_t prev_qp_delta;
	macroblock_t *prev_frame;

	neighbour_t top, left;
	macroblock_t *previous;

	neighbour_ctx_t(neighbour_ctx_t &&) = delete;
	neighbour_ctx_t(int num_mb_height, macroblock_t *previous_frame = nullptr)
	{
		prev_qp_delta = 0;
		prev_frame = previous_frame;
		num_mb_y = num_mb_height;
		previous = nullptr;

		left_luma     = lpc_alloc<uint8_t>(num_mb_height * MB_SIZE + 1, "LUMA neighbours");
		left_chroma_u = lpc_alloc<uint8_t>(num_mb_height * MB_SIZE / 2 + 1, "CHROMA U neighbours");
		left_chroma_v = lpc_alloc<uint8_t>(num_mb_height * MB_SIZE / 2 + 1, "CHROMA V neighbours");

		left_luma_modes   = lpc_alloc<intra_mode_t>(num_mb_height * LUMA_BLOCK_COUNT, "LUMA MODES neighbours");
		left_chroma_modes = lpc_alloc<intra_mode_t>(num_mb_height, "CHROMA MODES neighbours");

		left_types = lpc_alloc<mb_type_t>(num_mb_height, "MB TYPE neighbours");

		top.init(top_luma, top_chroma_u, top_chroma_v, top_luma_modes, &top_chroma_mode, &top_type);
	}
	~neighbour_ctx_t()
	{
		lpc_free(left_luma);
		lpc_free(left_chroma_u);
		lpc_free(left_chroma_v);

		lpc_free(left_luma_modes);
		lpc_free(left_chroma_modes);

		lpc_free(left_types);
	}

	void set_coords(int x, int y)
	{
		left.init(left_luma, left_chroma_u, left_chroma_v, left_luma_modes, left_chroma_modes, left_types, y);
		if (prev_frame) previous = prev_frame + x * num_mb_y + y;

		left.valid = (x != 0);
		top.valid = (y != 0);
	}

	void update_data(const struct predicted_macroblock_t &predicted);

	LPC_DEBUG_ONLY(void print(const char *msg = NULL) const);
	LPC_DEBUG_ONLY(void print_all() const);
};

void neighbour_ctx_t::update_data(const predicted_macroblock_t &predicted)
{
	PROFILER_SCOPE(NEIGHBOUR_UPDATE);

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
		if (predicted.type == MB_TYPE_I_4x4)
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

	// Type
	*left.type = predicted.type;
	*top.type = predicted.type;

	prev_qp_delta = predicted.qp_delta;
}
