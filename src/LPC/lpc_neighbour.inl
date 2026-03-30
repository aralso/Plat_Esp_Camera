#include "lpc.h"

struct neighbour_t
{
	bool valid;

	uint8_t *luma;
	uint8_t *chroma_u;
	uint8_t *chroma_v;

	intra_mode_t *modes_luma;
	intra_mode_t *mode_chroma;

	void init(uint8_t *Y, uint8_t *Cb, uint8_t *Cr,
			intra_mode_t *modes_Y, intra_mode_t *modes_CbCr,
			int offset = 0)
	{
		// The +1 is to allow sampling previous neighbour using
		// a negative index like in the h264 standard
		luma = Y + (offset * MB_SIZE) + 1;
		chroma_u = Cb + (offset * MB_SIZE / 2) + 1;
		chroma_v = Cr + (offset * MB_SIZE / 2) + 1;

		modes_luma = modes_Y + (offset * LUMA_BLOCK_COUNT);
		mode_chroma = modes_CbCr + offset;
	}

	void validate() { valid = true; }
	void invalidate() { valid = false; }

	const uint8_t *get_luma()     const { return valid ? luma : NULL; }
	const uint8_t *get_chroma_u() const { return valid ? chroma_u : NULL; }
	const uint8_t *get_chroma_v() const { return valid ? chroma_v : NULL; }
};

struct neighbour_ctx_t
{
	uint8_t top_luma[MB_SIZE + 1];
	uint8_t top_chroma_u[MB_SIZE/2 + 1];
	uint8_t top_chroma_v[MB_SIZE/2 + 1];
	intra_mode_t top_luma_modes[LUMA_BLOCK_COUNT];
	intra_mode_t top_chroma_mode;

	uint8_t *left_luma;
	uint8_t *left_chroma_u;
	uint8_t *left_chroma_v;
	intra_mode_t *left_luma_modes;
	intra_mode_t *left_chroma_modes;

	neighbour_t top, left;

	neighbour_ctx_t(neighbour_ctx_t &&) = delete;
	neighbour_ctx_t(int num_mb_height)
	{
		left_luma     = lpc_alloc<uint8_t>(num_mb_height * MB_SIZE + 1, "LUMA neighbours");
		left_chroma_u = lpc_alloc<uint8_t>(num_mb_height * MB_SIZE / 2 + 1, "CHROMA U neighbours");
		left_chroma_v = lpc_alloc<uint8_t>(num_mb_height * MB_SIZE / 2 + 1, "CHROMA V neighbours");

		left_luma_modes   = lpc_alloc<intra_mode_t>(num_mb_height * LUMA_BLOCK_COUNT, "LUMA MODES neighbours");
		left_chroma_modes = lpc_alloc<intra_mode_t>(num_mb_height, "CHROMA MODES neighbours");

		top.init(top_luma, top_chroma_u, top_chroma_v, top_luma_modes, &top_chroma_mode);
		left.invalidate();
	}
	~neighbour_ctx_t()
	{
		lpc_free(left_luma);
		lpc_free(left_chroma_u);
		lpc_free(left_chroma_v);

		lpc_free(left_luma_modes);
		lpc_free(left_chroma_modes);
	}

	void start_column()
	{
		top.invalidate();
	}

	void end_column()
	{
		left.validate();
	}

	void set_row(int y)
	{
		left.init(left_luma, left_chroma_u, left_chroma_v, left_luma_modes, left_chroma_modes, y);
	}

	void update_data(const struct predicted_macroblock_t &predicted);

	intra_mode_t get_predicted_luma_mode(int i, int j) const
	{
		int mode_top = top.valid ? top.modes_luma[i] : INTRA_MODE_COUNT;
		int mode_left = left.valid ? left.modes_luma[j] : INTRA_MODE_COUNT;
		return (intra_mode_t)min(mode_top, mode_left);
	}

	intra_mode_t get_predicted_chroma_mode() const
	{
		int mode_top = top.valid ? *top.mode_chroma : INTRA_MODE_COUNT;
		int mode_left = left.valid ? *left.mode_chroma : INTRA_MODE_COUNT;
		return (intra_mode_t)min(mode_top, mode_left);
	}

	LPC_DEBUG_ONLY(void print(const char *msg = NULL) const);
	LPC_DEBUG_ONLY(void print_all() const);
};
