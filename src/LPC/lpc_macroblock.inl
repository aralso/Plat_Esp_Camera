#include "lpc.h"

#define MAX_COST(block_size) (block_size*block_size*255)
#define LPC_BIT(mode) (1 << (int)(mode))

static const int MB_SIZE = 16;
static const int LUMA_BLOCK_COUNT = 4;
static const int LUMA_BLOCK_SIZE = 4;
static const int CHROMA_BLOCK_SIZE = 8;

struct neighbour_ctx_t;
struct mb_residuals_t;

enum frame_type_t : uint8_t
{
	FRAME_TYPE_I,
	FRAME_TYPE_P,
};

enum mb_type_t : uint8_t
{
	MB_TYPE_I_4x4,
	MB_TYPE_I_16x16,
	MB_TYPE_P,
};

enum intra_mode_t : uint8_t
{
	INTRA_DC,
	INTRA_HORIZONTAL,
	INTRA_VERTICAL,
	INTRA_PLANE,

	INTRA_DIAGONAL_DOWN_LEFT,
	INTRA_DIAGONAL_DOWN_RIGHT,
	INTRA_VERTICAL_RIGHT,
	INTRA_HORIZONTAL_DOWN,
	INTRA_VERTICAL_LEFT,
	INTRA_HORIZONTAL_UP,

	INTRA_MODE_COUNT
};

const int LUMA_4x4_INTRA_MODES =
	LPC_BIT(INTRA_VERTICAL) |
	LPC_BIT(INTRA_HORIZONTAL) |
	LPC_BIT(INTRA_DC);
	//LPC_BIT(INTRA_DIAGONAL_DOWN_LEFT) |
	//LPC_BIT(INTRA_DIAGONAL_DOWN_RIGHT) |
	//LPC_BIT(INTRA_VERTICAL_RIGHT) |
	//LPC_BIT(INTRA_HORIZONTAL_DOWN) |
	//LPC_BIT(INTRA_VERTICAL_LEFT) |
	//LPC_BIT(INTRA_HORIZONTAL_UP);

const int LUMA_16x16_INTRA_MODES =
	LPC_BIT(INTRA_VERTICAL) |
	LPC_BIT(INTRA_HORIZONTAL) |
	LPC_BIT(INTRA_DC) |
	LPC_BIT(INTRA_PLANE);

const int CHROMA_INTRA_MODES =
	LPC_BIT(INTRA_VERTICAL) |
	LPC_BIT(INTRA_HORIZONTAL) |
	LPC_BIT(INTRA_DC) |
	LPC_BIT(INTRA_PLANE);

LPC_DEBUG_ONLY(static const char* to_string(intra_mode_t mode));

struct luma_block_t
{
	uint8_t Y[LUMA_BLOCK_SIZE*LUMA_BLOCK_SIZE];
};

struct chroma_block_t
{
	uint8_t C[CHROMA_BLOCK_SIZE*CHROMA_BLOCK_SIZE];
};

// A macroblock is 16x16 pixels in YUV 420
// So chrominance data is only 8x8
struct macroblock_t
{
	luma_block_t luma[LUMA_BLOCK_COUNT*LUMA_BLOCK_COUNT];
	chroma_block_t chroma_u;
	chroma_block_t chroma_v;

	void from_rgb(const uint8_t *rgb, int width, int height, int x, int y);
	void to_rgb(uint8_t *rgb, int width, int height, int x, int y) const;

	LPC_DEBUG_ONLY(void print(const char *msg = NULL, bool do_luma = true, bool do_chroma = true) const);
	LPC_DEBUG_ONLY(void print_luma() const);
	LPC_DEBUG_ONLY(void print_chroma(bool do_chroma_u = true, bool do_chroma_v = true) const);
};

struct predicted_macroblock_t
{
	macroblock_t mb;
	union
	{
		// 4x4
		intra_mode_t modes_luma[LUMA_BLOCK_COUNT*LUMA_BLOCK_COUNT];
		// 16x16
		struct
		{
			intra_mode_t mode_luma;
			uint8_t cbp_luma;
		};
	};
	intra_mode_t mode_chroma;
	uint8_t cbp_chroma;
	
	frame_type_t frame_type;
	mb_type_t type;
	uint8_t qp;
	uint8_t qp_backup;
	int8_t qp_delta;
	uint8_t qp_chroma_offset;

	void select_mode(const macroblock_t &orig, const neighbour_ctx_t &neighbours);
	void predict(const neighbour_ctx_t &neighbours);

	uint32_t select_intra_modes(const macroblock_t &orig, const neighbour_ctx_t &neighbours);
	void predict_intra(const neighbour_ctx_t &neighbours);

	void set_qp_delta(int8_t value);
	void compute_cbp_flags(const mb_residuals_t &residuals);

	void override_block_qp(uint8_t value);
	void restore_qp();

	void build_residuals(const macroblock_t &orig, mb_residuals_t *residuals) const;
	void add_residuals(mb_residuals_t &residuals);

	void encode_mb(const neighbour_ctx_t &neighbours, const mb_residuals_t &residuals,
		cabac_coder_t *cabac) const;
	void decode_mb(const neighbour_ctx_t &neighbours, mb_residuals_t *residuals,
		cabac_coder_t *cabac);

	LPC_DEBUG_ONLY(void print() const);
};

/// COST EVALUATION

// Sum of Absolute Differences
inline uint32_t cost_sad(uint8_t a, uint8_t b)
{
	return std::abs(a - b);
}

template <int SIZE>
uint32_t eval_cost(const uint8_t *a, const uint8_t *b)
{
	uint32_t cost = 0;

	for (int i = 0; i < SIZE * SIZE; i++)
		cost += std::abs(a[i] - b[i]);

	return cost;
}

uint32_t eval_cost(const luma_block_t &a, const luma_block_t &b)
{
	return eval_cost<LUMA_BLOCK_SIZE>(a.Y, b.Y);
}

uint32_t eval_cost(const chroma_block_t &a, const chroma_block_t &b)
{
	return eval_cost<CHROMA_BLOCK_SIZE>(a.C, b.C);
}
