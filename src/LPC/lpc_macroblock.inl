#include "lpc.h"

#define MAX_COST(block_size) (block_size*block_size*255)
#define BIT(mode) (1 << (int)(mode))

static const int MB_SIZE = 16;
static const int LUMA_BLOCK_COUNT = 4;
static const int LUMA_BLOCK_SIZE = 4;
static const int CHROMA_BLOCK_SIZE = 8;

struct neighbour_ctx_t;
struct mb_residuals_t;

enum mb_type_t : uint8_t
{
	MB_TYPE_4x4,
	MB_TYPE_16x16,
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
	BIT(INTRA_VERTICAL) |
	BIT(INTRA_HORIZONTAL) |
	BIT(INTRA_DC);
	//BIT(INTRA_DIAGONAL_DOWN_LEFT) |
	//BIT(INTRA_DIAGONAL_DOWN_RIGHT) |
	//BIT(INTRA_VERTICAL_RIGHT) |
	//BIT(INTRA_HORIZONTAL_DOWN) |
	//BIT(INTRA_VERTICAL_LEFT) |
	//BIT(INTRA_HORIZONTAL_UP);

const int LUMA_16x16_INTRA_MODES =
	BIT(INTRA_VERTICAL) |
	BIT(INTRA_HORIZONTAL) |
	BIT(INTRA_DC) |
	BIT(INTRA_PLANE);

const int CHROMA_INTRA_MODES =
	BIT(INTRA_VERTICAL) |
	BIT(INTRA_HORIZONTAL) |
	BIT(INTRA_DC) |
	BIT(INTRA_PLANE);

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
	void to_rgb(uint8_t *rgb, int width, int height, int x, int y);

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

	mb_type_t type;
	uint8_t qp;
	int8_t qp_delta;
	uint8_t qp_chroma_offset;

	void select_intra_modes(const macroblock_t &orig, const neighbour_ctx_t &neighbours);
	void predict(const neighbour_ctx_t &neighbours);

	void set_qp_delta(int8_t value);
	void compute_cbp_flags(const mb_residuals_t &residuals);

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
