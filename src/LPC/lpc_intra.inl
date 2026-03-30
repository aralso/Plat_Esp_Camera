#include "lpc.h"

/// INTRA PREDICTION

template <int BLOCK_SIZE>
uint32_t vertical_prediction(const uint8_t *top, const uint8_t *left, uint8_t *result)
{
	if (!top) return MAX_COST;

	for (int x = 0; x < BLOCK_SIZE; x++)
		for (int y = 0; y < BLOCK_SIZE; y++)
			result[x * BLOCK_SIZE + y] = top[x];

	return 0;
}

template <int BLOCK_SIZE>
uint32_t horizontal_prediction(const uint8_t *top, const uint8_t *left, uint8_t *result)
{
	if (!left) return MAX_COST;

	for (int x = 0; x < BLOCK_SIZE; x++)
		for (int y = 0; y < BLOCK_SIZE; y++)
			result[x * BLOCK_SIZE + y] = left[y];

	return 0;
}

template <int BLOCK_SIZE>
void dc_filter(const uint8_t *top, const uint8_t *left, uint8_t *result)
{
	result[0] = (top[0] + left[0] + 2 * result[0] + 2) >> 2;

	for (int x = 1; x < BLOCK_SIZE; x++)
		result[x * BLOCK_SIZE] = (top[x] + 3 * result[x * BLOCK_SIZE] + 2) >> 2;

	for (int y = 1; y < BLOCK_SIZE; y++)
		result[y] = (left[y] + 3 * result[y] + 2) >> 2;
}

template <int BLOCK_SIZE, int LOG2_BLOCK_SIZE>
uint32_t dc_prediction(const uint8_t *top, const uint8_t *left, uint8_t *result)
{
	uint8_t dc = 1 << 7;
	if (top || left)
	{
		int sum = 0;
		if (top)
		{
			sum += BLOCK_SIZE/2;
			for (int i = 0; i < BLOCK_SIZE; i++)
				sum += (int)top[i];
		}
		if (left)
		{
			sum += BLOCK_SIZE/2;
			for (int i = 0; i < BLOCK_SIZE; i++)
				sum += (int)left[i];
		}

		dc = sum >> (LOG2_BLOCK_SIZE + (top && left ? 1 : 0));
	}
	for (int i = 0; i < BLOCK_SIZE*BLOCK_SIZE; i++)
		result[i] = dc;

	//if (top && left)
	//	dc_filter<BLOCK_SIZE>(top, left, result);

	return 0; // TODO: cost
}

inline uint8_t combine2(uint8_t a, uint8_t b)
{
	return (a + b + 1) >> 1;
}

inline uint8_t combine3(uint8_t a, uint8_t b, uint8_t c)
{
	return (a + 2 * b + c + 2) >> 2;
}

uint32_t ddl_prediction(const uint8_t *top, const uint8_t *left, uint8_t *result)
{
	if (!top) return MAX_COST;

	uint8_t A = top[0], B = top[1], C = top[2], D = top[3];
	uint8_t E = D;

	//result[i * BLOCK_SIZE + j] = combine3(top[i+j], top[i+j+1], top[i+j+2]);

	result[0 * LUMA_BLOCK_SIZE + 0] = combine3(A, B, C);
	result[1 * LUMA_BLOCK_SIZE + 0] = combine3(B, C, D);
	result[2 * LUMA_BLOCK_SIZE + 0] = combine3(C, D, E);
	result[3 * LUMA_BLOCK_SIZE + 0] = combine3(D, E, E);

	result[0 * LUMA_BLOCK_SIZE + 1] = combine3(B, C, D);
	result[1 * LUMA_BLOCK_SIZE + 1] = combine3(C, D, E);
	result[2 * LUMA_BLOCK_SIZE + 1] = combine3(D, E, E);
	result[3 * LUMA_BLOCK_SIZE + 1] = combine3(E, E, E);

	result[0 * LUMA_BLOCK_SIZE + 2] = combine3(C, D, E);
	result[1 * LUMA_BLOCK_SIZE + 2] = combine3(D, E, E);
	result[2 * LUMA_BLOCK_SIZE + 2] = combine3(E, E, E);
	result[3 * LUMA_BLOCK_SIZE + 2] = combine3(E, E, E);

	result[0 * LUMA_BLOCK_SIZE + 3] = combine3(D, E, E);
	result[1 * LUMA_BLOCK_SIZE + 3] = combine3(E, E, E);
	result[2 * LUMA_BLOCK_SIZE + 3] = combine3(E, E, E);
	result[3 * LUMA_BLOCK_SIZE + 3] = combine3(E, E, E);

	return 0;
}

uint32_t ddr_prediction(const uint8_t *top, const uint8_t *left, uint8_t *result)
{
	if (!top || !left) return MAX_COST;

	uint8_t X = top[-1];
	uint8_t A = top[0], B = top[1], C = top[2], D = top[3];
	uint8_t I = left[0], J = left[1], K = left[2], L = left[3];

	result[0 * LUMA_BLOCK_SIZE + 0] = combine3(I, X, A);
	result[1 * LUMA_BLOCK_SIZE + 0] = combine3(X, A, B);
	result[2 * LUMA_BLOCK_SIZE + 0] = combine3(A, B, C);
	result[3 * LUMA_BLOCK_SIZE + 0] = combine3(B, C, D);

	result[0 * LUMA_BLOCK_SIZE + 1] = combine3(J, I, X);
	result[1 * LUMA_BLOCK_SIZE + 1] = combine3(I, X, A);
	result[2 * LUMA_BLOCK_SIZE + 1] = combine3(X, A, B);
	result[3 * LUMA_BLOCK_SIZE + 1] = combine3(A, B, C);

	result[0 * LUMA_BLOCK_SIZE + 2] = combine3(K, J, I);
	result[1 * LUMA_BLOCK_SIZE + 2] = combine3(J, I, X);
	result[2 * LUMA_BLOCK_SIZE + 2] = combine3(I, X, A);
	result[3 * LUMA_BLOCK_SIZE + 2] = combine3(X, A, B);

	result[0 * LUMA_BLOCK_SIZE + 3] = combine3(L, K, J);
	result[1 * LUMA_BLOCK_SIZE + 3] = combine3(K, J, I);
	result[2 * LUMA_BLOCK_SIZE + 3] = combine3(J, I, X);
	result[3 * LUMA_BLOCK_SIZE + 3] = combine3(I, X, A);

	return 0;
}

uint32_t vr_prediction(const uint8_t *top, const uint8_t *left, uint8_t *result)
{
	if (!top || !left) return MAX_COST;

	uint8_t X = top[-1];
	uint8_t A = top[0], B = top[1], C = top[2], D = top[3];
	uint8_t I = left[0], J = left[1], K = left[2];

	result[0 * LUMA_BLOCK_SIZE + 0] = combine2(X, A);
	result[1 * LUMA_BLOCK_SIZE + 0] = combine2(A, B);
	result[2 * LUMA_BLOCK_SIZE + 0] = combine2(B, C);
	result[3 * LUMA_BLOCK_SIZE + 0] = combine2(C, D);

	result[0 * LUMA_BLOCK_SIZE + 1] = combine2(I, X);
	result[1 * LUMA_BLOCK_SIZE + 1] = combine3(X, A, B);
	result[2 * LUMA_BLOCK_SIZE + 1] = combine3(A, B, C);
	result[3 * LUMA_BLOCK_SIZE + 1] = combine3(B, C, D);

	result[0 * LUMA_BLOCK_SIZE + 2] = combine2(J, I);
	result[1 * LUMA_BLOCK_SIZE + 2] = combine3(I, X, A);
	result[2 * LUMA_BLOCK_SIZE + 2] = combine3(X, A, B);
	result[3 * LUMA_BLOCK_SIZE + 2] = combine3(A, B, C);

	result[0 * LUMA_BLOCK_SIZE + 3] = combine2(K, J);
	result[1 * LUMA_BLOCK_SIZE + 3] = combine3(J, I, X);
	result[2 * LUMA_BLOCK_SIZE + 3] = combine3(I, X, A);
	result[3 * LUMA_BLOCK_SIZE + 3] = combine3(X, A, B);

	return 0;
}

uint32_t hd_prediction(const uint8_t *top, const uint8_t *left, uint8_t *result)
{
	if (!top || !left) return MAX_COST;

	uint8_t X = top[-1];
	uint8_t A = top[0], B = top[1], C = top[2], D = top[3];
	uint8_t I = left[0], J = left[1], K = left[2], L = left[3];

	result[0 * LUMA_BLOCK_SIZE + 0] = combine2(X, I);
	result[1 * LUMA_BLOCK_SIZE + 0] = combine2(I, J);
	result[2 * LUMA_BLOCK_SIZE + 0] = combine2(J, K);
	result[3 * LUMA_BLOCK_SIZE + 0] = combine2(K, L);

	result[0 * LUMA_BLOCK_SIZE + 1] = combine3(X, I, J);
	result[1 * LUMA_BLOCK_SIZE + 1] = combine3(I, J, K);
	result[2 * LUMA_BLOCK_SIZE + 1] = combine3(J, K, L);
	result[3 * LUMA_BLOCK_SIZE + 1] = combine3(K, L, L);

	result[0 * LUMA_BLOCK_SIZE + 2] = combine3(I, J, K);
	result[1 * LUMA_BLOCK_SIZE + 2] = combine3(J, K, L);
	result[2 * LUMA_BLOCK_SIZE + 2] = combine3(K, L, L);
	result[3 * LUMA_BLOCK_SIZE + 2] = combine3(L, L, L);

	result[0 * LUMA_BLOCK_SIZE + 3] = combine3(J, K, L);
	result[1 * LUMA_BLOCK_SIZE + 3] = combine3(K, L, L);
	result[2 * LUMA_BLOCK_SIZE + 3] = combine3(L, L, L);
	result[3 * LUMA_BLOCK_SIZE + 3] = combine3(L, L, L);

	return 0;
}

uint32_t vl_prediction(const uint8_t *top, const uint8_t *left, uint8_t *result)
{
	if (!top || !left) return MAX_COST;

	uint8_t X = top[-1];
	uint8_t A = top[0], B = top[1], C = top[2], D = top[3];
	uint8_t E = D;

	result[0 * LUMA_BLOCK_SIZE + 0] = combine2(A, B);
	result[1 * LUMA_BLOCK_SIZE + 0] = combine2(B, C);
	result[2 * LUMA_BLOCK_SIZE + 0] = combine2(C, D);
	result[3 * LUMA_BLOCK_SIZE + 0] = combine2(D, E);

	result[0 * LUMA_BLOCK_SIZE + 1] = combine3(A, B, C);
	result[1 * LUMA_BLOCK_SIZE + 1] = combine3(B, C, D);
	result[2 * LUMA_BLOCK_SIZE + 1] = combine3(C, D, E);
	result[3 * LUMA_BLOCK_SIZE + 1] = combine3(D, E, E);

	result[0 * LUMA_BLOCK_SIZE + 2] = combine3(B, C, D);
	result[1 * LUMA_BLOCK_SIZE + 2] = combine3(C, D, E);
	result[2 * LUMA_BLOCK_SIZE + 2] = combine3(D, E, E);
	result[3 * LUMA_BLOCK_SIZE + 2] = combine3(E, E, E);

	result[0 * LUMA_BLOCK_SIZE + 3] = combine3(C, D, E);
	result[1 * LUMA_BLOCK_SIZE + 3] = combine3(D, E, E);
	result[2 * LUMA_BLOCK_SIZE + 3] = combine3(E, E, E);
	result[3 * LUMA_BLOCK_SIZE + 3] = combine3(E, E, E);

	return 0;
}

uint32_t hu_prediction(const uint8_t *top, const uint8_t *left, uint8_t *result)
{
	if (!top || !left) return MAX_COST;

	uint8_t I = left[0], J = left[1], K = left[2], L = left[3];

	result[0 * LUMA_BLOCK_SIZE + 0] = combine2(I, J);
	result[1 * LUMA_BLOCK_SIZE + 0] = combine2(J, K);
	result[2 * LUMA_BLOCK_SIZE + 0] = combine2(K, L);
	result[3 * LUMA_BLOCK_SIZE + 0] = combine2(L, L);

	result[0 * LUMA_BLOCK_SIZE + 1] = combine3(I, J, K);
	result[1 * LUMA_BLOCK_SIZE + 1] = combine3(J, K, L);
	result[2 * LUMA_BLOCK_SIZE + 1] = combine3(K, L, L);
	result[3 * LUMA_BLOCK_SIZE + 1] = combine3(L, L, L);

	result[0 * LUMA_BLOCK_SIZE + 2] = combine3(J, K, L);
	result[1 * LUMA_BLOCK_SIZE + 2] = combine3(K, L, L);
	result[2 * LUMA_BLOCK_SIZE + 2] = combine3(L, L, L);
	result[3 * LUMA_BLOCK_SIZE + 2] = combine3(L, L, L);

	result[0 * LUMA_BLOCK_SIZE + 3] = combine3(K, L, L);
	result[1 * LUMA_BLOCK_SIZE + 3] = combine3(L, L, L);
	result[2 * LUMA_BLOCK_SIZE + 3] = combine3(L, L, L);
	result[3 * LUMA_BLOCK_SIZE + 3] = combine3(L, L, L);

	return 0;
}

uint32_t plane_prediction_8x8(const uint8_t *top, const uint8_t *left, uint8_t *result)
{
	if (!top || !left) return MAX_COST;

	const int BLOCK_SIZE = 8;

	int H = 0;
	int V = 0;

	for (int i = 0; i < 4; i++)
	{
		H += (i + 1) * (top[4 + i] - top[2 - i]);
		V += (i + 1) * (left[4 + i] - left[2 - i]);
	}

	int a = 16 * (top[BLOCK_SIZE-1] + left[BLOCK_SIZE-1]);
	int b = (17 * H + 16) >> 5;
	int c = (17 * V + 16) >> 5;

	for (int x = 0; x < BLOCK_SIZE; x++)
	{
		for (int y = 0; y < BLOCK_SIZE; y++)
		{
			int val = (a + b * (x - 3) + c * (y - 3) + 16) >> 5;
			result[x * BLOCK_SIZE + y] = clamp8(val);
		}
	}

	return 0;
}

uint32_t plane_prediction_16x16(const uint8_t *top, const uint8_t *left, uint8_t *result)
{
	if (!top || !left) return MAX_COST;

	const int BLOCK_SIZE = 16;

	int H = 0;
	int V = 0;

	for (int i = 0; i < 8; i++)
	{
		H += (i + 1) * (top[8 + i] - top[6 - i]);
		V += (i + 1) * (left[8 + i] - left[6 - i]);
	}

	int a = 16 * (top[BLOCK_SIZE-1] + left[BLOCK_SIZE-1]);
	int b = (5 * H + 32) >> 6;
	int c = (5 * V + 32) >> 6;

	for (int x = 0; x < BLOCK_SIZE; x++)
	{
		for (int y = 0; y < BLOCK_SIZE; y++)
		{
			int val = (a + b * (x - 7) + c * (y - 7) + 16) >> 5;
			result[x * BLOCK_SIZE + y] = clamp8(val);
		}
	}

	return 0;
}

/// BLOCK PREDICTION

uint32_t predict_luma_4x4(const luma_block_t &original, intra_mode_t mode,
		const uint8_t *top, const uint8_t *left, luma_block_t *result)
{
	const int LOG2_BLOCK_SIZE = 2;

	uint32_t cost = 0;

	switch (mode)
	{
		case INTRA_VERTICAL:
			cost = vertical_prediction<LUMA_BLOCK_SIZE>(top, left, result->Y);
			break;

		case INTRA_HORIZONTAL:
			cost = horizontal_prediction<LUMA_BLOCK_SIZE>(top, left, result->Y);
			break;

		case INTRA_DC:
			cost = dc_prediction<LUMA_BLOCK_SIZE, LOG2_BLOCK_SIZE>(top, left, result->Y);
			break;

		case INTRA_DIAGONAL_DOWN_LEFT:
			cost = ddl_prediction(top, left, result->Y);
			break;

		case INTRA_DIAGONAL_DOWN_RIGHT:
			cost = ddr_prediction(top, left, result->Y);
			break;

		case INTRA_VERTICAL_RIGHT:
			cost = vr_prediction(top, left, result->Y);
			break;

		case INTRA_HORIZONTAL_DOWN:
			cost = hd_prediction(top, left, result->Y);
			break;

		case INTRA_VERTICAL_LEFT:
			cost = vl_prediction(top, left, result->Y);
			break;

		case INTRA_HORIZONTAL_UP:
			cost = hu_prediction(top, left, result->Y);
			break;
	}

	return cost == 0 ? eval_cost(original, *result) : cost;
}

uint32_t predict_luma_16x16(const uint8_t *original, intra_mode_t mode,
		const uint8_t *top, const uint8_t *left, uint8_t *result)
{
	const int LOG2_BLOCK_SIZE = 4;
	const int BLOCK_SIZE = MB_SIZE;

	uint32_t cost = 0;
	uint8_t tmp_result[BLOCK_SIZE*BLOCK_SIZE];

	switch (mode)
	{
		case INTRA_VERTICAL:
			cost = vertical_prediction<BLOCK_SIZE>(top, left, tmp_result);
			break;

		case INTRA_HORIZONTAL:
			cost = horizontal_prediction<BLOCK_SIZE>(top, left, tmp_result);
			break;

		case INTRA_DC:
			cost = dc_prediction<BLOCK_SIZE, LOG2_BLOCK_SIZE>(top, left, tmp_result);
			break;

		case INTRA_PLANE:
			cost = plane_prediction_16x16(top, left, tmp_result);
			break;
	}

	// regorganize the predicted data into blocks to match layout of original
	luma_block_t *blocks = (luma_block_t*)result;
	for (int block_i = 0; block_i < LUMA_BLOCK_COUNT; block_i++)
	{
		for (int block_j = 0; block_j < LUMA_BLOCK_COUNT; block_j++)
		{
			luma_block_t &block = blocks[block_i * LUMA_BLOCK_COUNT + block_j];

			for (int i = 0; i < LUMA_BLOCK_SIZE; i++)
			{
				for (int j = 0; j < LUMA_BLOCK_SIZE; j++)
				{
					int src_i = (block_i * LUMA_BLOCK_SIZE + i);
					int src_j = (block_j * LUMA_BLOCK_SIZE + j);
					int src_idx = src_i * BLOCK_SIZE + src_j;

					block.Y[i * LUMA_BLOCK_SIZE + j] = tmp_result[src_idx];
				}
			}
		}
	}

	return cost == 0 ? eval_cost<BLOCK_SIZE>(original, result) : cost;
}

uint32_t predict_chroma(const chroma_block_t &original, intra_mode_t mode,
		const uint8_t *top, const uint8_t *left, chroma_block_t *result)
{
	const int LOG2_BLOCK_SIZE = 3;

	uint32_t cost = 0;

	switch (mode)
	{
		case INTRA_VERTICAL:
			cost = vertical_prediction<CHROMA_BLOCK_SIZE>(top, left, result->C);
			break;

		case INTRA_HORIZONTAL:
			cost = horizontal_prediction<CHROMA_BLOCK_SIZE>(top, left, result->C);
			break;

		case INTRA_DC:
			cost = dc_prediction<CHROMA_BLOCK_SIZE, LOG2_BLOCK_SIZE>(top, left, result->C);
			break;

		case INTRA_PLANE:
			cost = plane_prediction_8x8(top, left, result->C);
			break;
	}

	return cost == 0 ? eval_cost(original, *result) : cost;
}

/// MODE SELECTION

uint32_t find_mode_luma_4x4(const luma_block_t &original,
		const uint8_t *top, const uint8_t *left,
		luma_block_t *predicted, intra_mode_t *pred_mode)
{
	uint32_t pred_cost = MAX_COST;

	for (int i = 0; i < INTRA_MODE_COUNT; i++)
	{
		if ((LUMA_4x4_INTRA_MODES & (1 << i)) == 0)
			continue;

		uint32_t cost;
		auto mode = (intra_mode_t)i;
		luma_block_t prediction;

		cost = predict_luma_4x4(original, mode, top, left, &prediction);

		if (cost < pred_cost)
		{
			pred_cost = cost;

			*pred_mode = mode;
			memcpy((void*)predicted, (void*)&prediction, sizeof(luma_block_t));
		}
	}

	return pred_cost;
}

uint32_t find_mode_luma_16x16(const uint8_t *original,
		const uint8_t *top, const uint8_t *left,
		uint8_t *predicted, intra_mode_t *pred_mode)
{
	uint32_t pred_cost = MAX_COST;

	for (int i = 0; i < INTRA_MODE_COUNT; i++)
	{
		if ((LUMA_16x16_INTRA_MODES & (1 << i)) == 0)
			continue;

		uint32_t cost;
		auto mode = (intra_mode_t)i;
		uint8_t prediction[MB_SIZE * MB_SIZE];

		cost = predict_luma_16x16(original, mode, top, left, prediction);

		if (cost < pred_cost)
		{
			pred_cost = cost;

			*pred_mode = mode;
			memcpy((void*)predicted, (void*)prediction, sizeof(prediction));
		}
	}

	return pred_cost;
}

uint32_t find_mode_chroma(const macroblock_t &original,
		const neighbour_t &top, const neighbour_t &left,
		macroblock_t *predicted, intra_mode_t *predicted_mode)
{
	uint32_t pred_cost = MAX_COST;

	for (int i = 0; i < INTRA_MODE_COUNT; i++)
	{
		if ((CHROMA_INTRA_MODES & (1 << i)) == 0)
			continue;

		auto mode = (intra_mode_t)i;
		chroma_block_t prediction_u, prediction_v;

		uint32_t cost = 0;
		cost += predict_chroma(original.chroma_u, mode,
				top.get_chroma_u(), left.get_chroma_u(), &prediction_u);
		cost += predict_chroma(original.chroma_v, mode,
				top.get_chroma_v(), left.get_chroma_v(), &prediction_v);

		if (cost < pred_cost)
		{
			pred_cost = cost;
			*predicted_mode = mode;

			memcpy((void*)&predicted->chroma_u, (void*)&prediction_u, sizeof(chroma_block_t));
			memcpy((void*)&predicted->chroma_v, (void*)&prediction_v, sizeof(chroma_block_t));
		}
	}

	return pred_cost;
}
