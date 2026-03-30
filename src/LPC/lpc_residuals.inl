#include "lpc.h"

// 16 for luma and 4 for each chroma plane
static const int NUM_RESIDUALS = 16 + 2 * 4;

namespace cst
{
	// Quantization

	// a^2 = 1
	// ab/2 = 2
	// b^2/4 = 3
	const int coef_idx[16] = { 0, 1, 0, 1, 1, 2, 1, 2, 0, 1, 0, 1, 1, 2, 1, 2 };

	// MF = (PF/Qstep) >> q_bits
	const int mf_coef[6][3] = {
		{ 13107,8066,5243 }, { 11916,7490,4660 },
		{ 10082,6554,4194 }, { 9362,5825,3647 },
		{ 8192,5243,3355 }, { 7282,4559,2893 } };

	// V = Qstep*PF*64
	const int v_scale[6][3] = {
		{10,13,16}, {11,14,18}, {13,16,20}, {14,18,23}, {16,20,25}, {18,23,29} };

	const uint8_t qp_mod6[52] = { 0,1,2,3,4,5,0,1,2,3,4,5,0,1,2,3,4,5,0,1,2,3,4,5,
		0,1,2,3,4,5,0,1,2,3,4,5,0,1,2,3,4,5,0,1,2,3,4,5,0,1,2,3 };
	const uint8_t qp_div6[52] = { 0,0,0,0,0,0,1,1,1,1,1,1,2,2,2,2,2,2,3,3,3,3,3,3,
		4,4,4,4,4,4,5,5,5,5,5,5,6,6,6,6,6,6,7,7,7,7,7,7,8,8,8,8 };

	const uint8_t qp_chroma[22] = {
		29,30,31,32,32,33,34,34,35,35,36,36,37,37,37,38,38,38,39,39,39,39 };

	uint8_t compute_qp_chroma(uint8_t qp)
	{
		qp = min(max(0, (int)qp), 51);
		if (qp >= 30)
			return qp_chroma[qp - 30];
		return qp;
	}

	// Zigzag
	const int zigzag[16] = {
		0,  1,  4,  8,
		5,  2,  3,  6,
		9, 12, 13, 10,
		7, 11, 14, 15
	};

	// Cost threshold - ignore residuals below the threshold
	const uint32_t cost_threshold[52] = {
		1,    3,   3,   3,   3,   3,   4,   4,   5,   5,
		6,    7,   8,   9,  10,  11,  13,  14,  16,  18,
		20,  22,  26,  28,  32,  36,  40,  44,  52,  56,
		46,  72,  80,  88, 104, 112, 128, 144, 160, 176,
		208, 224, 256, 288, 320, 352, 416, 448, 512, 576,
		604, 704,
	};
}

struct residual_t
{
	int16_t val[4*4];

	// Quantization

	void quantize(uint8_t qp)
	{
		using namespace cst;

		uint8_t mod6 = qp_mod6[qp];
		int qbits = 15 + qp_div6[qp];
		int f = (1 << qbits) / 3;

		for (int i = 0; i < 16; i++)
		{
			if (val[i] > 0)
				val[i] = (int(val[i]) * mf_coef[mod6][coef_idx[i]] + f) >> qbits;
			else
				val[i] = -((int(-val[i]) * mf_coef[mod6][coef_idx[i]] + f) >> qbits);
		}
	}

	void inverse_quantize(uint8_t qp)
	{
		using namespace cst;

		uint8_t mod6 = qp_mod6[qp];
		int div6 = qp_div6[qp];

		for (int i = 0; i < 16; i++)
			val[i] = (int(val[i]) * v_scale[mod6][coef_idx[i]]) << div6;
	}

	// Integer transform

	void transform_and_quantize(uint8_t qp)
	{
		int tmp[4*4];

		// Horizontal pass
		for (int i = 0; i < 4; i++)
		{
			int s0 = val[i*4+0] + val[i*4+3];
			int s1 = val[i*4+1] + val[i*4+2];
			int s2 = val[i*4+1] - val[i*4+2];
			int s3 = val[i*4+0] - val[i*4+3];

			tmp[i*4+0] = s0 + s1;
			tmp[i*4+1] = (s3<<1) + s2;
			tmp[i*4+2] = s0 - s1;
			tmp[i*4+3] = s3 - (s2<<1);
		}

		// Vertical pass
		for (int i = 0; i < 4; i++)
		{
			int s0 = tmp[0*4+i] + tmp[3*4+i];
			int s1 = tmp[1*4+i] + tmp[2*4+i];
			int s2 = tmp[1*4+i] - tmp[2*4+i];
			int s3 = tmp[0*4+i] - tmp[3*4+i];

			val[0*4+i] = s0 + s1;
			val[1*4+i] = (s3<<1) + s2;
			val[2*4+i] = s0 - s1;
			val[3*4+i] = s3 - (s2<<1);
		}

		quantize(qp);
		reorder_zigzag();
	}

	void inverse_quantize_and_transform(uint8_t qp)
	{
		int tmp[4*4];

		reorder_linear();
		inverse_quantize(qp);

		// Horizontal pass
		for (int i = 0; i < 4; i++)
		{
			int s0 = val[i*4+0] + val[i*4+2];
			int s1 = val[i*4+0] - val[i*4+2];
			int s2 = (val[i*4+1]>>1) - val[i*4+3];
			int s3 = val[i*4+1] + (val[i*4+3]>>1);

			tmp[i*4+0] = s0 + s3;
			tmp[i*4+1] = s1 + s2;
			tmp[i*4+2] = s1 - s2;
			tmp[i*4+3] = s0 - s3;
		}

		// Vertical pass
		for (int i = 0; i < 4; i++)
		{
			int s0 = tmp[0*4+i] + tmp[2*4+i];
			int s1 = tmp[0*4+i] - tmp[2*4+i];
			int s2 = (tmp[1*4+i]>>1) - tmp[3*4+i];
			int s3 = tmp[1*4+i] + (tmp[3*4+i]>>1);

			val[0*4+i] = (s0 + s3 + 32) >> 6;
			val[1*4+i] = (s1 + s2 + 32) >> 6;
			val[2*4+i] = (s1 - s2 + 32) >> 6;
			val[3*4+i] = (s0 - s3 + 32) >> 6;
		}
	}

	// Zigzag

	// TODO: it's possible to implement that much more efficiently
	void reorder_zigzag()
	{
		residual_t reordered;
		for (int i = 0; i < 16; i++)
			reordered.val[i] = val[cst::zigzag[i]];

		memcpy(val, reordered.val, sizeof(val));
	}

	void reorder_linear()
	{
		residual_t reordered;
		for (int i = 0; i < 16; i++)
			reordered.val[cst::zigzag[i]] = val[i];

		memcpy(val, reordered.val, sizeof(val));
	}

	LPC_DEBUG_ONLY(void print(const char *msg = NULL) const);
};

void build_residuals(const macroblock_t &mb, const predicted_macroblock_t &predicted,
		residual_t *residuals)
{
	int idx = 0; // TODO: investigate different orderings

	uint8_t qp = predicted.qp;
	uint8_t qpc = predicted.qpc;

	// Luma residuals
	for (int i = 0; i < 4*4; i++)
	{
		for (int j = 0; j < 4*4; j++)
		{
			int orig = mb.luma[i].Y[j];
			int pred = predicted.mb.luma[i].Y[j];
			residuals[idx].val[j] = orig - pred;
		}

		residuals[idx].transform_and_quantize(qp);
		idx++;
	}

	// Chroma residuals
	for (int block_i = 0; block_i < 8; block_i += 4)
	{
		for (int block_j = 0; block_j < 8; block_j += 4)
		{
			int block_offset = block_i * 8 + block_j;

			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					int c_idx = block_offset + i * 8 + j;
					int r_idx = i * 4 + j;

					int orig_u = mb.chroma_u.C[c_idx];
					int pred_u = predicted.mb.chroma_u.C[c_idx];
					residuals[idx+0].val[r_idx] = orig_u - pred_u;

					int orig_v = mb.chroma_v.C[c_idx];
					int pred_v = predicted.mb.chroma_v.C[c_idx];
					residuals[idx+1].val[r_idx] = orig_v - pred_v;
				}
			}

			residuals[idx+0].transform_and_quantize(qpc);
			residuals[idx+1].transform_and_quantize(qpc);
			idx += 2;
		}
	}

	LPC_ASSERT(idx == NUM_RESIDUALS);
}

void build_macroblock(const predicted_macroblock_t &predicted, residual_t *residuals,
		macroblock_t *mb)
{
	int idx = 0;

	uint8_t qp = predicted.qp;
	uint8_t qpc = predicted.qpc;

	// Luma residuals
	for (int i = 0; i < 4*4; i++)
	{
		residuals[idx].inverse_quantize_and_transform(qp);

		for (int j = 0; j < 4*4; j++)
		{
			int residual = residuals[idx].val[j];
			int pred = predicted.mb.luma[i].Y[j];
			mb->luma[i].Y[j] = clamp8(residual + pred);
		}

		idx++;
	}

	// Chroma residuals
	for (int block_i = 0; block_i < 8; block_i += 4)
	{
		for (int block_j = 0; block_j < 8; block_j += 4)
		{
			residuals[idx+0].inverse_quantize_and_transform(qpc);
			residuals[idx+1].inverse_quantize_and_transform(qpc);

			int block_offset = block_i * 8 + block_j;

			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					int c_idx = block_offset + i * 8 + j;
					int r_idx = i * 4 + j;

					int residual_u = residuals[idx+0].val[r_idx];
					int pred_u = predicted.mb.chroma_u.C[c_idx];
					mb->chroma_u.C[c_idx] = clamp8(residual_u + pred_u);

					int residual_v = residuals[idx+1].val[r_idx];
					int pred_v = predicted.mb.chroma_v.C[c_idx];
					mb->chroma_v.C[c_idx] = clamp8(residual_v + pred_v);
				}
			}

			idx += 2;
		}
	}

	LPC_ASSERT(idx == NUM_RESIDUALS);
}
