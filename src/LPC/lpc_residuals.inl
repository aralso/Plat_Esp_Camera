#include "lpc.h"

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

	// Table 8-13
	const uint8_t qp_chroma[22] = {
		29,30,31,32,32,33,34,34,35,35,36,36,37,37,37,38,38,38,39,39,39,39 };

	uint8_t compute_qp_chroma(uint8_t qp)
	{
		qp = min(max(0, (int)qp), QP_MAX);
		if (qp >= 30)
			return qp_chroma[qp - 30];
		return qp;
	}
}

struct residual_t
{
	int16_t val[4*4];

	/* CORE TRANSFORM */

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

	void transform()
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
	}

	void inverse_transform()
	{
		int tmp[4*4];

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

	/* MB_TYPE_16x16 DC TRANFORM */

	// Quantization

	void dc_quantize(uint8_t qp)
	{
		using namespace cst;

		int mf = mf_coef[qp_mod6[qp]][0];
		int qbits = 15 + qp_div6[qp] + 1;
		int f = 1 << (qbits - 1);

		for (int i = 0; i < 16; i++)
		{
			if (val[i] > 0)
				val[i] = (int(val[i]) * mf + f) >> qbits;
			else
				val[i] = -((int(-val[i]) * mf + f) >> qbits);
		}
	}

	void dc_inverse_quantize(uint8_t qp)
	{
		using namespace cst;

		int v = v_scale[qp_mod6[qp]][0];

		if (qp >= 12)
		{
			int shift = qp_div6[qp] - 2;

			for (int i = 0; i < 16; i++)
				val[i] = (val[i] * v) << shift;
		}
		else
		{
			int shift = 2 - qp_div6[qp];
			int f = 1 << (shift - 1);

			for (int i = 0; i < 16; i++)
				val[i] = (val[i] * v + f) >> shift;
		}
	}

	// Integer transform

	int dc_div2(int value)
	{
		return (value + (value > 0 ? 1 : -1)) / 2;
	}

	void dc_transform()
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
			tmp[i*4+1] = s3 + s2;
			tmp[i*4+2] = s0 - s1;
			tmp[i*4+3] = s3 - s2;
		}

		// Vertical pass
		for (int i = 0; i < 4; i++)
		{
			int s0 = tmp[0*4+i] + tmp[3*4+i];
			int s1 = tmp[1*4+i] + tmp[2*4+i];
			int s2 = tmp[1*4+i] - tmp[2*4+i];
			int s3 = tmp[0*4+i] - tmp[3*4+i];

			val[0*4+i] = dc_div2(s0 + s1);
			val[1*4+i] = dc_div2(s3 + s2);
			val[2*4+i] = dc_div2(s0 - s1);
			val[3*4+i] = dc_div2(s3 - s2);
		}
	}

	void dc_inverse_transform()
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
			tmp[i*4+1] = s3 + s2;
			tmp[i*4+2] = s0 - s1;
			tmp[i*4+3] = s3 - s2;
		}

		// Vertical pass
		for (int i = 0; i < 4; i++)
		{
			int s0 = tmp[0*4+i] + tmp[3*4+i];
			int s1 = tmp[1*4+i] + tmp[2*4+i];
			int s2 = tmp[1*4+i] - tmp[2*4+i];
			int s3 = tmp[0*4+i] - tmp[3*4+i];

			val[0*4+i] = s0 + s1;
			val[1*4+i] = s3 + s2;
			val[2*4+i] = s0 - s1;
			val[3*4+i] = s3 - s2;
		}
	}

	LPC_DEBUG_ONLY(void print(const char *msg = NULL) const);
};

struct residual2x2_t
{
	int16_t val[2*2];

	// Quantization

	void quantize(uint8_t qp)
	{
		using namespace cst;

		int mf = mf_coef[qp_mod6[qp]][0];
		int qbits = 15 + qp_div6[qp] + 1;
		int f = 1 << (qbits - 1);

		for (int i = 0; i < 4; i++)
		{
			if (val[i] > 0)
				val[i] = (int(val[i]) * mf + f) >> qbits;
			else
				val[i] = -((int(-val[i]) * mf + f) >> qbits);
		}
	}

	void inverse_quantize(uint8_t qp)
	{
		using namespace cst;

		int v = v_scale[qp_mod6[qp]][0];

		if (qp >= 6)
		{
			int shift = qp_div6[qp] - 1;

			for (int i = 0; i < 4; i++)
				val[i] = (val[i] * v) << shift;
		}
		else
		{
			for (int i = 0; i < 4; i++)
				val[i] = (val[i] * v) >> 1;
		}
	}

	// Integer transform

	void transform()
	{
		int s0 = val[0*2+0] + val[0*2+1];
		int s1 = val[1*2+0] + val[1*2+1];
		int s2 = val[0*2+0] - val[0*2+1];
		int s3 = val[1*2+0] - val[1*2+1];

		val[0*2+0] = s0 + s1;
		val[1*2+0] = s0 - s1;
		val[0*2+1] = s2 + s3;
		val[1*2+1] = s2 - s3;
	}

	void inverse_transform()
	{
		transform();
	}

	LPC_DEBUG_ONLY(void print(const char *msg = NULL) const);
};

struct mb_residuals_t
{
	residual_t luma[4*4];
	residual_t luma_dc; // 16x16 only

	residual_t chroma_ac[2][2*2];
	residual2x2_t chroma_dc[2];
};
