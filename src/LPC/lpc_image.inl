#include "lpc.h"

struct rgb_t
{
	uint8_t r;
	uint8_t g;
	uint8_t b;

#if LPC_USE_YCBCR
	uint8_t get_Y()
	{
		return clamp8((77*(int)r + 150*(int)g + 29*(int)b) >> 8);
	}

	uint8_t get_Cb()
	{
		return clamp8(((-43*(int)r - 85*(int)g + 128*(int)b) >> 8) + 128);
	}

	uint8_t get_Cr()
	{
		return clamp8(((128*(int)r - 107*(int)g - 21*(int)b) >> 8) + 128);
	}
#else
	uint8_t get_Y() { return r; }
	uint8_t get_Cb() { return g; }
	uint8_t get_Cr() { return b; }
#endif

	void from_YCbCr(uint8_t Y, uint8_t Cb, uint8_t Cr)
	{
#if LPC_USE_YCBCR
		int y = Y;
		int cb = (int)Cb - 128;
		int cr = (int)Cr - 128;

		r = clamp8(y + ((359 * cr) >> 8));
		g = clamp8(y - (( 88 * cb + 183 * cr) >> 8));
		b = clamp8(y + ((454 * cb) >> 8));
#else
		r = Y;
		g = Cb;
		b = Cr;
#endif
	}
};

void macroblock_t::from_rgb(const uint8_t *rgb, int width, int height, int x, int y)
{
	rgb_t *pixels = (rgb_t*)rgb;
	int base_x = x * MB_SIZE;
	int base_y = y * MB_SIZE;

	for (int i = 0; i < CHROMA_BLOCK_SIZE; i++)
	{
		for (int j = 0; j < CHROMA_BLOCK_SIZE; j++)
		{
			int pos_x = base_x + i * 2;
			int pos_y = base_y + j * 2;

			int x0 = min(pos_x+0, width-1);
			int y0 = min(pos_y+0, height-1);

			int x1 = min(pos_x+1, width-1);
			int y1 = min(pos_y+1, height-1);

			rgb_t &p00 = pixels[x0 + y0 * width];
			rgb_t &p10 = pixels[x1 + y0 * width];
			rgb_t &p01 = pixels[x0 + y1 * width];
			rgb_t &p11 = pixels[x1 + y1 * width];

			// Luma
			int block_i = (i * 2) / LUMA_BLOCK_SIZE;
			int block_j = (j * 2) / LUMA_BLOCK_SIZE;
			auto &luma_block = luma[block_i * LUMA_BLOCK_COUNT + block_j];
			int luma_i = (i * 2) - (block_i * LUMA_BLOCK_SIZE);
			int luma_j = (j * 2) - (block_j * LUMA_BLOCK_SIZE);
			luma_block.Y[(luma_i+0) * LUMA_BLOCK_SIZE + (luma_j+0)] = p00.get_Y();
			luma_block.Y[(luma_i+1) * LUMA_BLOCK_SIZE + (luma_j+0)] = p10.get_Y();
			luma_block.Y[(luma_i+0) * LUMA_BLOCK_SIZE + (luma_j+1)] = p01.get_Y();
			luma_block.Y[(luma_i+1) * LUMA_BLOCK_SIZE + (luma_j+1)] = p11.get_Y();

			// Cb Cr
			rgb_t avg;
			avg.r = ((int)p00.r + (int)p10.r + (int)p01.r + (int)p11.r) >> 2;
			avg.g = ((int)p00.g + (int)p10.g + (int)p01.g + (int)p11.g) >> 2;
			avg.b = ((int)p00.b + (int)p10.b + (int)p01.b + (int)p11.b) >> 2;

			chroma_u.C[i * CHROMA_BLOCK_SIZE + j] = avg.get_Cb();
			chroma_v.C[i * CHROMA_BLOCK_SIZE + j] = avg.get_Cr();
		}
	}
}

void macroblock_t::to_rgb(uint8_t *rgb, int width, int height, int x, int y)
{
	rgb_t *pixels = (rgb_t*)rgb;
	int base_x = x * MB_SIZE;
	int base_y = y * MB_SIZE;

	for (int block_i = 0; block_i < LUMA_BLOCK_COUNT; block_i++)
	{
		for (int block_j = 0; block_j < LUMA_BLOCK_COUNT; block_j++)
		{
			auto &block = luma[block_i * LUMA_BLOCK_COUNT + block_j];

			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					int pos_block_x = block_i * 4 + i;
					int pos_block_y = block_j * 4 + j;

					int pos_x = base_x + pos_block_x;
					int pos_y = base_y + pos_block_y;

					if (pos_x >= width || pos_y >= height)
						continue;

					uint8_t Y = block.Y[i * 4 + j];
					uint8_t Cb = chroma_u.C[(pos_block_x/2) * 8 + (pos_block_y/2)];
					uint8_t Cr = chroma_v.C[(pos_block_x/2) * 8 + (pos_block_y/2)];

					pixels[pos_x + pos_y * width].from_YCbCr(Y, Cb, Cr);
				}
			}
		}
	}
}
