#include "lpc.h"

#include <string>

/// PRINT

static const char* to_string(intra_mode_t mode)
{
	switch (mode)
	{
		case INTRA_DC:
			return "INTRA_DC";
		case INTRA_HORIZONTAL:
			return "INTRA_HORIZONTAL";
		case INTRA_VERTICAL:
			return "INTRA_VERTICAL";
		case INTRA_PLANE:
			return "INTRA_PLANE";

		case INTRA_DIAGONAL_DOWN_LEFT:
			return "INTRA_DIAGONAL_DOWN_LEFT";
		case INTRA_DIAGONAL_DOWN_RIGHT:
			return "INTRA_DIAGONAL_DOWN_RIGHT";
		case INTRA_VERTICAL_RIGHT:
			return "INTRA_VERTICAL_RIGHT";
		case INTRA_HORIZONTAL_DOWN:
			return "INTRA_HORIZONTAL_DOWN";
		case INTRA_VERTICAL_LEFT:
			return "INTRA_VERTICAL_LEFT";
		case INTRA_HORIZONTAL_UP:
			return "INTRA_HORIZONTAL_UP";

		default:
			return "(unknown)";
	};
}

void macroblock_t::print(const char *msg, bool do_luma, bool do_chroma) const
{
	if (msg)
		printf("Macroblock (%s)\n", msg);
	if (do_luma)
	{
		printf("Luma\n");
		print_luma();
	}
	if (do_chroma)
	{
		printf("Chroma\n");
		print_chroma();
	}
}

void macroblock_t::print_luma() const
{
	for (int j = 0; j < 16; j++)
	{
		for (int i = 0; i < 16; i++)
		{
			int block_i = i / 4;
			int block_j = j / 4;
			int pos_i = i % 4;
			int pos_j = j % 4;
			printf("%3d ", luma[block_i*4+block_j].Y[pos_i*4+pos_j]);
			if (i % 4 == 3)
				printf("  ");
		}
		printf("\n");
		if (j % 4 == 3)
			printf("\n");
	}
}

void macroblock_t::print_chroma(bool do_chroma_u, bool do_chroma_v) const
{
	for (int j = 0; j < 8; j++)
	{
		for (int i = 0; i < 8; i++)
		{
			if (!do_chroma_u)
				printf("%3d  ", chroma_v.C[i*8+j]);
			else if (!do_chroma_v)
				printf("%3d  ", chroma_u.C[i*8+j]);
			else
				printf("(%3d, %3d)  ", chroma_u.C[i*8+j], chroma_v.C[i*8+j]);
		}
		printf("\n");
	}
	printf("\n");
}

void predicted_macroblock_t::print() const
{
	printf("Luma\n");
	mb.print_luma();
	printf("Chroma (intra mode = %s)\n", to_string(mode_chroma));
	mb.print_chroma();
}

void residual_t::print(const char *msg) const
{
	if (msg)
		printf("Residuals (%s)\n", msg);

	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < 4; i++)
		{
			printf("%d\t", val[i*4+j]);
		}
		printf("\n");
	}
	printf("\n");
}

void residual2x2_t::print(const char *msg) const
{
	if (msg)
		printf("Residuals (%s)\n", msg);

	for (int j = 0; j < 2; j++)
	{
		for (int i = 0; i < 2; i++)
		{
			printf("%d\t", val[i*2+j]);
		}
		printf("\n");
	}
	printf("\n");
}

void neighbour_ctx_t::print(const char *msg) const
{
	if (msg)
		printf("Neighbours (%s)\n", msg);

	// Luma

	// Corner
	if (left.valid) printf("%3d", left.luma[-1]); else printf(" - ");
	printf("/");
	if (top.valid) printf("%d", top.luma[-1]); else printf(" - ");

	for (int i = 0; i < MB_SIZE; i++)
	{
		if (top.valid) printf(" %3d", top.luma[i]); else printf("  - ");
	}
	for (int i = 0; i < MB_SIZE; i++)
	{
		if (left.valid) printf("\n%3d", left.luma[i]); else printf("\n - ");
	}
	printf("\n");

	// Chroma u
	#define PLANE chroma_u

	// Corner
	if (left.valid) printf("%3d", left.PLANE[-1]); else printf(" - ");
	printf("/");
	if (top.valid) printf("%d", top.PLANE[-1]); else printf(" - ");

	for (int i = 0; i < MB_SIZE/2; i++)
	{
		if (top.valid) printf(" %3d", top.PLANE[i]); else printf("  - ");
	}
	for (int i = 0; i < MB_SIZE/2; i++)
	{
		if (left.valid) printf("\n%3d", left.PLANE[i]); else printf("\n - ");
	}
	printf("\n");

	#undef PLANE
}

void neighbour_ctx_t::print_all() const
{
	printf("%s", top.valid ? "v" : "_");
	for (int i = 0; i < MB_SIZE/2 + 1; i++)
	{
		printf(" %3d", top_chroma_u[i]);
	}
	printf("\n");
	printf("%s", left.valid ? "v" : "_");
	for (int i = 0; i < 2 * MB_SIZE/2 + 1; i++)
	{
		printf(" %3d", left_chroma_u[i]);
	}
	printf("\n");
	printf("\n");
}

/// LPC_STATS

#if EXTENDED_STATS
float compute_psnr(float mse)
{
	if (mse == 0)
		return FLT_MAX;

	return 10.0f * log10(float(255 * 255) / mse);
}

float compute_mse(const macroblock_t &original, const macroblock_t &coded)
{
	float mse = 0.0f;
	for (int i = 0; i < LUMA_BLOCK_COUNT*LUMA_BLOCK_COUNT; i++)
	{
		for (int j = 0; j < LUMA_BLOCK_SIZE*LUMA_BLOCK_SIZE; j++)
		{
			float diff = (float)std::abs(original.luma[i].Y[j] - coded.luma[i].Y[j]);
			mse += diff * diff;
		}
	}
	mse /= (LUMA_BLOCK_COUNT*LUMA_BLOCK_SIZE) * (LUMA_BLOCK_COUNT*LUMA_BLOCK_SIZE);
	return mse;
}
#endif

struct countbytes_t : public lpc_stream_out_t
{
	void write(const uint8_t *data, size_t size) override
	{
		total_bytes += size;
	}
	float kb() { flush(); return total_bytes/1000.0f; }
	private:
	size_t total_bytes = 0;
};

lpc_stats_t::lpc_stats_t()
{
	memset(this, 0, sizeof(lpc_stats_t));
	for (int i = 0; i < STAT_COUNT; i++)
		num_block_per_intra_mode[i] = (int*)calloc(INTRA_MODE_COUNT, sizeof(int));
}

lpc_stats_t::~lpc_stats_t()
{
	for (int i = 0; i < STAT_COUNT; i++)
		free(num_block_per_intra_mode[i]);

	if (debug_img)
		delete[] debug_img;
}

// One value per pixel
#define IMG_SIZE_X (num_mb_x * MB_SIZE)
#define IMG_SIZE_Y (num_mb_y * MB_SIZE)

void lpc_stats_t::reset(int mb_x, int mb_y)
{
	this->~lpc_stats_t();
	new (this) lpc_stats_t();

	num_mb_x = mb_x;
	num_mb_y = mb_y;

	debug_img = new uint8_t[IMG_SIZE_X * IMG_SIZE_Y * 3];
}

void lpc_stats_t::set_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
	if (!debug_img)
		return;
		
	int idx = IMG_SIZE_X * y + x;
	debug_img[idx * 3 + 0] = r;
	debug_img[idx * 3 + 1] = g;
	debug_img[idx * 3 + 2] = b;

	has_pixel = true;
}

static void print_stat(const char *prefix, const char *msg, int value, int max_value)
{
	printf("%s %s = %d (%.1f%%)\n", prefix, msg, value, 100.0f*value/(float)max_value);
}

#include <fstream>
bool fmt2bmp(uint8_t *src, uint16_t width, uint16_t height, uint8_t ** out, size_t * out_len);

void lpc_stats_t::print()
{
	if (num_macroblocks == 0)
		return;

	int num_luma_4x4_block = num_mb_luma_4x4 * LUMA_BLOCK_COUNT*LUMA_BLOCK_SIZE;
	int num_luma_16x16_block = num_macroblocks - num_mb_luma_4x4;
	int num_chroma_block = num_macroblocks;
	(void) num_luma_4x4_block;
	(void) num_luma_16x16_block;
	(void) num_chroma_block;

	if (has_pixel)
	{
		uint8_t *bmp;
		size_t bmp_len;
		fmt2bmp(debug_img, IMG_SIZE_X, IMG_SIZE_Y, &bmp, &bmp_len);

		std::ofstream file_out("stats_debug_img.bmp", std::ios::binary);
		for (int i = 0; i < bmp_len; i++)
			file_out << bmp[i];

		free(bmp);
	}

	printf("\n === ENCODING STATISTICS ===\n");
	printf("> MSE = %.3f\n", mse/num_macroblocks);
	
#if EXTENDED_STATS
	printf("> PSNR = %.3f\n", compute_psnr(mse/num_macroblocks));

	printf("\n");
	printf("> QP average = %.3f\n", float(qp_avg) / num_macroblocks);
	printf("> Num macroblocks = %d\n", num_macroblocks);
	print_stat(">", "Num macroblocks using 4x4 luma blocks",
			num_mb_luma_4x4, num_macroblocks);
	print_stat(">", "Num 16x16 luma AC blocks non coded",
			num_block_non_coded[0], num_luma_16x16_block);
	print_stat(">", "Num 16x16 chroma blocks non coded",
			num_block_non_coded[1], num_block_non_coded[3]);
	print_stat(">", "Num 16x16 chroma AC blocks non coded",
			num_block_non_coded[2], num_block_non_coded[3]);

	printf("\n");
	print_stat(">", "Num 4x4 luma blocks using predicted mode",
			num_block_match_pred, num_luma_4x4_block);

	printf("\n");
	printf("> Luma 4x4 modes\n");
	for (int i = 0; i < INTRA_MODE_COUNT; i++)
	{
		if ((LUMA_4x4_INTRA_MODES & (1 << i)) == 0)
			continue;

		print_stat(" -", to_string((intra_mode_t)i),
				num_block_per_intra_mode[STAT_LUMA_4x4][i], num_luma_4x4_block);
	}
	printf("> Luma 16x16 modes\n");
	for (int i = 0; i < INTRA_MODE_COUNT; i++)
	{
		if ((LUMA_16x16_INTRA_MODES & (1 << i)) == 0)
			continue;

		print_stat(" -", to_string((intra_mode_t)i),
				num_block_per_intra_mode[STAT_LUMA_16x16][i], num_luma_16x16_block);
	}

	printf("> Chroma modes\n");
	for (int i = 0; i < INTRA_MODE_COUNT; i++)
	{
		if ((CHROMA_INTRA_MODES & (1 << i)) == 0)
			continue;

		print_stat(" -", to_string((intra_mode_t)i),
				num_block_per_intra_mode[STAT_CHROMA][i], num_chroma_block);
	}

	system("pause");
#endif
}

void stats_add_mb(lpc_stats_t &stats, const predicted_macroblock_t &predicted, const macroblock_t &original)
{
	stats.num_macroblocks++;

	if (predicted.frame_type == FRAME_TYPE_P)
	{
		if (predicted.type == MB_TYPE_P)
			stats.num_block_match_pred++;
		return;
	}

	stats.num_mb_luma_4x4 += (predicted.type == MB_TYPE_I_4x4) ? 1 : 0;

	if (predicted.type == MB_TYPE_I_16x16)
	{
		if (predicted.cbp_luma == 0)
			stats.num_block_non_coded[0]++;
		if (predicted.cbp_chroma == 0)
			stats.num_block_non_coded[1]++;
		if (predicted.cbp_chroma <= 1)
			stats.num_block_non_coded[2]++;
		stats.num_block_non_coded[3]++;
	}

	if (predicted.type == MB_TYPE_I_4x4)
	{
		for (int i = 0; i < LUMA_BLOCK_COUNT*LUMA_BLOCK_COUNT; i++)
			stats.num_block_per_intra_mode[STAT_LUMA_4x4][predicted.modes_luma[i]]++;
	}
	else
	{
		stats.num_block_per_intra_mode[STAT_LUMA_16x16][predicted.mode_luma]++;
	}

	stats.num_block_per_intra_mode[STAT_CHROMA][predicted.mode_chroma]++;

	stats.qp_avg += predicted.qp;
	
#if EXTENDED_STATS
	stats.mse += compute_mse(original, predicted.mb);
#endif
}

/// UNIT TESTS

#if LPC_TESTS
namespace lpc_unit_tests
{
	static void test_quantization()
	{
		int qp = 0;

		// Process for Luma with MB_4x4
		{
			residual_t residuals;
			for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				residuals.val[i * 4 + j] = (i ^ (i + j * 4) + j) ^ (j - i);

			residual_t test;
			memcpy(&test, &residuals, sizeof(residual_t));

			test.transform();
			test.quantize(qp);

			test.inverse_quantize(qp);
			test.inverse_transform();

			for (int i = 0; i < 4 * 4; i++)
				assert(test.val[i] == residuals.val[i]);
		}

		// Process for Luma with MB_16x16
		{
			residual_t residuals;
			for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				residuals.val[i * 4 + j] = (i ^ (i + j * 4) + j) ^ (j - i);

			residual_t test;
			memcpy(&test, &residuals, sizeof(residual_t));

			residual_t residuals2;
			for (int i = 0; i < 4*4; i++)
				residuals2.val[i] = (i << 1) ^ (i + 7);

			// Move DC components to 4x4 block
			test.transform();
			residuals2.val[0] = test.val[0];
			test.quantize(qp);

			residuals2.dc_transform();
			residuals2.dc_quantize(qp);

			// DECODE

			residuals2.dc_inverse_transform();
			residuals2.dc_inverse_quantize(qp);

			// Move back DC components
			test.inverse_quantize(qp);
			test.val[0] = residuals2.val[0];
			test.inverse_transform();

			for (int i = 0; i < 4 * 4; i++)
				assert(test.val[i] == residuals.val[i]);
		}

		// Process for Chroma
		{
			residual_t residuals;
			for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				residuals.val[i * 4 + j] = (i ^ (i + j * 4) + j) ^ (j - i);

			residual_t test;
			memcpy(&test, &residuals, sizeof(residual_t));

			residual2x2_t residuals2;
			residuals2.val[1] = 12;
			residuals2.val[2] = -11;
			residuals2.val[3] = 20;

			// Move DC components to 2x2 block
			test.transform();
			residuals2.val[0] = test.val[0];
			test.quantize(qp);

			residuals2.transform();
			residuals2.quantize(qp);

			// DECODE

			residuals2.inverse_quantize(qp);
			residuals2.inverse_transform();

			// Move back DC components
			test.inverse_quantize(qp);
			test.val[0] = residuals2.val[0];
			test.inverse_transform();

			for (int i = 0; i < 4 * 4; i++)
				assert(test.val[i] == residuals.val[i]);
		}
	}
}

namespace lpc_unit_tests
{

static void do_test_intra_prediction(const neighbour_ctx_t &neighbours,
		bool test_top, const predicted_macroblock_t &top_mb,
		bool test_left, const predicted_macroblock_t &left_mb,
		int expected_dc, int chroma_dc)
{
	uint32_t cost;
	predicted_macroblock_t predicted;

	// Luma 4x4
	if (test_top)
	{
		find_mode_luma_blocks(top_mb.mb,
				neighbours.top.get_luma(), neighbours.left.get_luma(),
				&predicted);

		for (int i = 0; i < LUMA_BLOCK_COUNT; i++)
			assert(neighbours.top.modes_luma[i] == top_mb.modes_luma[i]);
		for (int i = 0; i < 4*4; i++)
			assert(predicted.modes_luma[i] == INTRA_VERTICAL);
	}
	if (test_left)
	{
		find_mode_luma_blocks(left_mb.mb,
				neighbours.top.get_luma(), neighbours.left.get_luma(),
				&predicted);

		for (int i = 0; i < LUMA_BLOCK_COUNT; i++)
			assert(neighbours.left.modes_luma[i] == left_mb.modes_luma[i]);
		for (int i = 0; i < 4*4; i++)
			assert(predicted.modes_luma[i] == INTRA_HORIZONTAL);
	}
	{
		for (int i = 0; i < 4*4; i++)
			predicted.modes_luma[i] = INTRA_DC;
		predict_luma_blocks(predicted.modes_luma,
				neighbours.top.get_luma(), neighbours.left.get_luma(),
				predicted.mb.luma);
		for (int i = 0; i < 4*4; i++)
			for (int j = 0; j < 4*4; j++)
				assert(predicted.mb.luma[i].Y[j] == expected_dc);
	}

	// Luma 16x16
	if (test_top)
	{
		uint8_t orig_luma[16 * 16];
		reorder_luma_16x16_linear(top_mb.mb.luma, orig_luma);

		uint8_t *predicted_luma = (uint8_t*)predicted.mb.luma;
		cost = find_mode_luma_16x16(orig_luma,
				neighbours.top.get_luma(), neighbours.left.get_luma(),
				predicted_luma, &predicted.mode_luma);

		for (int i = 0; i < LUMA_BLOCK_COUNT; i++)
			assert(neighbours.top.modes_luma[i] == top_mb.mode_luma);
		assert(predicted.mode_luma == INTRA_VERTICAL);
	}
	if (test_left)
	{
		uint8_t orig_luma[16 * 16];
		reorder_luma_16x16_linear(left_mb.mb.luma, orig_luma);

		uint8_t *predicted_luma = (uint8_t*)predicted.mb.luma;
		cost = find_mode_luma_16x16(orig_luma,
				neighbours.top.get_luma(), neighbours.left.get_luma(),
				predicted_luma, &predicted.mode_luma);

		for (int i = 0; i < LUMA_BLOCK_COUNT; i++)
			assert(neighbours.left.modes_luma[i] == left_mb.mode_luma);
		assert(predicted.mode_luma == INTRA_HORIZONTAL);
	}
	{
		predicted.mode_luma = INTRA_DC;
		uint8_t *block = (uint8_t*)predicted.mb.luma;
		cost = predict_luma_16x16(block, predicted.mode_luma,
				neighbours.top.get_luma(), neighbours.left.get_luma(),
				block);

		for (int i = 0; i < 4*4; i++)
			for (int j = 0; j < 4*4; j++)
				assert(predicted.mb.luma[i].Y[j] == expected_dc);
	}

	// Chroma
	if (test_top)
	{
		cost = find_mode_chroma(top_mb.mb,
				neighbours.top, neighbours.left,
				&predicted.mb, &predicted.mode_chroma);

		assert(*neighbours.top.mode_chroma == top_mb.mode_chroma);
		assert(predicted.mode_chroma == INTRA_VERTICAL);
	}
	if (test_left)
	{
		cost = find_mode_chroma(left_mb.mb,
				neighbours.top, neighbours.left,
				&predicted.mb, &predicted.mode_chroma);

		assert(*neighbours.left.mode_chroma == left_mb.mode_chroma);
		assert(predicted.mode_chroma == INTRA_HORIZONTAL);
	}
	{
		predict_chroma(predicted.mb.chroma_u, INTRA_DC,
				neighbours.top.get_chroma_u(), neighbours.left.get_chroma_u(),
				&predicted.mb.chroma_u);

		for (int i = 0; i < CHROMA_BLOCK_SIZE*CHROMA_BLOCK_SIZE; i++)
			assert(predicted.mb.chroma_u.C[i] == chroma_dc);
	}
}

static void test_intra_prediction()
{
	neighbour_ctx_t neighbours(2);
	predicted_macroblock_t m1;
	predicted_macroblock_t m3;

	m1.mode_chroma = INTRA_PLANE;
	m3.mode_chroma = INTRA_HORIZONTAL;

	for (int i = 0; i < 4*4; i++)
	{
		m1.modes_luma[i] = INTRA_PLANE;
		m3.modes_luma[i] = INTRA_HORIZONTAL;

		for (int j = 0; j < 4*4; j++)
		{
			bool corner = (j == 0 || j == 3 || j == 12 || j == 15);
			m1.mb.luma[i].Y[j] = corner ? 10 : 20;
			m3.mb.luma[i].Y[j] = corner ? 30 : 40;
		}
	}
	for (int i = 0; i < 8*8; i++)
	{
		bool corner = (i == 0 || i == 7 || i == 56 || i == 63);
		m1.mb.chroma_u.C[i] = corner ? 10 : 20;
		m1.mb.chroma_v.C[i] = corner ? 10 : 20;

		m3.mb.chroma_u.C[i] = corner ? 30 : 40;
		m3.mb.chroma_v.C[i] = corner ? 30 : 40;
	}

	{
		neighbours.set_coords(0, 0);
		neighbours.update_data(m1);

		neighbours.set_coords(0, 1);
		// The image has the following macroblocks
		// | 1 | ? |
		// | X | ? |
		do_test_intra_prediction(neighbours, true, m1, false, m3, 15, 18);
		neighbours.update_data(m3);

	}

	{
		neighbours.set_coords(1, 0);
		// The image has the following macroblocks
		// | 1 | X |
		// | 3 | ? |
		do_test_intra_prediction(neighbours, false, m1, true, m1, 15, 18);
		neighbours.update_data(m1);

		neighbours.set_coords(1, 1);
		// The image has the following macroblocks
		// | 1 | 1 |
		// | 3 | X |
		do_test_intra_prediction(neighbours, true, m1, true, m3, 25, 28);
	}
}

}

namespace lpc_unit_tests
{
	const char *filename = "bin/stream.tst";

	struct filestream_t
	{
		FILE *file;
		filestream_t(const char *mode, const char *path = NULL) { file = fopen(path ? path : filename, mode); }
		~filestream_t() { close(); }
		void close() { fclose(file); }
	};

	struct filestream_in_t : public filestream_t, lpc_stream_in_t
	{
		filestream_in_t(const char *mode, const char *path = NULL) : filestream_t(mode, path) {}
		size_t read(uint8_t *data, size_t size) override { return fread(data, 1, size, file); }
	};
	struct filestream_out_t : public filestream_t, lpc_stream_out_t
	{
		filestream_out_t(const char *mode, const char *path = NULL) : filestream_t(mode, path) {}
		void write(const uint8_t *data, size_t size) override { fwrite(data, 1, size, file); }
	};

	struct inout_stream_t : public lpc_stream_in_t, lpc_stream_out_t
	{
		uint8_t DATA[2048];
		size_t bytes_written = 0;
		size_t bytes_read = 0;

		size_t read(uint8_t *data, size_t size) override
		{
			if (bytes_read + size > bytes_written)
				size = bytes_written - bytes_read;
			if (size != 0)
				memcpy(data, DATA+bytes_read, size);
			bytes_read += size;
			return size;
		}
		void write(const uint8_t *data, size_t size) override { memcpy(DATA+bytes_written, data, size); bytes_written += size; }
	};

	static void test_stream()
	{
		{
			filestream_out_t out("w+");

			for (int i = 0; i < 8; i++)
				out.write_bit(('>' >> (7 - i)) & 1);
			for (int i = 0; i < strlen(filename); i++)
				out.write_byte(filename[i]);
			for (int i = 0; i < 8; i++)
				out.write_bit(('\n' >> (7 - i)) & 1);
			out.write_bytes((const uint8_t *)filename, strlen(filename));
			out.write_byte('\n');

			out.flush();
		}

		{
			filestream_in_t in("r");

			std::string ref = ">" + std::string(filename) + "\n" + std::string(filename) + "\n";
			std::string tmp = "";
			char character = 0;
			for (int i = 0; i < 8; i++)
				character |= in.read_bit() << (7 - i);
			tmp += character;

			do {
				tmp += in.read_byte();
			}
			while (tmp[tmp.size() - 1] != '\n');

			uint8_t tmp2[256];
			size_t byte_count = in.read_bytes(tmp2, 4);
			while (!in.empty())
				tmp2[byte_count++] = in.read_byte();
			for (size_t i = 0; i < byte_count; i++)
				tmp += tmp2[i];

			assert(strcmp(tmp.c_str(), ref.c_str()) == 0);
		}

	}

}

namespace lpc_unit_tests
{
	void test_coder()
	{
		int qp = 0;
		const char *text = "bonjour";
		inout_stream_t stream;

		bool bit1 = true;
		bool bit2 = false;
		uint8_t decoded[256];

		{
			cabac_coder_t encoder((lpc_stream_out_t *)&stream, qp);
			encoder.encode_bytes((const uint8_t *)text, strlen(text), 10);

			encoder.encode_bypass(bit1);
			encoder.encode_bypass(bit1);
			encoder.encode_bypass(bit2);
			encoder.encode_bypass(bit2);
			encoder.encode_bypass(bit1);
			encoder.encode_bypass(bit2);
			encoder.encode_bypass(bit1);
			encoder.encode_bypass(bit2);

			encoder.encode_bytes((const uint8_t *)text, strlen(text), 50);

			encoder.encode_bypass(bit2);
			encoder.encode_bypass(bit2);
			encoder.encode_bypass(bit1);
			encoder.encode_bypass(bit2);
			encoder.encode_bypass(bit2);

			encoder.encode_terminate(bit1);
			stream.flush();
		}

		{
			cabac_coder_t decoder((lpc_stream_in_t *)&stream, qp);

			decoder.decode_bytes(decoded, strlen(text), 10);
			for (int i = 0; i < strlen(text); i++)
				assert((char)decoded[i] == text[i]);

			assert(decoder.decode_bypass() == bit1);
			assert(decoder.decode_bypass() == bit1);
			assert(decoder.decode_bypass() == bit2);
			assert(decoder.decode_bypass() == bit2);
			assert(decoder.decode_bypass() == bit1);
			assert(decoder.decode_bypass() == bit2);
			assert(decoder.decode_bypass() == bit1);
			assert(decoder.decode_bypass() == bit2);

			decoder.decode_bytes(decoded, strlen(text), 50);
			for (int i = 0; i < strlen(text); i++)
				assert((char)decoded[i] == text[i]);

			assert(decoder.decode_bypass() == bit2);
			assert(decoder.decode_bypass() == bit2);
			assert(decoder.decode_bypass() == bit1);
			assert(decoder.decode_bypass() == bit2);
			assert(decoder.decode_bypass() == bit2);

			assert(decoder.decode_terminate() == bit1);
		}
	}

	void test_modes_encoding()
	{
		int qp = 0;
		inout_stream_t stream;

		neighbour_ctx_t neighbours(2);
		neighbours.top.invalidate();
		neighbours.left.invalidate();

		predicted_macroblock_t pred;
		intra_mode_t pred_mode = INTRA_DIAGONAL_DOWN_RIGHT;
		intra_mode_t mode;
		{
			cabac_coder_t encoder((lpc_stream_out_t *)&stream, qp);

			pred.type = MB_TYPE_I_4x4;
			encode_mb_type_i(pred, neighbours, &encoder);

			for (int i = 0; i < 3; i++)
			{
				pred.type = MB_TYPE_I_16x16;
				pred.mode_luma = INTRA_PLANE;
				pred.cbp_luma = 15;
				pred.cbp_chroma = i;
				encode_mb_type_i(pred, neighbours, &encoder);
			}

			encode_luma_mode(INTRA_DIAGONAL_DOWN_RIGHT, pred_mode, &encoder);
			encode_luma_mode(INTRA_DIAGONAL_DOWN_LEFT, pred_mode, &encoder);
			encode_chroma_mode(INTRA_PLANE, neighbours, &encoder);

			encoder.encode_terminate(1);
			stream.flush();
		}

		{
			cabac_coder_t decoder((lpc_stream_in_t *)&stream, qp);

			decode_mb_type_i(&pred, neighbours, &decoder);
			assert(pred.type == MB_TYPE_I_4x4);

			for (int i = 0; i < 3; i++)
			{
				decode_mb_type_i(&pred, neighbours, &decoder);
				assert(pred.type == MB_TYPE_I_16x16);
				assert(pred.mode_luma == INTRA_PLANE);
				assert(pred.cbp_luma == 15);
				assert(pred.cbp_chroma == i);
			}

			decode_luma_mode(&mode, pred_mode, &decoder);
			assert(mode == INTRA_DIAGONAL_DOWN_RIGHT);
			decode_luma_mode(&mode, pred_mode, &decoder);
			assert(mode == INTRA_DIAGONAL_DOWN_LEFT);
			decode_chroma_mode(&mode, neighbours, &decoder);
			assert(mode == INTRA_PLANE);
		}
	}

	void test_residual_encoding()
	{
		int qp = 0;
		inout_stream_t stream;
		residual_t residuals, decoded;
		residual2x2_t residuals2, decoded2;

		for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			residuals.val[i*4+j] = (i ^ (i + j * 4) + j) ^ (j - i);
		residuals.val[15] = residuals.val[14] = 0;

		for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			residuals2.val[i*2+j] = (i ^ (i + j * 4) + j) ^ (j - i);
		residuals2.val[3] = 0;

		neighbour_ctx_t neighbours(1);

		{
			cabac_coder_t encoder((lpc_stream_out_t*)&stream, qp);
			for (int i = 10; i < 20; i++)
				encode_coeff_abs(i, LUMA_BLOCK, 0, 0, &encoder);
			for (int i = -5; i < 5; i++)
				encode_qp_delta(i, neighbours, &encoder);
			encode_residual_block(LUMA_BLOCK, residuals.val, 16, &encoder);
			encode_residual_block(LUMA_AC_BLOCK, residuals.val, 16, &encoder);
			encode_residual_block(CHROMA_DC_BLOCK, residuals2.val, 4, &encoder);
			encode_residual_block(CHROMA_AC_BLOCK, residuals.val, 16, &encoder);

			encoder.encode_terminate(1);
			stream.flush();
		}

		{
			cabac_coder_t decoder((lpc_stream_in_t *)&stream, qp);
			for (int i = 10; i < 20; i++)
				assert(decode_coeff_abs(LUMA_BLOCK, 0, 0, &decoder) == i);
			for (int i = -5; i < 5; i++)
			{
				int8_t delta;
				decode_qp_delta(&delta, neighbours, &decoder);
				assert(delta == i);
			}
			decode_residual_block(LUMA_BLOCK, decoded.val, 16, &decoder);
			for (int i = 0; i < 16; i++)
				assert(decoded.val[i] == residuals.val[i]);
			decode_residual_block(LUMA_AC_BLOCK, decoded.val, 16, &decoder);
			for (int i = 1; i < 16; i++)
				assert(decoded.val[i] == residuals.val[i]);
			decode_residual_block(CHROMA_DC_BLOCK, decoded2.val, 4, &decoder);
			for (int i = 0; i < 4; i++)
				assert(decoded2.val[i] == residuals2.val[i]);
			decode_residual_block(CHROMA_AC_BLOCK, decoded.val, 16, &decoder);
			for (int i = 1; i < 16; i++)
				assert(decoded.val[i] == residuals.val[i]);
		}
	}

	uint8_t *create_img(int channel)
	{
		uint8_t *img_rgb = (uint8_t*)malloc(MB_SIZE * MB_SIZE * 3);

		for (int i = 0; i < MB_SIZE; i++)
		for (int j = 0; j < MB_SIZE; j++)
		{
			int idx = (i+j*MB_SIZE)*3;
			img_rgb[idx+0] = 1;
			img_rgb[idx+1] = 1;
			img_rgb[idx+2] = 1;

			// Gradient in the given channel
			img_rgb[idx+channel] = (i * 255) / MB_SIZE;
		}

		return img_rgb;
	}

	void test_mb_encoding()
	{
		int qp = 0;
		inout_stream_t stream;

		predicted_macroblock_t pred, pred_decoded;
		pred.frame_type = FRAME_TYPE_I;
		pred.qp = qp;
		pred.qp_chroma_offset = 0;
		memcpy(&pred_decoded, &pred, sizeof(pred));

		neighbour_ctx_t neighbours(1);
		neighbours.set_coords(0, 0);

		uint8_t *img_rgb = create_img(1);

		macroblock_t mb;
		mb_residuals_t residuals, resid_decoded;
		mb.from_rgb(img_rgb, MB_SIZE, MB_SIZE, 0, 0);

		{
			cabac_coder_t encoder((lpc_stream_out_t*)&stream, qp);

			pred.set_qp_delta(0);
			pred.select_intra_modes(mb, neighbours);
			pred.build_residuals(mb, &residuals);
			pred.compute_cbp_flags(residuals);
			pred.encode_mb(neighbours, residuals, &encoder);

			encoder.encode_terminate(1);
			stream.flush();
		}

		{
			cabac_coder_t decoder((lpc_stream_in_t *)&stream, qp);

			pred_decoded.decode_mb(neighbours, &resid_decoded, &decoder);

			assert(pred_decoded.type == pred.type);
			assert(pred_decoded.mode_chroma == pred.mode_chroma);
			assert(pred_decoded.cbp_chroma == pred.cbp_chroma);

			#if LPC_ADAPTIVE_QP
			assert(pred_decoded.qp_delta == pred.qp_delta);
			#endif

			if (pred.type == MB_TYPE_I_16x16)
			{
				assert(pred_decoded.cbp_luma == pred.cbp_luma);
				assert(pred_decoded.mode_luma == pred.mode_luma);
				for (int i = 0; i < 4*4; i++)
				{
					assert(resid_decoded.luma_dc.val[i] == residuals.luma_dc.val[i]);
					for (int j = 1; j < 4 * 4; j++)
						assert(resid_decoded.luma[i].val[j] == residuals.luma[i].val[j]);
				}
			}
			else
			{
				for (int i = 0; i < 4*4; i++)
				{
					assert(pred_decoded.modes_luma[i] == pred.modes_luma[i]);
					for (int j = 0; j < 4 * 4; j++)
						assert(resid_decoded.luma[i].val[j] == residuals.luma[i].val[j]);
				}
			}

			for (int plane = 0; plane < 2; plane++)
			{
				for (int i = 0; i < 2 * 2; i++)
				{
					assert(resid_decoded.chroma_dc[plane].val[i] == residuals.chroma_dc[plane].val[i]);
					for (int j = 1; j < 4 * 4; j++)
						assert(resid_decoded.chroma_ac[plane][i].val[j] == residuals.chroma_ac[plane][i].val[j]);
				}
			}

			pred_decoded.predict(neighbours);
			pred_decoded.add_residuals(residuals);

			for (int i = 0; i < 4*4; i++)
			for (int j = 0; j < 4*4; j++)
				assert(pred_decoded.mb.luma[i].Y[j] == mb.luma[i].Y[j]);

			for (int i = 0; i < 8*8; i++)
				assert(pred_decoded.mb.chroma_u.C[i] == mb.chroma_u.C[i]);
			for (int i = 0; i < 8*8; i++)
				assert(pred_decoded.mb.chroma_v.C[i] == mb.chroma_v.C[i]);
		}

		free(img_rgb);
	}

	void test_p_frame()
	{
		int qp = 0;
		inout_stream_t stream;

		// Create macroblocks
		uint8_t *img_0 = create_img(0);
		uint8_t *img_1 = create_img(1);

		macroblock_t prev_mb;
		prev_mb.from_rgb(img_0, MB_SIZE, MB_SIZE, 0, 0);
		macroblock_t curr_mb;
		curr_mb.from_rgb(img_1, MB_SIZE, MB_SIZE, 0, 0);

		// Create context with history
		predicted_macroblock_t pred, pred_decoded;
		pred.frame_type = FRAME_TYPE_P;
		pred.qp = qp;
		pred.qp_chroma_offset = 0;
		memcpy(&pred_decoded, &pred, sizeof(pred));

		neighbour_ctx_t neighbours(1, &prev_mb);
		neighbours.set_coords(0, 0);

		// Encode frame
		mb_residuals_t residuals, resid_decoded;
		{
			cabac_coder_t encoder((lpc_stream_out_t*)&stream, qp);

			pred.set_qp_delta(0);
			pred.select_mode(curr_mb, neighbours);
			pred.build_residuals(curr_mb, &residuals);
			pred.compute_cbp_flags(residuals);
			pred.encode_mb(neighbours, residuals, &encoder);

			encoder.encode_terminate(1);
			stream.flush();

			assert(pred.type == MB_TYPE_P); // test is invalid otherwise
		}

		// Decode frame
		{
			cabac_coder_t decoder((lpc_stream_in_t *)&stream, qp);

			pred_decoded.decode_mb(neighbours, &resid_decoded, &decoder);

			assert(pred_decoded.type == pred.type);
			assert(pred_decoded.cbp_chroma == pred.cbp_chroma);

			#if LPC_ADAPTIVE_QP
			assert(pred_decoded.qp_delta == pred.qp_delta);
			#endif

			{
				for (int i = 0; i < 4*4; i++)
				{
					for (int j = 0; j < 4 * 4; j++)
						assert(resid_decoded.luma[i].val[j] == residuals.luma[i].val[j]);
				}
			}

			for (int plane = 0; plane < 2; plane++)
			{
				for (int i = 0; i < 2 * 2; i++)
				{
					assert(resid_decoded.chroma_dc[plane].val[i] == residuals.chroma_dc[plane].val[i]);
					for (int j = 1; j < 4 * 4; j++)
						assert(resid_decoded.chroma_ac[plane][i].val[j] == residuals.chroma_ac[plane][i].val[j]);
				}
			}

			pred_decoded.predict(neighbours);
			pred_decoded.add_residuals(residuals);

			for (int i = 0; i < 4*4; i++)
			for (int j = 0; j < 4*4; j++)
				assert(pred_decoded.mb.luma[i].Y[j] == curr_mb.luma[i].Y[j]);

			for (int i = 0; i < 8*8; i++)
				assert(pred_decoded.mb.chroma_u.C[i] == curr_mb.chroma_u.C[i]);
			for (int i = 0; i < 8*8; i++)
				assert(pred_decoded.mb.chroma_v.C[i] == curr_mb.chroma_v.C[i]);
		}

		free(img_0);
		free(img_1);
	}
}

namespace lpc_unit_tests
{
	void run()
	{
		test_quantization();
		test_intra_prediction();
		test_stream();
		test_coder();
		test_modes_encoding();
		test_residual_encoding();
		test_mb_encoding();
		test_p_frame();

		remove(filename);
	}
}
#endif