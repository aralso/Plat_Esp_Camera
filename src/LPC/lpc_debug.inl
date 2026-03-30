#include "lpc.h"

#include <string>

/// PRINT

static const char* to_string(intra_mode_t mode)
{
	switch (mode)
	{
		case INTRA_VERTICAL:
			return "INTRA_VERTICAL";
		case INTRA_HORIZONTAL:
			return "INTRA_HORIZONTAL";
		case INTRA_DC:
			return "INTRA_DC";
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

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			printf("%d\t", val[i*4+j]);
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

struct countbytes_t : public lpc_stream_out_t
{
	int total_bytes = 0;
	void write(const uint8_t *data, size_t size) override
	{
		total_bytes += size;
	}
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
}

static void print_stat(const char *prefix, const char *msg, int value, int max_value)
{
	printf("%s %s = %d (%.1f%%)\n", prefix, msg, value, 100.0f*value/(float)max_value);
}

void lpc_stats_t::print()
{
	int num_luma_4x4_block = num_mb_luma_4x4 * LUMA_BLOCK_COUNT*LUMA_BLOCK_SIZE;
	int num_luma_16x16_block = num_macroblocks - num_mb_luma_4x4;
	int num_chroma_block = num_macroblocks;

	printf("\n === ENCODING STATISTICS ===\n");
	printf("> Num macroblocks = %d\n", num_macroblocks);

	printf("\n");
	print_stat(">", "Num macroblocks using 4x4 luma blocks",
			num_mb_luma_4x4, num_macroblocks);
	print_stat(">", "Num 4x4 luma blocks below cost threshold",
			num_block_below_threshold[STAT_LUMA_4x4], num_luma_4x4_block);
	print_stat(">", "Num 16x16 luma blocks below cost threshold",
			num_block_below_threshold[STAT_LUMA_16x16], num_luma_16x16_block);
	print_stat(">", "Num chroma blocks below cost threshold",
			num_block_below_threshold[STAT_CHROMA], num_chroma_block);

	printf("\n");
	print_stat(">", "Num 4x4 luma blocks using predicted mode",
			num_block_match_pred[STAT_LUMA_4x4], num_luma_4x4_block);
	print_stat(">", "Num 16x16 luma blocks using predicted mode",
			num_block_match_pred[STAT_LUMA_16x16], num_luma_16x16_block);
	print_stat(">", "Num chroma blocks using predicted mode",
			num_block_match_pred[STAT_CHROMA], num_chroma_block);

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
}

void stats_add_mb(lpc_stats_t &stats, const predicted_macroblock_t &predicted)
{
	stats.num_macroblocks++;
	stats.num_mb_luma_4x4 += (predicted.type == MB_TYPE_4x4) ? 1 : 0;

	if (predicted.type == MB_TYPE_4x4)
	{
		for (int i = 0; i < LUMA_BLOCK_COUNT*LUMA_BLOCK_COUNT; i++)
			stats.num_block_per_intra_mode[STAT_LUMA_4x4][predicted.modes_luma[i]]++;
	}
	else
	{
		stats.num_block_per_intra_mode[STAT_LUMA_16x16][predicted.mode_luma]++;
	}

	stats.num_block_per_intra_mode[STAT_CHROMA][predicted.mode_chroma]++;
}

/// UNIT TESTS

static void do_test_intra_prediction(const neighbour_ctx_t &neighbours,
		bool test_top, const predicted_macroblock_t &top_mb,
		bool test_left, const predicted_macroblock_t &left_mb,
		int expected_dc)
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
		const uint8_t *orig_luma = (uint8_t*)top_mb.mb.luma;
		uint8_t *predicted_luma = (uint8_t*)predicted.mb.luma;
		cost = find_mode_luma_16x16(orig_luma,
				neighbours.top.get_luma(), neighbours.left.get_luma(),
				predicted_luma, &predicted.mode_luma);

		for (int i = 0; i < LUMA_BLOCK_COUNT; i++)
			assert(neighbours.top.modes_luma[i] == top_mb.mode_luma);
		assert(predicted.mode_luma == INTRA_VERTICAL);
		assert(cost == 0);
	}
	if (test_left)
	{
		const uint8_t *orig_luma = (uint8_t*)left_mb.mb.luma;
		uint8_t *predicted_luma = (uint8_t*)predicted.mb.luma;
		cost = find_mode_luma_16x16(orig_luma,
				neighbours.top.get_luma(), neighbours.left.get_luma(),
				predicted_luma, &predicted.mode_luma);

		for (int i = 0; i < LUMA_BLOCK_COUNT; i++)
			assert(neighbours.left.modes_luma[i] == left_mb.mode_luma);
		assert(predicted.mode_luma == INTRA_HORIZONTAL);
		assert(cost == 0);
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
		assert(cost == 0);
	}
	if (test_left)
	{
		cost = find_mode_chroma(left_mb.mb,
				neighbours.top, neighbours.left,
				&predicted.mb, &predicted.mode_chroma);

		assert(*neighbours.left.mode_chroma == left_mb.mode_chroma);
		assert(predicted.mode_chroma == INTRA_HORIZONTAL);
		assert(cost == 0);
	}
	{
		predict_chroma(predicted.mb.chroma_u, INTRA_DC,
				neighbours.top.get_chroma_u(), neighbours.left.get_chroma_u(),
				&predicted.mb.chroma_u);

		for (int i = 0; i < CHROMA_BLOCK_SIZE*CHROMA_BLOCK_SIZE; i++)
			assert(predicted.mb.chroma_u.C[i] == expected_dc);
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
			m1.mb.luma[i].Y[j] = 1;
			m3.mb.luma[i].Y[j] = 3;
		}
	}
	for (int i = 0; i < 8*8; i++)
	{
		m1.mb.chroma_u.C[i] = 1;
		m1.mb.chroma_v.C[i] = 1;

		m3.mb.chroma_u.C[i] = 3;
		m3.mb.chroma_v.C[i] = 3;
	}

	neighbours.start_column();
	{
		neighbours.set_row(0);
		neighbours.update_data(m1);

		neighbours.set_row(1);
		// The image has the following macroblocks
		// | 1 | ? |
		// | X | ? |
		do_test_intra_prediction(neighbours, true, m1, false, m3, 1);
		neighbours.update_data(m3);

	}
	neighbours.end_column();

	neighbours.start_column();
	{
		neighbours.set_row(0);
		// The image has the following macroblocks
		// | 1 | X |
		// | 3 | ? |
		do_test_intra_prediction(neighbours, false, m1, true, m1, 1);
		neighbours.update_data(m1);

		neighbours.set_row(1);
		// The image has the following macroblocks
		// | 1 | 1 |
		// | 3 | X |
		do_test_intra_prediction(neighbours, true, m1, true, m3, 2);
	}
	neighbours.end_column();
}

namespace lpc_unit_tests
{

struct filestream_t
{
	FILE *file;
	void open(const char *path, const char *mode) { file = fopen(path, mode); }
	void close() { fclose(file); }
};

struct filestream_in_t : public filestream_t, lpc_stream_in_t
{
	size_t read(uint8_t *data, size_t size) override { return fread(data, 1, size, file); }
};
struct filestream_out_t : public filestream_t, lpc_stream_out_t
{
	void write(const uint8_t *data, size_t size) override { fwrite(data, 1, size, file); }
};

static void test_stream()
{
	const char *file = "/tmp/stream.tst";

	filestream_out_t out;
	out.open(file, "w+");
	{
		for (int i = 0; i < 8; i++)
			out.write_bit(('>' >> (7-i)) & 1);
		for (int i = 0; i < strlen(file); i++)
			out.write_byte(file[i]);
		for (int i = 0; i < 8; i++)
			out.write_bit(('\n' >> (7-i)) & 1);
		out.write_bytes((const uint8_t*)file, strlen(file));
		out.write_byte('\n');
		out.flush();
	}
	out.close();

	filestream_in_t in;
	in.open(file, "r");
	{
		std::string ref = ">" + std::string(file) + "\n" + std::string(file) + "\n";
		std::string tmp = "";
		char character = 0;
		for (int i = 0; i < 8; i++)
			character |= in.read_bit() << (7-i);
		tmp += character;
		while (!in.empty())
			tmp += in.read_byte();
		uint8_t tmp2[256];
		int byte_count = in.read_bytes(tmp2, strlen(file) + 1);
		for (int i = 0; i < byte_count; i++)
			tmp += tmp2[byte_count];

		assert(strcmp(tmp.c_str(), ref.c_str()) == 0);
	}
	in.close();
}
}

namespace lpc_unit_tests
{
	void run()
	{
		test_intra_prediction();
		test_stream();
	}
}
