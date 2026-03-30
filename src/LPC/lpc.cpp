#include <cmath>
#include <algorithm>

#include "lpc.h"

using std::min;
using std::max;

#define CABAC_CONTEXT 8
#define QP_CHROMA_OFFSET 8

/// UTILS

#ifdef LPC_DEBUG
static struct allocs_t
{
	uint32_t count = 0;
	size_t memory = 0;
	void add(size_t size, const char *msg)
	{
		count++;
		memory += size;
		//printf("TOTAL = %.1fKo\tAlloc [%s]: %d\n", allocated_memory/1000.0f, (msg ? msg : "Unknown"), size);
	}
	void free()
	{
		count--;
	}

	~allocs_t()
	{
		LPC_ASSERT(count == 0);
	}
} allocs;
#endif

inline void* lpc_alloc(size_t size, const char *msg = NULL)
{
	LPC_DEBUG_ONLY(allocs.add(size, msg));
	return malloc(size);
}

template <typename T>
inline T* lpc_alloc(size_t size, const char *msg = NULL)
{
	return (T*)lpc_alloc(size * sizeof(T), msg);
}

inline void lpc_free(void *ptr)
{
	LPC_DEBUG_ONLY(allocs.free());
	free(ptr);
}

inline int div_round_up(int num, int denom)
{
	return (num + denom - 1) / denom;
}

inline uint8_t clamp8(int x)
{
	x &= ~(x >> 31); // clamp negative to 0
	x |= ((255 - x) >> 31); // clamp >255 to 255
	return (uint8_t)x;
}

inline uint8_t compute_qp(uint8_t quality)
{
	return min(max(0, 51 * (100 - quality) / 100), 51);
}

LPC_DEBUG_ONLY(static lpc_stats_t *STATS = NULL);

/// HELPERS

#include "lpc_cabac.inl"
#include "lpc_macroblock.inl"
#include "lpc_image.inl"
#include "lpc_neighbour.inl"
#include "lpc_intra.inl"
#include "lpc_prediction.inl"
#include "lpc_residuals.inl"

#ifdef LPC_DEBUG
#include "lpc_debug.inl"
#endif

/// LPC_ENCODER

void lpc_encoder_t::open(lpc_settings_t settings, lpc_stream_out_t *stream_out)
{
	stream = stream_out;
	width = settings.width;
	height = settings.height;
	qp = compute_qp(settings.quality);

	stream->write_bytes((uint8_t*)&settings, sizeof(lpc_settings_t));
}

void lpc_encoder_t::close()
{
	stream->flush();
}

void lpc_encoder_t::encode_frame(const uint8_t *rgb_bytes)
{
	LPC_DEBUG_ONLY(STATS = &stats);

	int num_mb_x = div_round_up(width, MB_SIZE);
	int num_mb_y = div_round_up(height, MB_SIZE);

	lpc_cabac_t cabac(stream, CABAC_CONTEXT);
	macroblock_t mb;
	predicted_macroblock_t predicted;
	residual_t residuals[NUM_RESIDUALS];
	neighbour_ctx_t neighbours(num_mb_y);

	#ifdef LPC_DEBUG
	countbytes_t cb1;
	lpc_cabac_t cabac_mode(&cb1, CABAC_CONTEXT);
	countbytes_t cb2;
	lpc_cabac_t cabac_resi(&cb2, CABAC_CONTEXT);
	#endif

	predicted.qp = qp;
	predicted.qpc = cst::compute_qp_chroma(predicted.qp + QP_CHROMA_OFFSET);
	predicted.cost_threshold = cst::cost_threshold[predicted.qp];

	for (int x = 0; x < num_mb_x; x++)
	{
		neighbours.start_column();
		for (int y = 0; y < num_mb_y; y++)
		{
			mb.from_rgb(rgb_bytes, width, height, x, y);

			neighbours.set_row(y);

			// Intra prediction
			predicted.select_intra_modes(mb, neighbours.top, neighbours.left);
			predicted.encode_modes(&cabac, neighbours);

			// Residuals and quantized transform
			build_residuals(mb, predicted, residuals);
			cabac.encode_bytes((uint8_t*)residuals, sizeof(residual_t) * NUM_RESIDUALS);

			#ifdef LPC_DEBUG
			//predicted.encode_modes(&cabac_mode, neighbours);
			// Luma
			if (predicted.type == MB_TYPE_4x4)
			{
				for (int block_i = 0; block_i < LUMA_BLOCK_COUNT; block_i++)
				for (int block_j = 0; block_j < LUMA_BLOCK_COUNT; block_j++)
				{
					int block_idx = block_i * LUMA_BLOCK_COUNT + block_j;
					int i = block_idx;
					auto &block = mb.luma[block_idx];

					const uint8_t *top = neighbours.top.get_luma();
					const uint8_t *left = neighbours.left.get_luma();

					top = top ? &top[block_i * LUMA_BLOCK_SIZE] : NULL;
					left = left ? &left[block_j * LUMA_BLOCK_SIZE] : NULL;

					// Find the best prediction mode for this block
					luma_block_t p;
					intra_mode_t m;
					uint32_t cost = find_mode_luma_4x4(block, top, left, &p, &m);
					if (cost >= predicted.cost_threshold)
						cabac_resi.encode_bytes((uint8_t*)(residuals+i), sizeof(residual_t));
				}
			}
			else
			{
				cabac_resi.encode_bytes((uint8_t*)residuals, sizeof(residual_t)*16);
			}
			// Chroma
			for (int i = 16; i < NUM_RESIDUALS; i++)
				cabac_resi.encode_bytes((uint8_t*)(residuals+i), sizeof(residual_t));
			#endif

			// Reconstruction for next macroblock neighbours
			build_macroblock(predicted, residuals, &predicted.mb);
			neighbours.update_data(predicted);

			LPC_DEBUG_ONLY(stats_add_mb(stats, predicted));
		}
		neighbours.end_column();
	}

	cabac.Flush();

	#ifdef LPC_DEBUG
	cabac_resi.Flush();

	printf("\n === CABAC ===\n");
	printf(" > Modes: %.1fKo\n", cb1.total_bytes/1000.0f);
	printf(" > Residuals: %.1fKo\n", cb2.total_bytes/1000.0f);
	#endif
}

/// LPC_DECODER

void lpc_decoder_t::open(lpc_stream_in_t *stream_in)
{
	stream = stream_in;
	stream->read_bytes((uint8_t*)&settings, sizeof(lpc_settings_t));
	LPC_ASSERT(!stream->empty());
}

void lpc_decoder_t::close()
{
}

void lpc_decoder_t::decode_frame(uint8_t *rgb_bytes)
{
	if (settings.frame_count == 0)
		return;
	settings.frame_count--;

	int num_mb_x = div_round_up(settings.width, MB_SIZE);
	int num_mb_y = div_round_up(settings.height, MB_SIZE);

	lpc_cabac_t cabac(stream, CABAC_CONTEXT);
	predicted_macroblock_t predicted;
	residual_t residuals[NUM_RESIDUALS];
	neighbour_ctx_t neighbours(num_mb_y);

	predicted.qp = compute_qp(settings.quality);
	predicted.qpc = cst::compute_qp_chroma(predicted.qp + QP_CHROMA_OFFSET);
	predicted.cost_threshold = cst::cost_threshold[predicted.qp];

	LPC_DEBUG_ONLY(bool intra_only = 0);
	LPC_DEBUG_ONLY(macroblock_t intra);

	for (int x = 0; x < num_mb_x; x++)
	{
		neighbours.start_column();
		for (int y = 0; y < num_mb_y; y++)
		{
			neighbours.set_row(y);

			// Intra prediction
			predicted.decode_modes(&cabac, neighbours);
			predicted.predict(neighbours.top, neighbours.left);

			LPC_DEBUG_ONLY(if (intra_only) memcpy(&intra, &predicted.mb, sizeof(intra)));

			// Residuals
			cabac.decode_bytes((uint8_t*)residuals, sizeof(residual_t) * NUM_RESIDUALS);
			build_macroblock(predicted, residuals, &predicted.mb);

			neighbours.update_data(predicted);

			LPC_DEBUG_ONLY(if (intra_only) memcpy(&predicted.mb, &intra, sizeof(intra)));

			predicted.mb.to_rgb(rgb_bytes, settings.width, settings.height, x, y);
		}
		neighbours.end_column();
	}
}
