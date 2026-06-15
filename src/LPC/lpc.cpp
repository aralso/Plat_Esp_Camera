#include <cmath>
#include <algorithm>
#include <Arduino.h>

#include "lpc.h"

#define LPC_VERSION 0
#define QP_MAX 51
#define QP_CHROMA_OFFSET 8

using std::min;
using std::max;

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
	return min(max(0, QP_MAX * (100 - quality) / 100), QP_MAX);
}

LPC_DEBUG_ONLY(static lpc_stats_t *STATS = NULL);

/// HELPERS

#include "lpc_cabac.inl"
#include "lpc_macroblock.inl"
#include "lpc_residuals.inl"
#include "lpc_neighbour.inl"

#include "lpc_image.inl"
#include "lpc_intra.inl"
#include "lpc_prediction.inl"
#include "lpc_entropy.inl"

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

	stream->write_byte(LPC_VERSION);
	stream->write_bytes((uint8_t*)&settings, sizeof(lpc_settings_t));
}

void lpc_encoder_t::close()
{
	stream->flush();
}

void do_encode(
	predicted_macroblock_t &predicted,
	macroblock_t &mb,
	neighbour_ctx_t &neighbours,
	cabac_coder_t &cabac)
{
	mb_residuals_t residuals;

	// Encoding
	predicted.select_intra_modes(mb, neighbours);
	predicted.build_residuals(mb, &residuals);
	predicted.compute_cbp_flags(residuals);
	predicted.encode_mb(neighbours, residuals, &cabac);

	// Neighbours update
	predicted.add_residuals(residuals);
	neighbours.update_data(predicted);

	LPC_DEBUG_ONLY(stats_add_mb(*STATS, predicted, mb));
}

void lpc_encoder_t::encode_frame(const uint8_t *rgb_bytes)
{
	int num_mb_x = div_round_up(width, MB_SIZE);
	int num_mb_y = div_round_up(height, MB_SIZE);

	LPC_DEBUG_ONLY(stats.reset(num_mb_x, num_mb_y));
	LPC_DEBUG_ONLY(STATS = &stats);

	macroblock_t mb;
	neighbour_ctx_t neighbours(num_mb_y);
	cabac_coder_t cabac(stream, qp);

	predicted_macroblock_t predicted;
	predicted.qp_chroma_offset = QP_CHROMA_OFFSET;
	predicted.qp = qp;

	for (int x = 0; x < num_mb_x; x++)
	{
		neighbours.start_column();
		for (int y = 0; y < num_mb_y; y++)
		{
			mb.from_rgb(rgb_bytes, width, height, x, y);
			neighbours.set_row(y);

			predicted.set_qp_delta(0);
			do_encode(predicted, mb, neighbours, cabac);
		}
		neighbours.end_column();
	}

	cabac.encode_terminate(1);
}

void lpc_encoder_t::encode_jpeg(lpc_stream_in_t *stream_in)
{
	int num_mb_x = div_round_up(width, MB_SIZE);
	int num_mb_y = div_round_up(height, MB_SIZE);
	macroblock_t *macroblocks = lpc_alloc<macroblock_t>(num_mb_x * num_mb_y, "Decoded JPEG");

	if (!macroblocks)
	{
		Serial.println("ERREUR: allocation macroblocks échouée !");
		return;
	}
	Serial.printf("Alloc : %d macroblocks\n", num_mb_x * num_mb_y);
	decode_jpeg(stream_in, (uint8_t*)macroblocks, width, height);

	LPC_DEBUG_ONLY(stats.reset(num_mb_x, num_mb_y));
	LPC_DEBUG_ONLY(STATS = &stats);

	neighbour_ctx_t neighbours(num_mb_y);
	cabac_coder_t cabac(stream, qp);

	predicted_macroblock_t predicted;
	predicted.qp_chroma_offset = QP_CHROMA_OFFSET;
	predicted.qp = qp;

	#if LPC_ADAPTIVE_QP
	// Compute variance across the whole image
	uint32_t avg_var = 0;
	for (int i = 0; i < num_mb_x*num_mb_y; i++)
		avg_var += compute_variance(macroblocks[i]);
	avg_var /= num_mb_x * num_mb_y;
	float log_var_avg = logf(avg_var + 1.0f);
	float log_var_avg_inv = 1.0f / log_var_avg;
	LPC_DEBUG_ONLY(stats.log_var_avg = log_var_avg);
	#endif

	Serial.printf("Encoding %d columns \n", num_mb_x);
	for (int x = 0; x < num_mb_x; x++)
	{
		//Serial.printf("Encoding column %d/%d\n", x+1, num_mb_x);
		neighbours.start_column();
		for (int y = 0; y < num_mb_y; y++)
		{
			macroblock_t &mb = macroblocks[x * num_mb_y + y];
			neighbours.set_row(y);

			#if LPC_ADAPTIVE_QP
			int target_qp = qp + compute_qp_delta(mb, log_var_avg, log_var_avg_inv);
			predicted.set_qp_delta(target_qp - predicted.qp);
			#endif

			do_encode(predicted, mb, neighbours, cabac);
		}
		neighbours.end_column();
	}

	cabac.encode_terminate(1);
	lpc_free(macroblocks);
}

/// LPC_DECODER

void lpc_decoder_t::open(lpc_stream_in_t *stream_in)
{
	stream = stream_in;

	int version = stream->read_byte();
	
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
	int qp = compute_qp(settings.quality);

	mb_residuals_t residuals;
	neighbour_ctx_t neighbours(num_mb_y);
	cabac_coder_t cabac(stream, qp);

	predicted_macroblock_t predicted;
	predicted.qp_chroma_offset = QP_CHROMA_OFFSET;
	predicted.qp = qp;

	for (int x = 0; x < num_mb_x; x++)
	{
		neighbours.start_column();
		for (int y = 0; y < num_mb_y; y++)
		{
			neighbours.set_row(y);

			// Decode
			predicted.decode_mb(neighbours, &residuals, &cabac);
			predicted.predict(neighbours);
			predicted.add_residuals(residuals);

			neighbours.update_data(predicted);

			predicted.mb.to_rgb(rgb_bytes, settings.width, settings.height, x, y);
		}
		neighbours.end_column();
	}

	cabac.decode_terminate();
}
