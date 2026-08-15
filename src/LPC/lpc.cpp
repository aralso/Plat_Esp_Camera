#include "lpc.h"

#define LPC_VERSION 0
#define QP_MAX 51
#define QP_CHROMA_OFFSET 8
#define QP_MULT_P_FRAME 1
#define QP_OFFSET_P_FRAME 10

#ifdef __wasm__
// Handle the nolibc situation
extern "C" void* malloc(unsigned n);
extern "C" void free(void* p);
template<typename T> constexpr T min(T a, T b) { return a < b ? a : b; }
template<typename T> constexpr T max(T a, T b) { return a > b ? a : b; }
template<typename T> constexpr T abs(T x) { return x < 0 ? -x : x; }
extern "C" void* memcpy(void* dest, const void* src, size_t n)
{
	auto* d = static_cast<unsigned char*>(dest);
	const auto* s = static_cast<const unsigned char*>(src);
	for (size_t i = 0; i < n; ++i) d[i] = s[i];
	return dest;
}
extern "C" void* memset(void* dest, int value, size_t n)
{
	auto* d = static_cast<unsigned char*>(dest);
	auto v = static_cast<const unsigned char>(value);
	for (size_t i = 0; i < n; ++i) d[i] = v;
	return dest;
}
#else
#include <cstdlib>
#include <cstring>
#include <algorithm>

using std::min;
using std::max;
using std::abs;
#endif

/// UTILS

#ifdef LPC_DEBUG
#include <unordered_map>
static struct allocs_t
{
	std::unordered_map<void*, const char*> allocations;
	size_t memory = 0;
	void add(void *ptr, size_t size, const char *msg)
	{
		memory += size;
		allocations[ptr] = msg;
		//printf("TOTAL = %.1fKo\tAlloc [%s]: %d\n", allocated_memory/1000.0f, (msg ? msg : "Unknown"), size);
	}
	void free(void *ptr)
	{
		allocations.erase(ptr);
	}

	~allocs_t()
	{
		for (const auto it : allocations)
			printf("[LEAK] %p: %s\n", it.first, it.second);

		LPC_ASSERT(allocations.size() == 0);
	}
} allocs;
#endif

inline void* lpc_alloc(size_t size, const char *msg = nullptr)
{
	void *ptr = malloc(size);
	LPC_DEBUG_ONLY(allocs.add(ptr, size, msg));
	return ptr;
}

template <typename T>
inline T* lpc_alloc(size_t size, const char *msg = nullptr)
{
	return (T*)lpc_alloc(size * sizeof(T), msg);
}

inline void lpc_free(void *ptr)
{
	LPC_DEBUG_ONLY(allocs.free(ptr));
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

size_t lpc_stream_in_t::read_bytes(uint8_t *data, size_t size)
{
	LPC_ASSERT(bit_idx == 8);

	size_t bytes_read = 0;

	while (bytes_read < size)
	{
		if (idx == capacity)
		{
			capacity = (uint16_t)read(cache, LPC_STREAM_CACHE_SIZE);
			idx = 0;
		}

		if (idx == capacity)
		{
			done = true;
			break;
		}

		uint16_t cache_size = capacity - idx;
		uint16_t read_count = (uint16_t)(size - bytes_read);
		if (read_count > cache_size)
			read_count = cache_size;

		if (data)
			memcpy(data + bytes_read, cache + idx, read_count);
		bytes_read += read_count;
		idx += read_count;
	}

	return bytes_read;
}

LPC_DEBUG_ONLY(static lpc_stats_t *STATS = nullptr);

/// PROFILER

#ifdef LPC_PROFILE
#define PROFILER_SCOPE(id) lpc_profiler_t profiler_##__LINE__(id)
lpc_profiler_t::stats_t lpc_profiler_t::markers[LPC_MARKER_COUNT];
int lpc_profiler_t::nesting = 0;
#else
#define PROFILER_SCOPE(id)
#endif

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
	prev_frame = nullptr;

	stream->write_byte(LPC_VERSION);
	stream->write_uint16(settings.width);
	stream->write_uint16(settings.height);
	stream->write_byte(settings.quality);
	stream->write_byte(settings.frame_count);
	stream->write_byte(settings.frequency);
}

void lpc_encoder_t::close()
{
	if (prev_frame)
		lpc_free(prev_frame);
	stream->flush();
}

void do_encode(
	predicted_macroblock_t &predicted,
	macroblock_t &mb,
	neighbour_ctx_t &neighbours,
	cabac_coder_t &cabac)
{
	PROFILER_SCOPE(DO_ENCODE);

	mb_residuals_t residuals;

	// Encoding
	predicted.select_mode(mb, neighbours);
	predicted.build_residuals(mb, &residuals);
	predicted.compute_cbp_flags(residuals);
	predicted.encode_mb(neighbours, residuals, &cabac);

	// Neighbours update
	predicted.add_residuals(residuals);
	neighbours.update_data(predicted);

	LPC_DEBUG_ONLY(stats_add_mb(*STATS, predicted, mb));

	// Restore original qp if it was overriden
	predicted.restore_qp();
}

void lpc_encoder_t::encode_frame(const uint8_t *rgb_bytes)
{
	PROFILER_SCOPE(ENCODE_FRAME);

	int num_mb_x = div_round_up(width, MB_SIZE);
	int num_mb_y = div_round_up(height, MB_SIZE);

	LPC_DEBUG_ONLY(stats.reset(num_mb_x, num_mb_y));
	LPC_DEBUG_ONLY(STATS = &stats);

	macroblock_t mb;
	neighbour_ctx_t neighbours(num_mb_y);
	cabac_coder_t cabac(stream, qp);

	predicted_macroblock_t predicted;
	predicted.frame_type = FRAME_TYPE_I;
	predicted.qp_chroma_offset = QP_CHROMA_OFFSET;
	predicted.qp = qp;

	cabac.encode_bypass(predicted.frame_type == FRAME_TYPE_I);

	for (int x = 0; x < num_mb_x; x++)
	{
		for (int y = 0; y < num_mb_y; y++)
		{
			mb.from_rgb(rgb_bytes, width, height, x, y);
			neighbours.set_coords(x, y);

			predicted.set_qp_delta(0);
			do_encode(predicted, mb, neighbours, cabac);
		}
	}

	cabac.encode_terminate(1);
}

void lpc_encoder_t::encode_jpeg(lpc_stream_in_t *stream_in)
{
	PROFILER_SCOPE(ENCODE_FRAME);

	int num_mb_x = div_round_up(width, MB_SIZE);
	int num_mb_y = div_round_up(height, MB_SIZE);
	macroblock_t *macroblocks = lpc_alloc<macroblock_t>(num_mb_x * num_mb_y, "Decoded JPEG");

	// Handle out of memory
	if (macroblocks == nullptr) { macroblocks = prev_frame; prev_frame = nullptr; }
	if (macroblocks == nullptr) return;

	decode_jpeg(stream_in, (uint8_t*)macroblocks, width, height);

	LPC_DEBUG_ONLY(stats.reset(num_mb_x, num_mb_y));
	LPC_DEBUG_ONLY(STATS = &stats);

	neighbour_ctx_t neighbours(num_mb_y, prev_frame);
	cabac_coder_t cabac(stream, qp);

	predicted_macroblock_t predicted;
	predicted.frame_type = prev_frame ? FRAME_TYPE_P : FRAME_TYPE_I;
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

	cabac.encode_bypass(predicted.frame_type == FRAME_TYPE_I);

	for (int x = 0; x < num_mb_x; x++)
	{
		for (int y = 0; y < num_mb_y; y++)
		{
			macroblock_t &mb = macroblocks[x * num_mb_y + y];
			neighbours.set_coords(x, y);

			#if LPC_ADAPTIVE_QP
			int target_qp = qp + compute_qp_delta(mb, log_var_avg, log_var_avg_inv);
			predicted.set_qp_delta(target_qp - predicted.qp);
			#endif

			do_encode(predicted, mb, neighbours, cabac);
		}
	}

	cabac.encode_terminate(1);

	if (prev_frame)
		lpc_free(prev_frame);
	if (LPC_SUPPORT_P_FRAMES)
		prev_frame = macroblocks;
	else
		lpc_free(macroblocks);
}

/// LPC_DECODER

void lpc_decoder_t::open(lpc_stream_in_t *stream_in)
{
	stream = stream_in;
	prev_frame = nullptr;

	int version = stream->read_byte();
	(void) version; // unused for now
	
	settings.width = stream->read_uint16();
	settings.height = stream->read_uint16();
	settings.quality = stream->read_byte();
	settings.frame_count = stream->read_byte();
	settings.frequency = stream->read_byte();

	LPC_ASSERT(!stream->empty());
}

void lpc_decoder_t::close()
{
	if (prev_frame)
		lpc_free(prev_frame);
}

void lpc_decoder_t::decode_frame(uint8_t *rgb_bytes)
{
	int num_mb_x = div_round_up(settings.width, MB_SIZE);
	int num_mb_y = div_round_up(settings.height, MB_SIZE);
	int qp = compute_qp(settings.quality);

	macroblock_t *macroblocks = lpc_alloc<macroblock_t>(num_mb_x * num_mb_y, "Previous frame");

	mb_residuals_t residuals;
	neighbour_ctx_t neighbours(num_mb_y, prev_frame);
	cabac_coder_t cabac(stream, qp);

	predicted_macroblock_t predicted;
	predicted.frame_type = cabac.decode_bypass() ? FRAME_TYPE_I : FRAME_TYPE_P;
	predicted.qp_chroma_offset = QP_CHROMA_OFFSET;
	predicted.qp = qp;

	for (int x = 0; x < num_mb_x; x++)
	{
		for (int y = 0; y < num_mb_y; y++)
		{
			neighbours.set_coords(x, y);

			// Decode
			predicted.decode_mb(neighbours, &residuals, &cabac);
			predicted.predict(neighbours);
			predicted.add_residuals(residuals);

			neighbours.update_data(predicted);
			predicted.restore_qp();

			macroblocks[x * num_mb_y + y] = predicted.mb;
			predicted.mb.to_rgb(rgb_bytes, settings.width, settings.height, x, y);
		}
	}

	if (prev_frame)
		lpc_free(prev_frame);
	if (LPC_SUPPORT_P_FRAMES)
		prev_frame = macroblocks;
	else
		lpc_free(macroblocks);

	cabac.decode_terminate();
}
