#pragma once

#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cstddef>
#include <cinttypes>

/// Compile time settings
#ifndef LPC_STREAM_CACHE_SIZE
#define LPC_STREAM_CACHE_SIZE 1024
#endif

#define LPC_USE_YCBCR 1


/// Debug macros

#ifdef DEBUG
#define LPC_DEBUG
#endif

#ifdef LPC_DEBUG
#define LPC_DEBUG_ONLY(...) __VA_ARGS__
#define LPC_ASSERT(x) assert(x)
#else
#define LPC_DEBUG_ONLY(x)
#define LPC_ASSERT(x)
#endif


/// Public API

struct lpc_settings_t
{
	uint16_t width;
	uint16_t height;
	uint8_t quality; // [0 - 100]
	uint8_t frame_count;
	uint8_t frequency; // number of images per second
};

struct lpc_stream_in_t
{
	bool empty() { return done; }

	bool read_bit()
	{
		if (bit_idx == 8)
		{
			tmp_byte = read_byte();
			bit_idx = 0;
		}

		bool bit = (tmp_byte >> (7-bit_idx)) & 1;
		bit_idx++;
		return bit;
	}

	uint8_t read_byte()
	{
		LPC_ASSERT(bit_idx == 8);

		if (idx == capacity)
		{
			capacity = read(cache, LPC_STREAM_CACHE_SIZE);
			idx = 0;
		}

		if (idx == capacity)
			done = true;

		return done ? '\0' : cache[idx++];
	}

	inline size_t read_bytes(uint8_t *data, size_t size)
	{
		LPC_ASSERT(bit_idx == 8);

		int from_cache = capacity - idx;
		memcpy(data, cache, from_cache);
		int bytes_read = read(data + from_cache, size - from_cache);
		idx = capacity = 0;
		if (bytes_read < (size - from_cache))
			done = true;
		return from_cache + bytes_read;
	}

protected:
	virtual size_t read(uint8_t *data, size_t size) = 0;

private:
	uint8_t cache[LPC_STREAM_CACHE_SIZE];
	uint16_t capacity = 0, idx = 0;
	bool done = false;

	uint8_t bit_idx = 8;
	uint8_t tmp_byte = 0;
};

struct lpc_stream_out_t
{
	inline void flush() { write(cache, len); len = 0; }

	void write_bit(bool bit)
	{
		tmp_byte |= (bit << (7-bit_idx));
		bit_idx++;

		if (bit_idx == 8)
		{
			bit_idx = 0;
			write_byte(tmp_byte);
			tmp_byte = 0;
		}
	}

	void write_byte(uint8_t byte)
	{
		LPC_ASSERT(bit_idx == 0);

		if (len == LPC_STREAM_CACHE_SIZE)
			flush();

		cache[len++] = byte;
	}

	inline void write_bytes(const uint8_t *data, size_t size)
	{
		LPC_ASSERT(bit_idx == 0);
		flush();
		write(data, size);
	}

protected:
	virtual void write(const uint8_t *data, size_t size) = 0;

private:
	uint8_t cache[LPC_STREAM_CACHE_SIZE];
	uint16_t len = 0;

	uint8_t bit_idx = 0;
	uint8_t tmp_byte = 0;
};

#ifdef LPC_DEBUG
enum
{
	STAT_LUMA_4x4,
	STAT_LUMA_16x16,
	STAT_CHROMA,
	STAT_COUNT
};

struct lpc_stats_t
{
	int num_macroblocks;

	int num_mb_luma_4x4;
	int num_block_below_threshold[STAT_COUNT];
	int num_block_match_pred[STAT_COUNT];
	int *num_block_per_intra_mode[STAT_COUNT];

	lpc_stats_t();
	~lpc_stats_t();
	void print();
};
#endif

struct lpc_encoder_t
{
	void open(lpc_settings_t settings, lpc_stream_out_t *stream_out);
	void close();

	void encode_frame(const uint8_t *rgb_bytes);

	LPC_DEBUG_ONLY(lpc_stats_t stats);

private:

	lpc_stream_out_t *stream;
	uint16_t width;
	uint16_t height;
	uint8_t qp;
};

struct lpc_decoder_t
{
	void open(lpc_stream_in_t *stream_in);
	void close();

	void decode_frame(uint8_t *rgb_bytes);

private:

	lpc_stream_in_t *stream;
	lpc_settings_t settings;
};

#ifdef LPC_DEBUG
namespace lpc_unit_tests
{
	void run();
}
#endif
