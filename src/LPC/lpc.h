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
#define LPC_USE_CABAC 1
#define LPC_SUPPORT_P_FRAMES 1
#define LPC_SUPPORT_4x4 0   // 1: support 4x4 luma blocks, 0: only 16x16 luma blocks
#define LPC_ADAPTIVE_QP 0


/// Debug macros

#define EXTENDED_STATS 0
#define LPC_TESTS 0

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

const char *get_filename_ext(const char *filename);

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
			capacity = (uint16_t)read(cache, LPC_STREAM_CACHE_SIZE);
			idx = 0;
		}

		if (idx == capacity)
			done = true;

		return done ? '\0' : cache[idx++];
	}

	uint16_t read_uint16()
	{
		return (((uint16_t)read_byte()) << 8) | (uint16_t)read_byte();
	}

	inline size_t read_bytes(uint8_t *data, size_t size)
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
	inline void flush()
	{
		if (len) write(cache, len);
		if (bit_idx != 0) write(&tmp_byte, 1);
		tmp_byte = 0;
		bit_idx = 0;
		len = 0;
	}

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

	void write_byte(uint8_t input)
	{
		LPC_ASSERT(bit_idx == 0);

		if (len == LPC_STREAM_CACHE_SIZE)
			flush();

		cache[len++] = input;
	}

	void write_uint16(uint16_t val)
	{
		write_byte(val >> 8);
		write_byte(val & 0xFF);
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
	int num_mb_x, num_mb_y;
	int num_macroblocks;

	int num_mb_luma_4x4;
	int num_block_non_coded[4];
	int num_block_match_pred;
	int *num_block_per_intra_mode[STAT_COUNT];

	float mse;
	int qp_avg;
	int log_var_avg;

	bool has_pixel;
	uint8_t *debug_img;
	void set_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b);

	lpc_stats_t();
	~lpc_stats_t();
	void reset(int mb_x, int mb_y);
	void print();
};
#endif

struct lpc_encoder_t
{
	void open(lpc_settings_t settings, lpc_stream_out_t *stream_out);
	void close();

	void encode_frame(const uint8_t *rgb_bytes);
	void encode_jpeg(lpc_stream_in_t *stream_in);

	LPC_DEBUG_ONLY(lpc_stats_t stats);

private:
	lpc_stream_out_t *stream;
	struct macroblock_t *prev_frame;
	uint16_t width;
	uint16_t height;
	uint8_t qp;
};

struct lpc_decoder_t
{
	void open(lpc_stream_in_t *stream_in);
	void close();

	void decode_frame(uint8_t *rgb_bytes);

	const lpc_settings_t &get_settings() const { return settings; }

private:
	lpc_stream_in_t *stream;
	struct macroblock_t *prev_frame;
	lpc_settings_t settings;
};

#ifdef LPC_DEBUG
namespace lpc_unit_tests
{
	void run();
}
#endif
