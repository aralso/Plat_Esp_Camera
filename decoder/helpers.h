#ifndef HELPERS_H
#define HELPERS_H

// For JPEG
#include "header.h"

static inline uint8_t clamp(int v)
{
	if (v < 0) return 0;
	if (v > 255) return 255;
	return v;
}

struct img_data_t
{
	int w, h;
	const char *name;

	uint8_t *bytes;
	size_t bytes_len;

	void alloc()
	{
		// RGB888
		bytes_len = w * h * 3 * sizeof(uint8_t);
		bytes = (uint8_t*)malloc(bytes_len);
		memset(bytes, 0, bytes_len);
	}

	img_data_t(int width, int height):
		w(width), h(height), bytes(NULL)
	{
		alloc();
	}

	~img_data_t()
	{
		if (bytes != NULL)
		{
			free(bytes);
			bytes = NULL;
		}
	}

	img_data_t(const char *_name):
		name(_name), w(800), h(600), bytes(NULL)
	{
		printf(">>>> %s\n", name);
		std::ifstream file(name, std::ios_base::binary);
		if (!file)
		{
			printf("File not found\n");
			return;
		}

		std::string jpeg(std::istreambuf_iterator<char>(file), (std::istreambuf_iterator<char>()));

			// Lightweight JPEG size probe: scan for SOF marker (0xFFC0 or 0xFFC2) and read
			// height/width. This is faster than a full decode and avoids an extra decode
			// when only dimensions are needed.
			auto getJpegSizeFromBuffer = [](const uint8_t *buf, size_t len, int &out_w, int &out_h)->bool {
				for (size_t i = 0; i + 9 < len; i++) {
					if (buf[i] == 0xFF && buf[i+1] == 0xC0) {
						out_h = (buf[i+5] << 8) + buf[i+6];
						out_w = (buf[i+7] << 8) + buf[i+8];
						if ((out_w>100) && (out_w<2000) && (out_h>50) && (out_h<2000)) 
							return true;
						else
							return false;
					}
				}
				return false;
			};

			int iw = 0, ih = 0;
			uint8_t *jpg_cast = (uint8_t*)jpeg.c_str();
			if (!getJpegSizeFromBuffer(jpg_cast, jpeg.size(), iw, ih)) {
				printf("Failed to read JPEG size\n");
				return;
			}

			w = iw; h = ih;
			alloc();

			// Decode using SDK-style fmt2rgb888 (4-arg). We pre-read the dimensions to
			// allocate the RGB buffer correctly and avoid a separate heavy decode for dims.
			if (!fmt2rgb888(jpg_cast, jpeg.size(), PIXFORMAT_JPEG, bytes)) {
				printf("JPEG decode failed\n");
				if (bytes) { free(bytes); bytes = NULL; }
				return;
			}
		}

	void dump_jpg(const char *path, uint8_t quality) // quality is in [1, 100]
	{
		if (bytes == NULL)
			return;
		uint8_t *jpg;
		size_t jpg_len;
		fmt2jpg(bytes, bytes_len, w, h, PIXFORMAT_RGB888, quality, &jpg, &jpg_len);
		std::ofstream file(path, std::ios_base::binary);
		for (int i = 0; i < jpg_len; i++)
			file << jpg[i];
		free(jpg);
	}

	void dump_bmp(const char *path)
	{
		uint8_t *bmp;
		size_t bmp_len;
		fmt2bmp(bytes, bytes_len, w, h, PIXFORMAT_RGB888, &bmp, &bmp_len);

		std::ofstream file(path, std::ios::binary);
		for (int i = 0; i < bmp_len; i++)
			file << bmp[i];

		free(bmp);
	}
};


#endif
