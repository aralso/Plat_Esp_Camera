#ifndef HELPERS_H
#define HELPERS_H

// For JPEG
#include "header.h"

#define FMT_RGB 0
#define FMT_422 1
#define FMT_420 2

#define IMG_NORMAL 0
#define IMG_LARGE 1

#include <FS.h>
#include "SD_MMC.h"

static inline uint8_t clamp(int v)
{
	if (v < 0) return 0;
	if (v > 255) return 255;
	return v;
}


const char *get_img_normal(int i)
{
	i++;
	if (i >= 2) i++;
	static char filename[64];
	sprintf(filename, "/test/normal/C01-026-F5-Q3-T0-201001-%06d.jpg", i);
	return filename;
}

const char *get_img_large(int i)
{
	i = 4 + i;
	static char filename[64];
	sprintf(filename, "/test/large/C01-066-F5-Q3-T0-201001-%06d.jpg", i);
	return filename;
}

const char *get_img(int i, int type)
{
	if (type == IMG_NORMAL)
		return get_img_normal(i);
	if (type == IMG_LARGE)
		return get_img_large(i);

	return NULL;
}

const char *get_img_recomp(int i)
{
	static char filename[64];
	sprintf(filename, "img/normal-recomp/img%02d.jpg", i);
	return filename;
}

const char *get_img_diff(int i, int type)
{
	static char filename[64];
	const char* type_name = (type == IMG_NORMAL) ? "normal" : "large";
	sprintf(filename, "img/%s-diff/img%02d.jpg", type_name, i);
	return filename;
}

const char *get_img_extract(int i, int type, bool to_bmp)
{
	static char filename[64];
	const char* type_name = (type == IMG_NORMAL) ? "normal" : "large";
	const char* format = to_bmp ? "bmp" : "jpg";
	sprintf(filename, "img/%s-extract/img%02d.%s", type_name, i, format);
	return filename;
}

const char *get_img_heif(int i, int type, bool to_bmp)
{
	static char filename[64];
	const char* type_name = (type == IMG_NORMAL) ? "normal" : "large";
	const char* format = to_bmp ? "bmp" : "jpg";
	sprintf(filename, "img/%s-heif/img%02d.%s", type_name, i, format);
	return filename;
}


struct img_data_t
{
	int w, h;
	const char *name;
	bool valid = false;
	uint8_t *bytes;
	size_t bytes_len;

	void alloc()
	{
		// RGB888
		bytes_len = (size_t)w * (size_t)h * 3 * sizeof(uint8_t);
		bytes = (uint8_t*)malloc(bytes_len);
		if (bytes) 
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
		File file = SD_MMC.open(name);
		//std::ifstream file(name);
		if (!file)
		{
			printf("File not found\n");
			return;
		}

		//std::string jpeg(std::istreambuf_iterator<char>(file), (std::istreambuf_iterator<char>()));
		size_t size = file.size();
    	uint8_t* jpg_buf = (uint8_t*)malloc(size);
		if (jpg_buf)
		{
			file.read(jpg_buf, size);
			file.close();

			alloc();

			if (bytes)
			{
				fmt2rgb888(jpg_buf, size, PIXFORMAT_JPEG, bytes);
				//uint8_t *jpg_cast = (uint8_t*)jpeg.c_str();
				//fmt2rgb888(jpg_cast, jpeg.size(), PIXFORMAT_JPEG, bytes);
				free(jpg_buf);
				valid = true;
			}
			else
			{
				free(jpg_buf);
			}
		}
		else
		{
			file.close();
		}
	}

	void dump_jpg(const char *path, uint8_t quality) // quality is in [1, 100]
	{
		if (bytes == NULL)
			return;
		uint8_t *jpg;
		size_t jpg_len;
		fmt2jpg(bytes, bytes_len, w, h, PIXFORMAT_RGB888, quality, &jpg, &jpg_len);
		std::ofstream file(path);
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
