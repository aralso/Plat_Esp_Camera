//#define CONFIG_ESP32 0

//#include <chrono>

#include <stdint.h>
#include <cstring>

#include <fstream>
#include <iostream>
#include <string>

#include "mjpegw.h"

#include "header.h"
#include "LPC/lpc.h"
#include "FS.h"
#include "SD_MMC.h"
#include "SDMMC.h"

#include "variables.h"


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

/// FILE STREAMS

/*#if CONFIG_ESP32
struct filestream_t : public lpc_stream_out_t
{
	File file;
	filestream_t(fs::FS &fs, const char *path)
	{
		file = fs.open(path, FILE_WRITE);
		assert(file);
	}
	~filestream_t()
	{
		file.close();
	}
	void write(const uint8_t *data, size_t size) override
	{
		file.write(data, size);
	}
};
#else*/
struct filestream_t : public lpc_stream_out_t, lpc_stream_in_t
{
	FILE *file;
	filestream_t(const char *path, const char *mode) { file = fopen(path, mode); }
	~filestream_t() { fclose(file); }

	#if 1
	size_t read(uint8_t *data, size_t size) override { return fread(data, 1, size, file); }
	void write(const uint8_t *data, size_t size) override { fwrite(data, 1, size, file); }
	#else
	size_t read(uint8_t *data, size_t size) override
	{
		for (int i = 0, value; i < size; i++)
			data[i] = (fscanf(file, "%d", &value), value);
		fscanf(file, "\n");
		return size;
	}
	void write(const uint8_t *data, size_t size) override
	{
		for (int i = 0; i < size; i++)
			fprintf(file, "%d ", data[i]);
		fprintf(file, "\n");
	}
	#endif
};
//#endif

bool file_exists(const char *name)
{
    std::ifstream f(name);
    return f.good();
}

const char *get_filename_ext(const char *filename)
{
    const char *dot = strrchr(filename, '.');
    if (!dot || dot == filename) return "";
    return dot + 1;
}

/// MAIN LOOP

void lpc_to_avi(lpc_decoder_t &lpc, const char *path)
{
	const lpc_settings_t settings = lpc.get_settings();

	std::string output = path;
	output.replace(output.find_last_of('.'), std::string::npos, ".avi");

	int cap_interval_dsec2 = settings.frequency;
	struct mjpegw_context *avi = mjpegw_open(output.c_str(), settings.width, settings.height, cap_interval_dsec2 * 100000, 60, settings.frame_count, NULL);
	{
		size_t bytes_len = settings.width * settings.height * 3 * sizeof(uint8_t);
		uint8_t* bytes = (uint8_t*)malloc(bytes_len);
		
		for (int i = 0; i < settings.frame_count; i++)
		{
			lpc.decode_frame(bytes);
			mjpegw_add_frame(avi, bytes, 3);
		}
	}
	mjpegw_close(avi);

	std::string mpv = "C:\\Data\\Donnees\\IOT\\WS_Platformio\\Plat_Esp_Camera\\decoder\\mpv\\mpv.exe";
	std::string cmd = mpv + " \"" + output + "\"";
	system(cmd.c_str());
}


uint8_t lpc_to_jpeg(lpc_decoder_t &lpc, const char *path)
{
	const lpc_settings_t settings = lpc.get_settings();

	std::string output = path;
	output.replace(output.find_last_of('.'), std::string::npos, ".jpg");

	{
		img_data_t img_rgb(settings.width, settings.height);
		lpc.decode_frame(img_rgb.bytes);

		img_rgb.dump_jpg(output.c_str(), 60);
	}

	ShellExecute(NULL, NULL, output.c_str(), NULL, NULL, SW_SHOW);

	return 0;
}

void get_jpeg_size(const char *path, uint16_t *width, uint16_t *height)
{
	filestream_t stream(path, "rb");

    int off = 0;
    while(!stream.empty())
	{
        uint8_t mrkr = stream.read_byte();
        if (mrkr == 0xff) continue;
        
        if(mrkr==0xd8) continue;    // SOI
        if(mrkr==0xd9) break;       // EOI
        if(0xd0<=mrkr && mrkr<=0xd7) continue;
        if(mrkr==0x01) continue;    // TEM
        
        int len = stream.read_uint16();
        
        if (mrkr==0xc0)
		{
			int bpc = stream.read_byte();
            *height = stream.read_uint16();
            *width  = stream.read_uint16();
			return;
        }

		stream.read_bytes(NULL, len - 2);
    }
}

uint8_t decodeFile()
{
	// le path du fichier a décoder est dans la variable path_c


	if (file_exists(path_c))
	{
		if (strcmp(get_filename_ext(path_c), "lpc") == 0)
		{
			filestream_t stream(path_c, "rb");
			lpc_decoder_t decoder;
			decoder.open(&stream);

			const lpc_settings_t settings = decoder.get_settings();
			printf("Decoding %d frames of size (%dx%d)\n", settings.frame_count, settings.width, settings.height);
			if (settings.frame_count > 1)
				lpc_to_avi(decoder, path_c);
			else	
				lpc_to_jpeg(decoder, path_c);

			decoder.close();
		}

		return 0;
	}
}
