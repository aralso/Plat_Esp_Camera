#define CONFIG_ESP32 0

#include <chrono>

#include <stdint.h>
#include <cstring>

#include <fstream>
#include <iostream>
#include <string>

#include "mjpegw.h"

#include "helpers.h"
#include "lpc.h"

/// FILE STREAMS

#if CONFIG_ESP32
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
#else
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
#endif

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

int main(int argc, const char **argv)
{
	if (file_exists(argv[1]))
	{
		if (strcmp(get_filename_ext(argv[1]), "lpc") == 0)
		{
			filestream_t stream(argv[1], "rb");
			lpc_decoder_t decoder;
			decoder.open(&stream);

			const lpc_settings_t settings = decoder.get_settings();
			printf("Decoding %d frames of size (%dx%d)\n", settings.frame_count, settings.width, settings.height);
			if (settings.frame_count > 1)
				lpc_to_avi(decoder, argv[1]);
			else	
				lpc_to_jpeg(decoder, argv[1]);

			decoder.close();
		}

		return 0;
	}


	/*
	lpc_settings_t settings =
	{
		.width = 578,
		.height = 430,
		.quality = (uint8_t)50,
		.frequency = 2
	};

	{
		img_data_t img_rgb(settings.width, settings.height);

		filestream_t stream("procedural.lpc", "wb");
		settings.frame_count = 1;

		lpc_encoder_t encoder;
		encoder.open(settings, &stream);

		filestream_t jpeg("CA-260804-094224-H-145.jpg", "rb");
		encoder.encode_jpeg(&jpeg);

		printf("Encoding complete\n");
		encoder.stats.print();

		encoder.close();
	}
	{
		printf("Decoding\n");

		filestream_t stream("procedural.lpc", "rb");
		lpc_decoder_t decoder;
		decoder.open(&stream);

		img_data_t img_rgb(settings.width, settings.height);
		for (int frame = 0; frame < settings.frame_count; frame++)
		{
			decoder.decode_frame(img_rgb.bytes);
			img_rgb.dump_jpg(("procedural_" + std::to_string(frame) + ".jpg").c_str(), 60);
		}

		decoder.close();
	}
	*/
}
