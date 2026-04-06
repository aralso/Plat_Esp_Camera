#define ESP32 1

#include <stdint.h>
#include <cstring>

#include <fstream>
#include <iostream>
#include <string>

#include "helpers.h"

#include "lpc.h"
#include "FS.h"
#include "SD_MMC.h"
#include "SDMMC.h"


/// FILE STREAMS

#if ESP32
	struct filestream_t : public lpc_stream_out_t
	{
		File file;
		filestream_t(fs::FS &fs, const char *path)
		{
			file = fs.open(path, FILE_APPEND);
			assert(file);  // Vérifie que le fichier s'est bien ouvert
		}
		~filestream_t()
		{
			file.close();  // Ferme le fichier à la destruction de l'objet
		}
		void write(const uint8_t *data, size_t size) override
		{
			file.write(data, size);  // Écrit les données dans le fichier
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

/// CONSOLE CONTROLS

#define INTERACTIVE 1

const int g_run_encode		= 1 << 0;
const int g_run_decode		= 1 << 1;
const int g_display_stats	= 1 << 2;
const int g_set_quality		= 1 << 3;
const int g_run_unit_tests	= 1 << 4;
const int g_procedural_img	= 1 << 5;
const int g_exit 			= 1 << 31;

/*int parse_cmd(const char* cmd, uint8_t *quality)
{
	int actions = 0;
	for (; *cmd != '\0' && *cmd != ' '; cmd++)
	{
		int digit = *cmd - '0';
		int action = (1 << (digit - 1));
		if (*cmd == 'q')
			action = g_exit;

		actions |= action;
	}

	if (actions & g_set_quality)
	{
		*quality = (uint8_t)atoi(cmd);
		printf("Encoding with quality=%d\n", *quality);
	}

	return actions;
}*/

/// MAIN LOOP
int encode_lpc(const lpc_settings_t &settings)
{
	int type = IMG_LARGE;
	uint32_t img_count = (type == IMG_NORMAL) ? 14 : 12;
	const char *output_path = "test/test.lpc";
	Serial.println("1. Run encoder");
	if (SD_MMC.exists(output_path)) {
					SD_MMC.remove(output_path);
	}			
	filestream_t stream(SD_MMC, output_path);
	lpc_encoder_t encoder;
	encoder.open(settings, &stream);

	for (uint32_t i = 0; i < settings.frame_count; ++i)
	{
		img_data_t img_rgb(get_img(i, type));
		encoder.encode_frame(img_rgb.bytes);
	}

	encoder.close();
	Serial.println("Encoding done");
	return 0;
}