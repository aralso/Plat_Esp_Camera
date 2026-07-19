#define ESP32 1

#include <stdint.h>
#include <cstring>

#include <fstream>
#include <iostream>
#include <string>

#include "helpers.h"
#include "mjpegw.h"

#include "lpc.h"
#include "FS.h"
#include "SD_MMC.h"
#include "SDMMC.h"

#include "variables.h"

extern uint8_t code_encod;
extern char path_c[128];

// Size/codec helpers
// New signatures: each function provides the other two values via output parameters.
// - size_to_code: input width (pixels), outputs framesize (cam_size) and code (index), returns code
// - camcode_to_code: input framesize (cam_code), outputs code and representative width, returns code
// - code_to_size: input code, outputs representative width and framesize, returns code
uint8_t size_to_code(uint16_t size, framesize_t &cam_size, uint8_t &code);
uint8_t camcode_to_code(framesize_t cam_size, uint8_t &code, uint16_t &rep_width);
uint8_t code_to_size(uint8_t code, uint16_t &rep_width, framesize_t &cam_size);

// JPEG compression mapping helpers
// tx_compjpg_to_code: input txJpg(percent), outputs txCam (camera jpeg_quality) and code
uint8_t tx_compjpg_to_code(uint8_t txJpg, uint8_t &txCam, uint8_t &code);
// inverse: from txCam to representative txJpg and code
uint8_t txcam_to_compjpg(uint8_t txCam, uint8_t &txJpg, uint8_t &code);
// inverse: from code to txJpg and txCam
uint8_t code_to_compjpg(uint8_t code, uint8_t &txJpg, uint8_t &txCam);

uint8_t nbIm_to_code (uint8_t nb_images);
uint8_t code_to_nbIm (uint8_t code);

// Global code mapping helpers
// Convert a triplet (images_code, size_code, comp_code) into a single global char code
char triplet_to_global(uint8_t images_code, uint8_t size_code, uint8_t comp_code);
// Parse a global char code into the triplet; returns true if found
bool global_to_triplet(char global_code, uint8_t &images_code, uint8_t &size_code, uint8_t &comp_code);

/// FILE STREAMS

#ifdef __cplusplus
extern "C" {
#endif

void debug_ctx(mjpegw_context* avi);

#ifdef __cplusplus
}
#endif


struct jpeg_reader_t : public lpc_stream_in_t
{
File file;
jpeg_reader_t(fs::FS &fs, const char *path)
{
file = fs.open(path, FILE_READ);
assert(file);
}
~jpeg_reader_t()
{
file.close();
}
size_t read(uint8_t *data, size_t size) override
{
return file.read(data, size);
}
};


#if ESP32
struct filestream_t : public lpc_stream_out_t, public lpc_stream_in_t
{
    File file;

    filestream_t(fs::FS &fs, const char *path, const char *mode = FILE_READ)
    {
        file = fs.open(path, mode);
        assert(file);
    }

    ~filestream_t()
    {
        file.close();
    }

    // OUT
    void write(const uint8_t *data, size_t size) override
    {
        file.write(data, size);
    }

    // IN
    size_t read(uint8_t *data, size_t size) override
    {
        return file.read(data, size);
    }

    uint8_t read_byte()
    {
        return file.read();
    }

    uint16_t read_uint16()
    {
        uint16_t v;
        file.read((uint8_t*)&v, 2);
        return __builtin_bswap16(v); // JPEG = big endian
    }

    bool empty()
    {
        return file.available() == 0;
    }
	uint32_t position()
	{
		return file.position();
	}
	bool seek(uint32_t pos)
	{
		return file.seek(pos);
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

uint8_t lpc_to_avi(lpc_decoder_t &lpc, const char *input_path, const char *output_path);
uint8_t lpc_to_jpg(lpc_decoder_t &lpc, const char *input_path, const char *output_path);

int parse_cmd(const char* cmd, uint8_t *quality)
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
}

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

uint8_t encode_lpc2(const lpc_settings_t &settings, uint8_t * jpg_bu, size_t jpg_len, const char*path1)
{

	const char *output_path = "/test/test.lpc";
	//Serial.printf("1. Run encoder  quality=%d, frame_count=%d\n", settings.quality, settings.frame_count);
	if (SD_MMC.exists(output_path)) {
			Serial.println("suppression du fichier de sortie existant");
			SD_MMC.remove(output_path);
	}			
	filestream_t stream(SD_MMC, output_path, FILE_APPEND);
	Serial.println("EnAAA");
	lpc_encoder_t encoder;
	encoder.open(settings, &stream);
	Serial.printf("Encodage width=%d height=%d quality=%d\n", settings.width, settings.height, settings.quality);

	// --- 2. décoder JPEG → RGB ---
    uint8_t* rgb_buf = NULL;
    size_t rgb_len;

	// --- Lightweight probe to extract JPEG dimensions before decoding ---
	int Jwidth = 0, Jheight = 0;
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

	if (!getJpegSizeFromBuffer(jpg_bu, jpg_len, Jwidth, Jheight)) {
		Serial.println("Failed to read JPEG size");
		free(jpg_bu);
		encoder.close();
		return 3;
	}

	// alloc memoire RGB888 based on source JPEG size (safe allocation)
	rgb_len = (size_t)Jwidth * (size_t)Jheight * 3 * sizeof(uint8_t);
	rgb_buf = (uint8_t*)malloc(rgb_len);
	Serial.println("EnCCC");
	if (rgb_buf)
		memset(rgb_buf, 0, rgb_len);
	else {
		free(jpg_bu);
		encoder.close();
		return 9;
	}

	// Decode JPEG to RGB using SDK-style 4-arg fmt2rgb888
	if (!fmt2rgb888(jpg_bu, jpg_len, PIXFORMAT_JPEG, rgb_buf)) {
		Serial.println("JPEG decode failed");
		free(jpg_bu);
		free(rgb_buf);
		encoder.close();
		return 4;
	}
	Serial.printf("width:%i height:%i\n", Jwidth, Jheight);
	free(jpg_bu);

   // 3. encoder RGB888 → LPC

	encoder.encode_frame(rgb_buf);
	Serial.println("EnEEE");
	encoder.close();
	Serial.println("Encoding done");
	free(rgb_buf);
	printMemoryStatus();

	return 0;
}


uint8_t decode_lpc(const char *path_in, uint8_t qual_decod)
{
	if (strcmp(get_filename_ext(path_in), "lpc") != 0)
	{
		Serial.println("Input file n'est pas un fichier LPC");
		return 1;
	}	

	// remplacement .lpc par .jpg  Attention : les repertoires ne doivent pas contenir de '.'
	String output_path = String(path_in);
	int pos = output_path.lastIndexOf('.');
	if (pos >= 0)
		output_path = output_path.substring(0, pos) + ".avi";
	else
		output_path += ".avi";

	if (SD_MMC.exists(output_path)) {
			Serial.println("suppression du fichier de sortie existant");
			SD_MMC.remove(output_path);
	}


	filestream_t stream_input(SD_MMC, path_in, FILE_READ);

	lpc_decoder_t decoder;
	decoder.open(&stream_input);

	const lpc_settings_t settings = decoder.get_settings();
	printf("Decoding %s %d frames of size (%dx%d)\n", output_path.c_str(),settings.frame_count, settings.width, settings.height);

	int res = lpc_to_avi(decoder, path_in, output_path.c_str());

	/*// buffer image RGB
	img_data_t img_rgb(settings.width, settings.height);

	// décodage LPC → RGB
	decoder.decode_frame(img_rgb.bytes);

	filestream_t stream_output(SD_MMC, output_path.c_str(), FILE_APPEND);

	// encodage RGB -> JPEG
    auto write_cb = [](void *ctx, void *data, int size)
    {
        lpc_stream_out_t *stream = (lpc_stream_out_t *)ctx;
		stream->write_bytes((uint8_t *)data, size);
    };

    int res = tje_encode_with_func(
        write_cb,
        &stream_output,
        settings.quality,   // qualité JPEG depuis LPC settings
        settings.width,
        settings.height,
        3,                  // RGB
        img_rgb.bytes
    );*/

    if (res != 0)
    {
        Serial.println("Erreur encodage JPEG");
        return 2;
    }

    Serial.println("Decode LPC → JPEG OK");
    return 0;
}

/// MAIN LOOP

uint8_t lpc_to_avi(lpc_decoder_t &lpc, const char *input_path, const char *output_path)
{
	const lpc_settings_t settings = lpc.get_settings();

	std::string frame_path = input_path;
	frame_path.replace(frame_path.find_last_of('.'), std::string::npos, "_frame_");

	Serial.printf("Encodage A: %s width=%d height=%d quality=%d\n", output_path, settings.width, settings.height, settings.quality);

	uint32_t microsec_per_frame = (settings.frequency > 0) ? (1000000u / settings.frequency) : 1000000u;
	struct mjpegw_context *avi = mjpegw_open(output_path, settings.width, settings.height, microsec_per_frame, NULL);
	if(!avi)
	{
		Serial.println("mjpegw_open FAILED");
		return 1;
	}

	Serial.printf("avi=%p\n", avi);
	debug_ctx(avi);

	{
		img_data_t img_rgb(settings.width, settings.height);
		for (int i = 0; i < settings.frame_count; i++)
		{
			lpc.decode_frame(img_rgb.bytes);

			img_rgb.dump_bmp((frame_path + std::to_string(i) + ".bmp").c_str());
			Serial.println("AB5");

			Serial.printf("avi=%p\n", avi);
			Serial.printf("pixels=%p\n", img_rgb.bytes);

			mjpegw_add_frame(avi, img_rgb.bytes, 3);
			Serial.println("AB6");
		}
	}
	mjpegw_close(avi);
	return 0;
}

uint8_t lpc_to_jpg(lpc_decoder_t &lpc, const char *input_path, const char *output_path)
{
	const lpc_settings_t settings = lpc.get_settings();

	std::string frame_path = input_path;
	frame_path.replace(frame_path.find_last_of('.'), std::string::npos, "_frame_");

	uint32_t microsec_per_frame2 = (settings.frequency > 0) ? (1000000u / settings.frequency) : 1000000u;
	struct mjpegw_context *avi = mjpegw_open(output_path, settings.width, settings.height, microsec_per_frame2, NULL);
	{
		img_data_t img_rgb(settings.width, settings.height);
		for (int i = 0; i < settings.frame_count; i++)
		{
			lpc.decode_frame(img_rgb.bytes);

			img_rgb.dump_bmp((frame_path + std::to_string(i) + ".bmp").c_str());

			mjpegw_add_frame(avi, img_rgb.bytes, 3);
		}
	}
	mjpegw_close(avi);
	return 0;
}

//fonction plus robuste mais moins rapide que getJpegSize
bool get_jpeg_size(const char *path, uint16_t *width, uint16_t *height)
{
	#if ESP32
	filestream_t stream(SD_MMC, path);

    while (!stream.empty())
    {
        uint8_t marker = stream.read_byte();

        if (marker != 0xFF)
            continue;

        // lire vrai marqueur (skip padding 0xFF répétés)
        do {
            if (stream.empty()) return false;
            marker = stream.read_byte();
        } while (marker == 0xFF);

        // SOI (Start Of Image)
        if (marker == 0xD8) continue;

        // EOI (End Of Image)
        if (marker == 0xD9) break;

        // restart markers (RST0 - RST7)
        if (marker >= 0xD0 && marker <= 0xD7) continue;

        // marqueur sans payload
        if (marker == 0x01) continue;

        // longueur du segment (inclut les 2 bytes de length)
        uint16_t len = stream.read_uint16();
        if (len < 2) return false;

        // SOF0 (baseline JPEG) ou SOF2 (progressive JPEG)
        if (marker == 0xC0 || marker == 0xC2)
        {
            stream.read_byte(); // precision (bits par composant)

            // hauteur et largeur sont stockées en big endian
            *height = stream.read_uint16();
            *width  = stream.read_uint16();

            return true;
        }

        // skip du reste du segment
        for (uint16_t i = 0; i < len - 2; i++)
            stream.read_byte();
    }

    return false;

	#else

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
			return true;
        }

		stream.read_bytes(NULL, len - 2);
    }
	return false;

	#endif	
}

/*int main(int argc, const char **argv)
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

			lpc_to_avi(decoder, argv[1]);
		}
		else // encode
		{
			lpc_settings_t settings =
			{
				.quality = (uint8_t)10,
				.frame_count = 1,
				.frequency = 2
			};
			get_jpeg_size(argv[1], &settings.width, &settings.height);
			printf("Encoding '%s' (%dx%d) with quality = %d\n", argv[1], settings.width, settings.height, settings.quality);

			std::string output = argv[1];
			output.replace(output.find_last_of('.'), std::string::npos, ".lpc");
			filestream_t stream(output.c_str(), "wb");

			lpc_encoder_t encoder;
			encoder.open(settings, &stream);

			filestream_t jpeg(argv[1], "rb");
			encoder.encode_jpeg(&jpeg);

			encoder.close();
		}

		return 0;
	}

	int type = IMG_LARGE;
	uint32_t img_count = (type == IMG_NORMAL) ? 14 : 12;

	const char *output_path = "bin/large.lpc";
	lpc_settings_t settings =
	{
		.width = 800,
		.height = 600,
		.quality = (uint8_t)10,
		.frequency = 2
	};

	int actions = 0;
	if (argc > 1)
	{
		std::string cmd = argv[1];
		for (int i = 2; i < argc; i++)
			cmd += " " + std::string(argv[i]);
		actions = parse_cmd(cmd.c_str(), &settings.quality);
	}
	bool interactive = actions == 0;

	do {

		while (actions == 0)
		{
			printf("What to do ?\n");
			printf(" 1. Run encoder\n");
			printf(" 2. Run decoder\n");
			printf(" 3. Display encoding stats\n");
			printf(" 4. Set encoding quality\n");
			LPC_DEBUG_ONLY(printf(" 5. Run unit tests\n"));
			LPC_DEBUG_ONLY(printf(" 6. Run encoder with procedural image\n"));
			printf(" q. Exit\n");
			printf(" > ");

			std::string answer;
			std::getline(std::cin, answer);
			actions = parse_cmd(answer.c_str(), &settings.quality);
		}

		printf("\n");

		if (actions & g_exit)
		{
			interactive = false;
		}

		if (actions & g_run_encode)
		{
			#if ESP32
			filestream_t stream(fs, output_path, FILE_APPEND);
			#else
			filestream_t stream(output_path, "wb");
			#endif

			settings.frame_count = 1;

			lpc_encoder_t encoder;
			encoder.open(settings, &stream);

			for (uint32_t i = 0; i < settings.frame_count; ++i)
			{
				filestream_t jpeg(get_img(i, type), "rb");
				encoder.encode_jpeg(&jpeg);

				#ifdef LPC_DEBUG
				if (actions & g_display_stats)
					encoder.stats.print();
				#endif
			}

			encoder.close();
		}

		#if ESP32 == 0
		if (actions & g_run_decode)
		{
			printf("\n>>>>> Decoding frames %s\n", output_path);

			filestream_t stream(output_path, "rb");

			lpc_decoder_t decoder;
			decoder.open(&stream);

			img_data_t img_rgb(decoder.get_settings().width, decoder.get_settings().height);
			for (int frame = 0; frame < decoder.get_settings().frame_count; frame++)
			{
				decoder.decode_frame(img_rgb.bytes);
				img_rgb.dump_bmp(("bin/decoded_" + std::to_string(frame) + ".bmp").c_str());
			}
		}
		#endif

		#ifdef LPC_DEBUG
		if (actions & g_run_unit_tests)
		{
			lpc_unit_tests::run();
		}
		#endif

		if (actions & g_procedural_img)
		{
			{
				img_data_t img_rgb(settings.width, settings.height);

				filestream_t stream("bin/procedural.lpc", "wb");
				settings.frame_count = 3;

				lpc_encoder_t encoder;
				encoder.open(settings, &stream);

				for (int frame = 0; frame < settings.frame_count; frame++)
				{
					// Make a gradient
					for (int i = 0; i < settings.width; i++)
					for (int j = 0; j < settings.height; j++)
					{
						int idx = (i + j * settings.width) * 3;
						img_rgb.bytes[idx + 0] = 1;
						img_rgb.bytes[idx + 1] = 1;
						img_rgb.bytes[idx + 2] = 1;
						img_rgb.bytes[idx + min(frame, 3)] = i * 255 / settings.width;
					}
					encoder.encode_frame(img_rgb.bytes);
				}

				encoder.close();
			}
			{
				filestream_t stream("bin/procedural.lpc", "rb");
				lpc_decoder_t decoder;
				decoder.open(&stream);

				img_data_t img_rgb(settings.width, settings.height);
				for (int frame = 0; frame < settings.frame_count; frame++)
				{
					decoder.decode_frame(img_rgb.bytes);
					img_rgb.dump_bmp(("bin/procedural_" + std::to_string(frame) + ".bmp").c_str());
				}
			}
		}

		if (interactive)
		{
			actions = 0;
			printf("\n\n");
		}
	}
	while (interactive);
}*/


uint8_t encodeFile()
{
	unsigned long start_enc = millis();

	uint8_t nb_images_orig, nb_images, qualite_orig, qualite;
	uint16_t width_orig, height_orig, width, height;

	uint8_t code_images, code_width, code_compression;

	global_to_triplet(code_encod, code_images, code_width, code_compression);  // G -> 2-4-3
	Serial.printf("code_encod=%c code_images=%i code_width=%i code_compression=%i\n", code_encod, code_images, code_width, code_compression);

	nb_images = code_to_nbIm(code_images); // Code 2 -> nb_images=2

	framesize_t cam_code;
	code_to_size( code_width , width, cam_code);  // Code 4 -> width=800

    uint8_t txCam;
    code_to_compjpg( code_compression , qualite, txCam); // Code 3 -> qualite=15

	uint8_t type_fichier = 0;  // 0:jpg  1:avi
	//  Attention : les repertoires ne doivent pas contenir de '.'
	if (strcmp(get_filename_ext(path_c), "jpg") != 0)
	{
		type_fichier = 1;
		if (strcmp(get_filename_ext(path_c), "avi") != 0)
		{
			Serial.println("Input file n'est ni un fichier JPEG ni un fichier AVI");
			return 1;
		}
	}		

	String output_path = String(path_c);
	int pos = output_path.lastIndexOf('.');
	if (pos != -1) {
		output_path = output_path.substring(0, pos);  // CA_260303_140202_R_543
	}
	else return 2;

	if (output_path.length() >= 5) {
		output_path.remove(output_path.length() - 5); // CA_260303_140202_
	}
	else return 3;

	// --- 1. ouvrir fichier source ---
	File inFile = SD_MMC.open(path_c, FILE_READ);
	if (!inFile) {
		Serial.println("Failed to open input file");
		return 1;
	}

	size_t fileSize = inFile.size();

	if (type_fichier) // AVI
	{
		nb_images_orig = 2;
		width_orig = 800;
		height_orig = 600;
		qualite_orig = 30;

	}
	else  // JPEG
	{
        nb_images_orig = 1;

		uint8_t* jpg_bu = (uint8_t*)malloc(fileSize);

		if (!jpg_bu) {
			Serial.println("Malloc failed");
			inFile.close();
			return 2;
		}

		inFile.read(jpg_bu, fileSize);
		inFile.close();

		// recuperation de l w et h de l'image source.
		uint16_t w = 0, h = 0;

		if ((!getJpegSize(jpg_bu, fileSize, w, h)) || (w<50) || (h<50) || (w>2500) || (h>2000)) {
			Serial.println("Failed to read JPEG size");
			free(jpg_bu);
			return 3;
		}
		free(jpg_bu);
		// Calcul de la qualité de compression à partir de la taille du fichier et des dimensions de l'image
		qualite_orig = 20;
		if (w && h) 
		{
			float qual = ((float)fileSize * 8.0f / (float)(w * h) - 0.15f) / 2.4f;
			if (qual > 0)
				qualite_orig = (uint8_t)(sqrt(qual) * 100);
		}
	}

	// les nouvelles valeurs doivent être plus petites que les anciennes
	if (nb_images > nb_images_orig) nb_images = nb_images_orig;
	if (width > width_orig) width = width_orig;
	//if (qualite > qualite_orig) qualite = qualite_orig;
    height = (float)height_orig * ((float)width / (float)width_orig);  // calcul de la nouvelle hauteur pour garder le ratio

	Serial.printf("Encoding %s Nb_images: %i -> %i\n", path_c, nb_images_orig, nb_images);
	Serial.printf("Encoding type:%i  Format : %u x %u -> %u x %u\n", type_fichier, width_orig, height_orig, width, height);
	Serial.printf("Encoding  Qualite : %i -> %i\n", qualite_orig, qualite);

	lpc_settings_t settings = {
		width,    // width
		height,    // height
		qualite,     // quality
		nb_images,      // frame_count
		1,       // frequency
	};

	uint8_t code_im = nbIm_to_code (nb_images);
	uint8_t code_si;
	framesize_t cam_size;
	size_to_code(width, cam_size, code_si);
	uint8_t code_qual;
	tx_compjpg_to_code(qualite, txCam, code_qual);
	uint8_t code_global = triplet_to_global(code_im, code_si, code_qual);

	output_path  = output_path + String((char)code_global) + '-' + String(code_im) + String(code_si) + String(code_qual) + ".lpc";

	if (SD_MMC.exists(output_path)) {
			Serial.println("suppression du fichier de sortie existant");
			SD_MMC.remove(output_path);
	}

	Serial.printf("Encodage vers %s width=%d height=%d quality=%d\n", output_path.c_str(), settings.width, settings.height, settings.quality);

	filestream_t stream_out(SD_MMC, output_path.c_str(), FILE_APPEND);
	jpeg_reader_t jpeg(SD_MMC, path_c);   // lecture du fichier. Renvoie le nb d'octets lus
	lpc_encoder_t encoder;
	unsigned long enc2 = millis();
	encoder.open(settings, &stream_out);

	encoder.encode_jpeg(&jpeg);
	unsigned long enc3 = millis();
	encoder.close();
	unsigned long end_enc = millis();
	Serial.printf("Encoding time: prep:%lu ms, encode:%lu ms, close:%lu ms\n", enc2-start_enc, enc3-enc2, end_enc-enc3);
	return 0;
}