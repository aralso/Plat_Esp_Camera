#define ESP32 1

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

// Reader that extracts the nth MJPEG frame from an AVI (searches for JPEG SOI/EOI markers)
struct avi_frame_reader_t : public lpc_stream_in_t
{
    File file;
    uint32_t start_pos;
    uint32_t end_pos;
    uint32_t remaining;

    avi_frame_reader_t(fs::FS &fs, const char *path, uint32_t index)
    {
        file = fs.open(path, FILE_READ);
        assert(file);
        start_pos = 0;
        end_pos = 0;
        remaining = 0;

        // Find the index-th JPEG SOI (0xFFD8)
        uint8_t prev = 0;
        uint32_t frame_count = 0;
        file.seek(0);
        while (file.available()) {
            uint8_t b = file.read();
            if (prev == 0xFF && b == 0xD8) {
                if (frame_count == index) {
                    start_pos = file.position() - 2;
                    break;
                }
                frame_count++;
            }
            prev = b;
        }

        if (start_pos != 0 || (start_pos == 0 && index == 0)) {
            // find EOI 0xFFD9
            prev = 0;
            file.seek(start_pos);
            while (file.available()) {
                uint8_t b = file.read();
                if (prev == 0xFF && b == 0xD9) {
                    end_pos = file.position();
                    break;
                }
                prev = b;
            }
        }

        if (end_pos > start_pos) {
            remaining = end_pos - start_pos;
            file.seek(start_pos);
        } else {
            // not found - set remaining to 0
            remaining = 0;
        }
    }

    ~avi_frame_reader_t()
    {
        file.close();
    }

    size_t read(uint8_t *data, size_t size) override
    {
        if (remaining == 0) return 0;
        size_t toRead = (size_t)(remaining < size ? remaining : size);
        size_t r = file.read(data, toRead);
        if (r > 0) remaining -= r;
        return r;
    }
};

// Lightweight AVI MJPEG metadata probe used to extract frame count, dimensions
// and, when present, the stream quality from the container headers.
#pragma pack(push, 1)
struct avi_chunk_header_t
{
    char id[4];
    uint32_t size;
};

struct avi_avih_chunk_t
{
    uint32_t microsec_per_frame;
    uint32_t max_bytes_per_sec;
    uint32_t padding_granularity;
    uint32_t flags;
    uint32_t total_frames;
    uint32_t initial_frames;
    uint32_t streams;
    uint32_t suggested_buffer_size;
    uint32_t width;
    uint32_t height;
    uint32_t reserved[4];
};

struct avi_strh_chunk_t
{
    char type[4];
    char handler[4];
    uint32_t flags;
    uint32_t priority;
    uint16_t language;
    uint16_t initial_frames;
    uint32_t scale;
    uint32_t rate;
    uint32_t start;
    uint32_t length;
    uint32_t suggested_buffer_size;
    uint32_t quality;
    uint32_t sample_size;
    struct
    {
        int16_t left;
        int16_t top;
        int16_t right;
        int16_t bottom;
    } frame;
};

struct avi_strf_chunk_t
{
    uint32_t bi_size;
    int32_t bi_width;
    int32_t bi_height;
    uint16_t bi_planes;
    uint16_t bi_bit_count;
    uint32_t bi_compression;
    uint32_t bi_image_size;
    int32_t bi_x_ppm;
    int32_t bi_y_ppm;
    uint32_t bi_clr_used;
    uint32_t bi_clr_important;
};

struct avi_idx1_entry_t
{
    char id[4];
    uint32_t flags;
    uint32_t offset;
    uint32_t size;
};
#pragma pack(pop)

struct avi_probe_info_t
{
    uint32_t avih_total_frames = 0;
    uint32_t strh_total_frames = 0;
    uint32_t idx1_total_frames = 0;
    uint32_t movi_total_frames = 0;
    uint32_t width_avih = 0;
    uint32_t height_avih = 0;
    int32_t width_strf = 0;
    int32_t height_strf = 0;
    uint32_t quality_raw = 0xFFFFFFFFu;
    bool is_mjpeg = false;
};

static bool read_file_exact(File &file, void *buffer, size_t length)
{
    return file.read((uint8_t *)buffer, length) == length;
}

static bool is_avi_video_chunk(const char id[4])
{
    return (id[2] == 'd') && ((id[3] == 'b') || (id[3] == 'c'));
}

static bool parse_avi_chunks(File &file, uint32_t range_end, avi_probe_info_t &info)
{
    while ((uint32_t)file.position() + sizeof(avi_chunk_header_t) <= range_end)
    {
        uint32_t chunk_start = (uint32_t)file.position();
        avi_chunk_header_t chunk;
        if (!read_file_exact(file, &chunk, sizeof(chunk))) {
            Serial.printf("PARSE_FAIL: short read chunk header at pos=%u, expected=%u\n", (unsigned)file.position(), (unsigned)sizeof(chunk));
            return false;
        }

        uint32_t data_start = (uint32_t)file.position();
        uint64_t data_end64 = (uint64_t)data_start + (uint64_t)chunk.size;
        if (data_end64 > range_end) return false;

        uint32_t data_end = (uint32_t)data_end64;
        uint32_t padded_end = data_end + (chunk.size & 1u);

        if ((memcmp(chunk.id, "RIFF", 4) == 0) || (memcmp(chunk.id, "LIST", 4) == 0))
        {
            char list_type[4];
            if (chunk.size < 4) {
                Serial.printf("PARSE_FAIL: LIST chunk too small at pos=%u size=%u\n", (unsigned)data_start, (unsigned)chunk.size);
                return false;
            }
            if (!read_file_exact(file, list_type, sizeof(list_type))) {
                Serial.printf("PARSE_FAIL: short read LIST type at pos=%u expected %u\n", (unsigned)file.position(), (unsigned)sizeof(list_type));
                return false;
            }
            if (!parse_avi_chunks(file, data_end, info)) {
                Serial.printf("PARSE_FAIL: nested parse failed for LIST at data_end=%u\n", (unsigned)data_end);
                return false;
            }
        }
        else if ((memcmp(chunk.id, "avih", 4) == 0) && (chunk.size >= sizeof(avi_avih_chunk_t)))
        {
            avi_avih_chunk_t avih;
            if (!read_file_exact(file, &avih, sizeof(avih))) {
                Serial.printf("PARSE_FAIL: short read avih at pos=%u expected=%u\n", (unsigned)file.position(), (unsigned)sizeof(avih));
                return false;
            }
            info.avih_total_frames = avih.total_frames;
            info.width_avih = avih.width;
            info.height_avih = avih.height;
        }
        else if ((memcmp(chunk.id, "strh", 4) == 0) && (chunk.size >= sizeof(avi_strh_chunk_t)))
        {
            avi_strh_chunk_t strh;
            if (!read_file_exact(file, &strh, sizeof(strh))) {
                Serial.printf("PARSE_FAIL: short read strh at pos=%u expected=%u\n", (unsigned)file.position(), (unsigned)sizeof(strh));
                return false;
            }
            if (memcmp(strh.type, "vids", 4) == 0)
            {
                info.strh_total_frames = strh.length;
                info.quality_raw = strh.quality;
                if (memcmp(strh.handler, "MJPG", 4) == 0) info.is_mjpeg = true;
            }
        }
        else if ((memcmp(chunk.id, "strf", 4) == 0) && (chunk.size >= sizeof(avi_strf_chunk_t)))
        {
            avi_strf_chunk_t strf;
            if (!read_file_exact(file, &strf, sizeof(strf))) {
                Serial.printf("PARSE_FAIL: short read strf at pos=%u expected=%u\n", (unsigned)file.position(), (unsigned)sizeof(strf));
                return false;
            }
            info.width_strf = strf.bi_width;
            info.height_strf = strf.bi_height;
            if (strf.bi_compression == 0x47504A4Du) info.is_mjpeg = true;
        }
        else if ((memcmp(chunk.id, "idx1", 4) == 0) && (chunk.size >= sizeof(avi_idx1_entry_t)))
        {
            uint32_t entry_count = chunk.size / sizeof(avi_idx1_entry_t);
            for (uint32_t i = 0; i < entry_count; ++i)
            {
                avi_idx1_entry_t entry;
                if (!read_file_exact(file, &entry, sizeof(entry))) {
                    Serial.printf("PARSE_FAIL: short read idx1 entry at pos=%u expected=%u (i=%u of %u)\n", (unsigned)file.position(), (unsigned)sizeof(entry), (unsigned)i, (unsigned)entry_count);
                    return false;
                }
                if (is_avi_video_chunk(entry.id)) info.idx1_total_frames++;
            }
        }
        else if (is_avi_video_chunk(chunk.id))
        {
            info.movi_total_frames++;
        }

        if (!file.seek(padded_end)) {
            Serial.printf("PARSE_FAIL: seek to padded_end failed. padded_end=%u file.pos=%u\n", (unsigned)padded_end, (unsigned)file.position());
            return false;
        }
        if ((uint32_t)file.position() <= chunk_start) {
            Serial.printf("PARSE_FAIL: file position did not advance. chunk_start=%u file.pos=%u\n", (unsigned)chunk_start, (unsigned)file.position());
            return false;
        }
    }

    uint32_t final_pos = (uint32_t)file.position();
    if (final_pos != range_end) {
        Serial.printf("PARSE_END: final_pos=%u range_end=%u\n", final_pos, (unsigned)range_end);
    }
    return (final_pos == range_end);
}

static uint8_t probe_avi_mjpeg(File &file, size_t file_size, uint32_t &frame_count, uint16_t &width, uint16_t &height, uint8_t &quality)
{
	uint8_t res=0;  // 0:echec  2à9:autres erreurs   10:reussi 11à14:dégradé
    frame_count = 0;
    width = 0;
    height = 0;
    quality = 20;

    if (!file.seek(0)) return false;

    avi_chunk_header_t riff;
    if (!read_file_exact(file, &riff, sizeof(riff))) return false;
    if ((memcmp(riff.id, "RIFF", 4) != 0) || (riff.size < 4)) return false;

    char riff_type[4];
    if (!read_file_exact(file, riff_type, sizeof(riff_type))) return false;
    if (memcmp(riff_type, "AVI ", 4) != 0) return false;

    avi_probe_info_t info;
    // Serial.printf("PROBE: calling parse_avi_chunks file.pos=%u range_end=%u\n", (unsigned)file.position(), (unsigned)file_size);
    if (!parse_avi_chunks(file, (uint32_t)file_size, info)) {
        //Serial.printf("PROBE_FAIL: parse_avi_chunks returned false at pos=%u range_end=%u\n", (unsigned)file.position(), (unsigned)file_size);
        return false;
    }

    uint32_t parsed_frames = info.idx1_total_frames;
    if (parsed_frames == 0) parsed_frames = info.avih_total_frames;
    if (parsed_frames == 0) parsed_frames = info.strh_total_frames;
    if (parsed_frames == 0) parsed_frames = info.movi_total_frames;
	//Serial.printf("AVI probe: frames=%u, idx1=%u, avih=%u, strh=%d, movi=%d\n",
	//	parsed_frames, info.idx1_total_frames, info.avih_total_frames, info.strh_total_frames, info.movi_total_frames);

    uint32_t parsed_width = 0;
    uint32_t parsed_height = 0;
    if ((info.width_strf > 0) && (info.height_strf != 0))
    {
        parsed_width = (uint32_t)info.width_strf;
        parsed_height = (uint32_t)((info.height_strf < 0) ? -info.height_strf : info.height_strf);
    }
    else
    {
        parsed_width = info.width_avih;
        parsed_height = info.height_avih;
    }

    if ((parsed_frames == 0) || (parsed_width < 50) || (parsed_height < 50) || (parsed_width > 2500) || (parsed_height > 2000))
        return false;

    frame_count = parsed_frames;
    width = (uint16_t)parsed_width;
    height = (uint16_t)parsed_height;

	Serial.printf("quality:%i\n", info.quality_raw);
    if ((info.quality_raw != 0) && (info.quality_raw != 0xFFFFFFFFu))
    {
        uint32_t quality_value = info.quality_raw;
        if (quality_value > 100u) quality_value = (quality_value + 50u) / 100u;
        if (quality_value > 100u) quality_value = 100u;
        quality = (uint8_t)quality_value;
		res=10;
		//Serial.println("0HH");
    }
    else
    {
        float bpp = (float)file_size * 8.0f / ((float)parsed_frames * (float)parsed_width * (float)parsed_height);
        float qual = 1.0f - expf(-3.0f * bpp);
        if (qual < 0.0f) qual = 0.0f;
        if (qual > 1.0f) qual = 1.0f;
        quality = (uint8_t)(qual * 100.0f);
		res = 11;
		Serial.println("0II");
    }

	Serial.printf("AVI probe: frames=%u, width=%u, height=%u, quality=%u\n", frame_count, width, height, quality);
    return res;
}


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
const int g_run_profile		= 1 << 6;
const int g_exit 			= 1 << 31;

uint8_t lpc_to_avi(lpc_decoder_t &lpc, const char *input_path, const char *output_path);

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

uint8_t encodeFile()
{
	unsigned long start_enc = millis();

	uint8_t nb_images_orig=0, nb_images, qualite_orig=0, qualite;
	uint16_t width_orig=0, height_orig=0, width, height;

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
		// Probe AVI headers to obtain frame count, width, height and stream quality when available
		uint32_t parsed_frames = 0;
		uint16_t parsed_width = 0, parsed_height = 0;
		uint8_t parsed_quality = 20;
		// inFile is already opened above
		uint8_t probe_result = probe_avi_mjpeg(inFile, fileSize, parsed_frames, parsed_width, parsed_height, parsed_quality);	
		if ((probe_result >= 10))
		{
			nb_images_orig = (int)parsed_frames;
			width_orig = parsed_width;
			height_orig = parsed_height;
			qualite_orig = parsed_quality;
			Serial.printf("probe: res:%i nb_image:%i\n %i*%i qual:%i\n", probe_result, nb_images_orig, width_orig, height_orig, qualite_orig);
		}
		else
		{
			Serial.printf("probe echec: res:%i nb_image:%i\n", probe_result, nb_images_orig);
			// Fallback: extract first MJPEG frame from the AVI and inspect its JPEG header for size/quality
			avi_frame_reader_t afr(SD_MMC, path_c, 0);
			if (afr.remaining > 0)
			{
				uint32_t frlen = afr.remaining;
				uint8_t *frame_buf = (uint8_t*)malloc(frlen);
				if (frame_buf)
				{
					// read full frame
					size_t total = 0;
					while (total < frlen) {
						size_t r = afr.read(frame_buf + total, frlen - total);
						if (r == 0) break;
						total += r;
					}
					uint16_t w=0, h=0;
					if (getJpegSize(frame_buf, total, w, h))
					{
						nb_images_orig = 1; // at least one frame
						width_orig = w;
						height_orig = h;
						Serial.printf("frame_buf total:%i wi:%i he:%i\n", total, w, h);
						// Try to extract quality from COM/APP markers inside the JPEG frame
						auto extract_quality = [](const uint8_t *buf, size_t len)->int {
							if (!buf || len < 4) return -1;
							if (buf[0] != 0xFF || buf[1] != 0xD8) return -1;
							size_t pos = 2;
							while (pos + 3 < len)
							{
								if (buf[pos] != 0xFF) { pos++; continue; }
								uint8_t marker = buf[pos+1];
								if (marker == 0xDA) break; // SOS
								if (marker == 0xD8 || marker == 0xD9) { pos += 2; continue; }
								if (pos + 4 > len) break;
								uint16_t mlen = (uint16_t)((buf[pos+2] << 8) | buf[pos+3]);
								if (mlen < 2) return -1;
								if (pos + 2 + mlen > len) break;
								const uint8_t *payload = &buf[pos+4];
								int payload_len = (int)mlen - 2;
								// COM marker (0xFE) - look for ASCII form "Q=NN"
								if (marker == 0xFE && payload_len >= 3) {
									if ((payload[0] == 'Q' || payload[0] == 'q') && payload[1] == '=') {
										int plen = payload_len - 2;
										char tmp[16];
										int copy_len = (plen < (int)sizeof(tmp)-1) ? plen : (int)sizeof(tmp)-1;
										memcpy(tmp, payload+2, copy_len);
										tmp[copy_len] = '\0';
										int q = atoi(tmp);
										if (q >= 0 && q <= 100) return q;
									}
								}
								// APPn marker: look for signature "QVAL" followed by one byte quality
								if (marker >= 0xE0 && marker <= 0xEF && payload_len >= 5) {
									if (memcmp(payload, "QVAL", 4) == 0) {
										int q = payload[4];
										if (q >= 0 && q <= 100) return q;
									}
								}
								pos += 2 + mlen;
							}
							return -1;
						};
						int q = extract_quality(frame_buf, total);
						if (q >= 0)
						{
							qualite_orig = (uint8_t)q;
							Serial.println("BBB");
						}
						else
						{
							// Fallback to probe heuristic similar to probe_avi_mjpeg: estimate using file total size (global)
							if (parsed_frames > 0 && width_orig > 0 && height_orig > 0) {
								float bpp = (float)fileSize * 8.0f / ((float)parsed_frames * (float)width_orig * (float)height_orig);
								float qualf = 1.0f - expf(-3.0f * bpp);
								if (qualf < 0.0f) qualf = 0.0f;
								if (qualf > 1.0f) qualf = 1.0f;
								qualite_orig = (uint8_t)(qualf * 100.0f);
								Serial.println("CCC");
							} else {
								// last resort
								qualite_orig = 30;
								Serial.println("DDD");
							}
						}
					}
					else 
						Serial.println("getjpegsize failed");

					free(frame_buf);
				}
				else
				{
					// malloc failed: set reasonable defaults
					nb_images_orig = 1;
					width_orig = 800;
					height_orig = 600;
					qualite_orig = 30;
					Serial.printf("Fallback malloc failed : long:%i code:134\n", frlen);
				}
			}
			else
			{
				// no frames found: fallback defaults
				nb_images_orig = 0;
				width_orig = 800;
				height_orig = 600;
				qualite_orig = 20;
				Serial.println("no frame found : nb_images=0");
			}
		}
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
		width_orig = w;
		height_orig = h;

		// Calcul de la qualité de compression à partir d'un marqueur JPEG si présent, sinon par estimation bpp
		qualite_orig = 20;
		if (w && h) 
		{
			// Try to extract embedded quality from JPEG COM/APP marker
			auto extract_quality = [](const uint8_t *buf, size_t len)->int {
				if (!buf || len < 4) return -1;
				// Ensure starts with SOI
				if (buf[0] != 0xFF || buf[1] != 0xD8) return -1;
				size_t pos = 2; // after SOI
				while (pos + 3 < len) {
					if (buf[pos] != 0xFF) { pos++; continue; }
					uint8_t marker = buf[pos+1];
					// SOS (start of scan) -> stop parsing headers
					if (marker == 0xDA) break;
					// Standalone markers without length
					if (marker == 0xD8 || marker == 0xD9) { pos += 2; continue; }
					// Markers with length
					if (pos + 4 > len) break;
					uint16_t mlen = (uint16_t)((buf[pos+2] << 8) | buf[pos+3]);
					if (mlen < 2) return -1;
					if (pos + 2 + mlen > len) break;
					const uint8_t *payload = &buf[pos+4];
					int payload_len = (int)mlen - 2;
					// COM marker (0xFE) - look for ASCII form "Q=NN"
					if (marker == 0xFE && payload_len >= 3) {
						if ((payload[0] == 'Q' || payload[0] == 'q') && payload[1] == '=') {
							int plen = payload_len - 2;
							char tmp[16];
							int copy_len = (plen < (int)sizeof(tmp)-1) ? plen : (int)sizeof(tmp)-1;
							memcpy(tmp, payload+2, copy_len);
							tmp[copy_len] = '\0';
							int q = atoi(tmp);
							Serial.printf("Found COM marker quality: %d\n", q);
							if (q >= 0 && q <= 100) return q;
						}
					}
					// APPn marker: look for signature "QVAL" followed by one byte quality
					if (marker >= 0xE0 && marker <= 0xEF && payload_len >= 5) {
						if (memcmp(payload, "QVAL", 4) == 0) {
							int q = payload[4];
							if (q >= 0 && q <= 100) return q;
						}
					}
					pos += 2 + mlen;
				}
				return -1;
			};

			int q = extract_quality(jpg_bu, fileSize);
			if (q >= 0) {
				qualite_orig = (uint8_t)q;
				Serial.printf("Extracted quality from JPEG marker: %d\n", qualite_orig);
			} else {
				float bpp = (float)fileSize * 8.0f / (w * h);
				float qual = 1.0f - expf(-3.0f * bpp);
				qualite_orig = (uint8_t)(qual * 100.0f);
				Serial.printf("Estimated quality from bpp: %d\n", qualite_orig);
			}
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
		cap_interval_dsec,       // frequency
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
	lpc_encoder_t encoder;
	unsigned long enc2 = millis();
	encoder.open(settings, &stream_out);
	unsigned long encprec = millis();
	unsigned long enc3=0;

	for (uint32_t i = 0; i < settings.frame_count; ++i)
	{
		if (type_fichier) {
			avi_frame_reader_t avi(SD_MMC, path_c, i);
			encoder.encode_jpeg(&avi);
		} else {
			jpeg_reader_t jpeg(SD_MMC, path_c);   // lecture du fichier JPEG
			encoder.encode_jpeg(&jpeg);
		}

		enc3 = millis();
		
		#ifdef LPC_DEBUG
		float num_mb = (float)max(encoder.stats.num_macroblocks, 1);
		float pblocks = 100.0f*encoder.stats.num_block_match_pred / num_mb;
		Serial.printf("Encoding image:%i time:%lu ms p-blocks:%.1f%%\n", i, enc3-encprec, pblocks);
		#else
		Serial.printf("Encoding image:%i time:%lu ms\n", i, enc3-encprec);
		#endif

		encprec = enc3;

		#ifdef LPC_PROFILE
		for (int i = 0; i < LPC_MARKER_COUNT; i++)
		{
			lpc_profiler_t::stats_t &s = lpc_profiler_t::markers[i];
			if (s.count != 0)
			{
				for (int i = 0; i < s.nesting; i++) printf("  ");
				printf("- %-16s %10.3f\n", to_string((LPC_MARKER)i), (double)s.total / 1e6);
			}
		}
		#endif
	}
	encoder.close();
	unsigned long end_enc = millis();

	// determination de la taille du fichier écrit
	// TODO mieux : lire le nb d'octets écrits par le stream_out (stream_out.get_bytes_written() ?)
	uint32_t f_size = 0;
	{
		// Diagnostic prints to verify file path and size reporting
		Serial.printf("Checking output_path='%s'\n", output_path.c_str());
		Serial.printf("SD_MMC.exists=%d\n", SD_MMC.exists(output_path.c_str()) ? 1 : 0);

		// small delay to allow filesystem metadata to settle after writer close
		delay(100);
		File outFile = SD_MMC.open(output_path.c_str(), FILE_READ);
		if (!outFile) {
			Serial.println("outFile open failed");
		} else {
			// Some FS implementations update size() only after flush/close. Seek to end and read position for a reliable value.
			size_t s1 = outFile.size();
			outFile.seek(0, SeekEnd);
			size_t s2 = outFile.position();
			Serial.printf("outFile.size()=%lu, position(end)=%lu\n", (unsigned long)s1, (unsigned long)s2);
			f_size = s2;
			outFile.close();
			Serial.println("ZZ");
		}

		Serial.printf("Encoding size:%lu time prep:%lu ms, encode:%lu ms, close:%lu ms\n",
					(unsigned long)f_size,
					(unsigned long)(enc2 - start_enc),
					(unsigned long)(enc3 - enc2),
					(unsigned long)(end_enc - enc3));
	}	

	Serial.printf("Encoding size:%i time prep:%lu ms, encode:%lu ms, close:%lu ms\n", f_size, enc2-start_enc, enc3-enc2, end_enc-enc3);
	return 0;
}