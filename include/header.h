#pragma once

#include <unordered_map>
#include <vector>

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef _WIN32
#include <winsock.h>
#else
#include <sys/time.h>
#endif

#define JPGE_VERBOSE 0

#define heap_caps_malloc(a, b) malloc(a)
#define IRAM_ATTR

template<typename... Types>
static inline void ESP_LOGE(const char *tag, Types... types)
{
	printf("[ERROR] %s ");
	printf(types...);
}

template<typename... Types>
static inline void ESP_LOGW(const char *tag, Types... types)
{
	printf("[WARNING] %s ");
	printf(types...);
}

typedef enum {
	JPG_SCALE_NONE,
	JPG_SCALE_2X,
	JPG_SCALE_4X,
	JPG_SCALE_8X,
	JPG_SCALE_MAX = JPG_SCALE_8X
} jpg_scale_t;


typedef int32_t esp_err_t;

typedef size_t(*jpg_out_cb)(void*,size_t,const void*, int);

bool jpg2bmp(const uint8_t *src, size_t src_len, uint8_t ** out, size_t * out_len);

/* Definitions for error constants. */
#define ESP_OK          0       /*!< esp_err_t value indicating success (no error) */
#define ESP_FAIL        -1      /*!< Generic esp_err_t code indicating failure */

#define ESP_ERR_NO_MEM              0x101   /*!< Out of memory */
#define ESP_ERR_INVALID_ARG         0x102   /*!< Invalid argument */
#define ESP_ERR_INVALID_STATE       0x103   /*!< Invalid state */
#define ESP_ERR_INVALID_SIZE        0x104   /*!< Invalid size */
#define ESP_ERR_NOT_FOUND           0x105   /*!< Requested resource not found */
#define ESP_ERR_NOT_SUPPORTED       0x106   /*!< Operation or feature not supported */
#define ESP_ERR_TIMEOUT             0x107   /*!< Operation timed out */
#define ESP_ERR_INVALID_RESPONSE    0x108   /*!< Received response was invalid */
#define ESP_ERR_INVALID_CRC         0x109   /*!< CRC or checksum was invalid */
#define ESP_ERR_INVALID_VERSION     0x10A   /*!< Version was invalid */
#define ESP_ERR_INVALID_MAC         0x10B   /*!< MAC address was invalid */

#define ESP_ERR_WIFI_BASE           0x3000  /*!< Starting number of WiFi error codes */
#define ESP_ERR_MESH_BASE           0x4000  /*!< Starting number of MESH error codes */


#define MALLOC_CAP_EXEC             (1<<0)  ///< Memory must be able to run executable code
#define MALLOC_CAP_32BIT            (1<<1)  ///< Memory must allow for aligned 32-bit data accesses
#define MALLOC_CAP_8BIT             (1<<2)  ///< Memory must allow for 8/16/...-bit data accesses
#define MALLOC_CAP_DMA              (1<<3)  ///< Memory must be able to accessed by DMA
#define MALLOC_CAP_PID2             (1<<4)  ///< Memory must be mapped to PID2 memory space (PIDs are not currently used)
#define MALLOC_CAP_PID3             (1<<5)  ///< Memory must be mapped to PID3 memory space (PIDs are not currently used)
#define MALLOC_CAP_PID4             (1<<6)  ///< Memory must be mapped to PID4 memory space (PIDs are not currently used)
#define MALLOC_CAP_PID5             (1<<7)  ///< Memory must be mapped to PID5 memory space (PIDs are not currently used)
#define MALLOC_CAP_PID6             (1<<8)  ///< Memory must be mapped to PID6 memory space (PIDs are not currently used)
#define MALLOC_CAP_PID7             (1<<9)  ///< Memory must be mapped to PID7 memory space (PIDs are not currently used)
#define MALLOC_CAP_SPIRAM           (1<<10) ///< Memory must be in SPI RAM
#define MALLOC_CAP_INTERNAL         (1<<11) ///< Memory must be internal; specifically it should not disappear when flash/spiram cache is switched off
#define MALLOC_CAP_DEFAULT          (1<<12) ///< Memory can be returned in a non-capability-specific memory allocation (e.g. malloc(), calloc()) call
#define MALLOC_CAP_INVALID          (1<<31) ///< Memory can't be used / list end marker


#define OV9650_PID     (0x96)
#define OV7725_PID     (0x77)
#define OV2640_PID     (0x26)
#define OV3660_PID     (0x36)
#define OV5640_PID     (0x56)

typedef enum {
	PIXFORMAT_RGB565,    // 2BPP/RGB565
	PIXFORMAT_YUV422,    // 2BPP/YUV422
	PIXFORMAT_GRAYSCALE, // 1BPP/GRAYSCALE
	PIXFORMAT_JPEG,      // JPEG/COMPRESSED
	PIXFORMAT_RGB888,    // 3BPP/RGB888
	PIXFORMAT_RAW,       // RAW
	PIXFORMAT_RGB444,    // 3BP2P/RGB444
	PIXFORMAT_RGB555,    // 3BP2P/RGB555
} pixformat_t;

typedef enum {
	FRAMESIZE_96X96,    // 96x96
	FRAMESIZE_QQVGA,    // 160x120
	FRAMESIZE_QCIF,     // 176x144
	FRAMESIZE_HQVGA,    // 240x176
	FRAMESIZE_240X240,  // 240x240
	FRAMESIZE_QVGA,     // 320x240
	FRAMESIZE_CIF,      // 400x296
	FRAMESIZE_HVGA,     // 480x320
	FRAMESIZE_VGA,      // 640x480
	FRAMESIZE_SVGA,     // 800x600
	FRAMESIZE_XGA,      // 1024x768
	FRAMESIZE_HD,       // 1280x720
	FRAMESIZE_SXGA,     // 1280x1024
	FRAMESIZE_UXGA,     // 1600x1200
	// 3MP Sensors
	FRAMESIZE_FHD,      // 1920x1080
	FRAMESIZE_P_HD,     //  720x1280
	FRAMESIZE_P_3MP,    //  864x1536
	FRAMESIZE_QXGA,     // 2048x1536
	// 5MP Sensors
	FRAMESIZE_QHD,      // 2560x1440
	FRAMESIZE_WQXGA,    // 2560x1600
	FRAMESIZE_P_FHD,    // 1080x1920
	FRAMESIZE_QSXGA,    // 2560x1920
	FRAMESIZE_INVALID
} framesize_t;



typedef struct {
	uint8_t* buf;              /*!< Pointer to the pixel data */
	size_t len;                 /*!< Length of the buffer in bytes */
	size_t width;               /*!< Width of the buffer in pixels */
	size_t height;              /*!< Height of the buffer in pixels */
	pixformat_t format;         /*!< Format of the pixel data */
	struct timeval timestamp;   /*!< Timestamp since boot of the first DMA buffer of the frame */
} camera_fb_t;

#define ESP_ERR_CAMERA_BASE 0x20000
#define ESP_ERR_CAMERA_NOT_DETECTED             (ESP_ERR_CAMERA_BASE + 1)
#define ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE (ESP_ERR_CAMERA_BASE + 2)
#define ESP_ERR_CAMERA_FAILED_TO_SET_OUT_FORMAT (ESP_ERR_CAMERA_BASE + 3)
#define ESP_ERR_CAMERA_NOT_SUPPORTED            (ESP_ERR_CAMERA_BASE + 4)




typedef size_t(*jpg_reader_cb)(void* arg, size_t index, uint8_t* buf, size_t len);
typedef bool (*jpg_writer_cb)(void* arg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t* data);

esp_err_t esp_jpg_decode(size_t len, jpg_scale_t scale, jpg_reader_cb reader, jpg_writer_cb writer, void* arg);

#define	JD_SZBUF		512	/* Size of stream input buffer */
#define JD_FORMAT		0	/* Output pixel format 0:RGB888 (3 BYTE/pix), 1:RGB565 (1 WORD/pix) */
#define	JD_USE_SCALE	1	/* Use descaling feature for output */
#define JD_TBLCLIP		1	/* Use table for saturation (might be a bit faster but increases 1K bytes of code size) */

/*---------------------------------------------------------------------------*/

/* These types must be 16-bit, 32-bit or larger integer */
typedef int				INT;
typedef unsigned int	UINT;

/* These types must be 8-bit integer */
typedef char			CHAR;
typedef unsigned char	UCHAR;
typedef unsigned char	BYTE;

/* These types must be 16-bit integer */
typedef short			SHORT;
typedef unsigned short	USHORT;
typedef unsigned short	WORD;

/* These types must be 32-bit integer */
typedef long			LONG;
typedef unsigned long	ULONG;
typedef unsigned long	DWORD;


/* Error code */
typedef enum {
	JDR_OK = 0,	/* 0: Succeeded */
	JDR_INTR,	/* 1: Interrupted by output function */
	JDR_INP,	/* 2: Device error or wrong termination of input stream */
	JDR_MEM1,	/* 3: Insufficient memory pool for the image */
	JDR_MEM2,	/* 4: Insufficient stream input buffer */
	JDR_PAR,	/* 5: Parameter error */
	JDR_FMT1,	/* 6: Data format error (may be damaged data) */
	JDR_FMT2,	/* 7: Right format but not supported */
	JDR_FMT3	/* 8: Not supported JPEG standard */
} JRESULT;



/* Rectangular structure */
typedef struct {
	WORD left, right, top, bottom;
} JRECT;



/* Decompressor object structure */
typedef struct JDEC JDEC;
struct JDEC {
	UINT dctr;				/* Number of bytes available in the input buffer */
	BYTE* dptr;				/* Current data read ptr */
	BYTE* inbuf;			/* Bit stream input buffer */
	BYTE dmsk;				/* Current bit in the current read byte */
	BYTE scale;				/* Output scaling ratio */
	BYTE msx, msy;			/* MCU size in unit of block (width, height) */
	BYTE qtid[3];			/* Quantization table ID of each component */
	SHORT dcv[3];			/* Previous DC element of each component */
	WORD nrst;				/* Restart inverval */
	UINT width, height;		/* Size of the input image (pixel) */
	BYTE* huffbits[2][2];	/* Huffman bit distribution tables [id][dcac] */
	WORD* huffcode[2][2];	/* Huffman code word tables [id][dcac] */
	BYTE* huffdata[2][2];	/* Huffman decoded data tables [id][dcac] */
	LONG* qttbl[4];			/* Dequaitizer tables [id] */
	void* workbuf;			/* Working buffer for IDCT and RGB output */
	BYTE* mcubuf;			/* Working buffer for the MCU */
	void* pool;				/* Pointer to available memory pool */
	UINT sz_pool;			/* Size of momory pool (bytes available) */
	UINT(*infunc)(JDEC*, BYTE*, UINT);/* Pointer to jpeg stream input function */
	void* device;			/* Pointer to I/O device identifiler for the session */
    bool swap = false;
};

void IRAM_ATTR yuv2rgb(uint8_t y, uint8_t u, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b);


/* TJpgDec API functions */
JRESULT jd_prepare(JDEC*, UINT(*)(JDEC*, BYTE*, UINT), void*, UINT, void*);
JRESULT jd_decomp(JDEC*, UINT(*)(JDEC*, void*, JRECT*), BYTE);

namespace jpge
{
        typedef unsigned char  uint8;
        typedef signed short   int16;
        typedef signed int     int32;
        typedef unsigned short uint16;
        typedef unsigned int   uint32;
        typedef size_t   uint;

        // JPEG chroma subsampling factors. Y_ONLY (grayscale images) and H2V2 (color images) are the most common.
        enum subsampling_t { Y_ONLY = 0, H1V1 = 1, H2V1 = 2, H2V2 = 3 };

        // JPEG compression parameters structure.
        struct params {
                inline params() : m_quality(85), m_subsampling(H2V2) { }

                inline bool check() const {
                        if ((m_quality < 1) || (m_quality > 100)) {
                                return false;
                        }
                        if ((uint)m_subsampling > (uint)H2V2) {
                                return false;
                        }
                        return true;
                }

                // Quality: 1-100, higher is better. Typical values are around 50-95.
                int m_quality;

                // m_subsampling:
                // 0 = Y (grayscale) only
                // 1 = H1V1 subsampling (YCbCr 1x1x1, 3 blocks per MCU)
                // 2 = H2V1 subsampling (YCbCr 2x1x1, 4 blocks per MCU)
                // 3 = H2V2 subsampling (YCbCr 4x1x1, 6 blocks per MCU-- very common)
                subsampling_t m_subsampling;
        };

        // Output stream abstract class - used by the jpeg_encoder class to write to the output stream.
        // put_buf() is generally called with len==JPGE_OUT_BUF_SIZE bytes, but for headers it'll be called with smaller amounts.
        class output_stream {
        public:
                virtual ~output_stream() { };
                virtual bool put_buf(const void* Pbuf, int len) = 0;
                virtual uint get_size() const = 0;
        };

        // Lower level jpeg_encoder class - useful if more control is needed than the above helper functions.
        class jpeg_encoder {
        public:
                jpeg_encoder();
                ~jpeg_encoder();

                // Initializes the compressor.
                // pStream: The stream object to use for writing compressed data.
                // params - Compression parameters structure, defined above.
                // width, height  - Image dimensions.
                // channels - May be 1, or 3. 1 indicates grayscale, 3 indicates RGB source data.
                // Returns false on out of memory or if a stream write fails.
                bool init(output_stream* pStream, int width, int height, int src_channels, const params& comp_params = params());

                // Call this method with each source scanline.
                // width * src_channels bytes per scanline is expected (RGB or Y format).
                // You must call with NULL after all scanlines are processed to finish compression.
                // Returns false on out of memory or if a stream write fails.
                bool process_scanline(const void* pScanline);

                // Deinitializes the compressor, freeing any allocated memory. May be called at any time.
                void deinit();

        private:
                jpeg_encoder(const jpeg_encoder&);
                jpeg_encoder& operator =(const jpeg_encoder&);

                typedef int32 sample_array_t;
                enum { JPGE_OUT_BUF_SIZE = 512 };

                output_stream* m_pStream;
                params m_params;
                uint8 m_num_components;
                uint8 m_comp_h_samp[3], m_comp_v_samp[3];
                int m_image_x, m_image_y, m_image_bpp, m_image_bpl;
                int m_image_x_mcu, m_image_y_mcu;
                int m_image_bpl_xlt, m_image_bpl_mcu;
                int m_mcus_per_row;
                int m_mcu_x, m_mcu_y;
                uint8* m_mcu_lines[16];
                uint8 m_mcu_y_ofs;
                sample_array_t m_sample_array[64];
                int16 m_coefficient_array[64];

                int m_last_dc_val[3];
                uint8 m_out_buf[JPGE_OUT_BUF_SIZE];
                uint8* m_pOut_buf;
                uint m_out_buf_left;
                uint32 m_bit_buffer;
                uint m_bits_in;
                uint8 m_pass_num;
                bool m_all_stream_writes_succeeded;

                bool jpg_open(int p_x_res, int p_y_res, int src_channels);

                void flush_output_buffer();
                void put_bits(uint bits, uint len);

                void emit_byte(uint8 i);
                void emit_word(uint i);
                void emit_marker(int marker);

                void emit_jfif_app0();
                void emit_dqt();
                void emit_sof();
                void emit_dht(uint8* bits, uint8* val, int index, bool ac_flag);
                void emit_dhts();
                void emit_sos();

                void compute_quant_table(int32* dst, const int16* src);
                void load_quantized_coefficients(int component_num);

                void load_block_8_8_grey(int x);
                void load_block_8_8(int x, int y, int c);
                void load_block_16_8(int x, int c);
                void load_block_16_8_8(int x, int c);

                void code_coefficients_pass_two(int component_num);
                void code_block(int component_num);

                void process_mcu_row();
                bool process_end_of_image();
                void load_mcu(const void* src);
                void clear();
                void init();
        };
}

bool fmt2rgb888(const uint8_t *src_buf, size_t src_len, pixformat_t format, uint8_t * rgb_buf);
bool fmt2jpg(uint8_t *src, size_t src_len, uint16_t width, uint16_t height, pixformat_t format, uint8_t quality, uint8_t ** out, size_t * out_len);
bool fmt2bmp(uint8_t *src, size_t src_len, uint16_t width, uint16_t height, pixformat_t format, uint8_t ** out, size_t * out_len);
