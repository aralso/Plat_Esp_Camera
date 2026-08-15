#include "lpc.h"

struct rgb_t
{
	uint8_t r;
	uint8_t g;
	uint8_t b;

#if LPC_USE_YCBCR
	uint8_t get_Y()
	{
		return clamp8((77*(int)r + 150*(int)g + 29*(int)b) >> 8);
	}

	uint8_t get_Cb()
	{
		return clamp8(((-43*(int)r - 85*(int)g + 128*(int)b) >> 8) + 128);
	}

	uint8_t get_Cr()
	{
		return clamp8(((128*(int)r - 107*(int)g - 21*(int)b) >> 8) + 128);
	}
#else
	uint8_t get_Y() { return r; }
	uint8_t get_Cb() { return g; }
	uint8_t get_Cr() { return b; }
#endif

	void from_YCbCr(uint8_t Y, uint8_t Cb, uint8_t Cr)
	{
#if LPC_USE_YCBCR
		int y = Y;
		int cb = (int)Cb - 128;
		int cr = (int)Cr - 128;

		r = clamp8(y + ((359 * cr) >> 8));
		g = clamp8(y - (( 88 * cb + 183 * cr) >> 8));
		b = clamp8(y + ((454 * cb) >> 8));
#else
		r = Y;
		g = Cb;
		b = Cr;
#endif
	}
};

void macroblock_t::from_rgb(const uint8_t *rgb, int width, int height, int x, int y)
{
	rgb_t *pixels = (rgb_t*)rgb;
	int base_x = x * MB_SIZE;
	int base_y = y * MB_SIZE;

	for (int i = 0; i < CHROMA_BLOCK_SIZE; i++)
	{
		for (int j = 0; j < CHROMA_BLOCK_SIZE; j++)
		{
			int pos_x = base_x + i * 2;
			int pos_y = base_y + j * 2;

			int x0 = min(pos_x+0, width-1);
			int y0 = min(pos_y+0, height-1);

			int x1 = min(pos_x+1, width-1);
			int y1 = min(pos_y+1, height-1);

			rgb_t &p00 = pixels[x0 + y0 * width];
			rgb_t &p10 = pixels[x1 + y0 * width];
			rgb_t &p01 = pixels[x0 + y1 * width];
			rgb_t &p11 = pixels[x1 + y1 * width];

			// Luma
			int block_i = (i * 2) / LUMA_BLOCK_SIZE;
			int block_j = (j * 2) / LUMA_BLOCK_SIZE;
			auto &luma_block = luma[block_i * LUMA_BLOCK_COUNT + block_j];
			int luma_i = (i * 2) - (block_i * LUMA_BLOCK_SIZE);
			int luma_j = (j * 2) - (block_j * LUMA_BLOCK_SIZE);
			luma_block.Y[(luma_i+0) * LUMA_BLOCK_SIZE + (luma_j+0)] = p00.get_Y();
			luma_block.Y[(luma_i+1) * LUMA_BLOCK_SIZE + (luma_j+0)] = p10.get_Y();
			luma_block.Y[(luma_i+0) * LUMA_BLOCK_SIZE + (luma_j+1)] = p01.get_Y();
			luma_block.Y[(luma_i+1) * LUMA_BLOCK_SIZE + (luma_j+1)] = p11.get_Y();

			// Cb Cr
			rgb_t avg;
			avg.r = ((int)p00.r + (int)p10.r + (int)p01.r + (int)p11.r) >> 2;
			avg.g = ((int)p00.g + (int)p10.g + (int)p01.g + (int)p11.g) >> 2;
			avg.b = ((int)p00.b + (int)p10.b + (int)p01.b + (int)p11.b) >> 2;

			chroma_u.C[i * CHROMA_BLOCK_SIZE + j] = avg.get_Cb();
			chroma_v.C[i * CHROMA_BLOCK_SIZE + j] = avg.get_Cr();
		}
	}
}

void macroblock_t::to_rgb(uint8_t *rgb, int width, int height, int x, int y) const
{
	rgb_t *pixels = (rgb_t*)rgb;
	int base_x = x * MB_SIZE;
	int base_y = y * MB_SIZE;

	for (int block_i = 0; block_i < LUMA_BLOCK_COUNT; block_i++)
	{
		for (int block_j = 0; block_j < LUMA_BLOCK_COUNT; block_j++)
		{
			auto &block = luma[block_i * LUMA_BLOCK_COUNT + block_j];

			for (int i = 0; i < LUMA_BLOCK_SIZE; i++)
			{
				for (int j = 0; j < LUMA_BLOCK_SIZE; j++)
				{
					int pos_block_x = block_i * LUMA_BLOCK_SIZE + i;
					int pos_block_y = block_j * LUMA_BLOCK_SIZE + j;

					int pos_x = base_x + pos_block_x;
					int pos_y = base_y + pos_block_y;

					if (pos_x >= width || pos_y >= height)
						continue;

					uint8_t Y = block.Y[i * LUMA_BLOCK_SIZE + j];
					uint8_t Cb = chroma_u.C[(pos_block_x/2) * CHROMA_BLOCK_SIZE + (pos_block_y/2)];
					uint8_t Cr = chroma_v.C[(pos_block_x/2) * CHROMA_BLOCK_SIZE + (pos_block_y/2)];

					pixels[pos_x + pos_y * width].from_YCbCr(Y, Cb, Cr);
				}
			}
		}
	}
}




/// JPEG decode

#define	JD_SZBUF		512	/* Size of stream input buffer */
#define JD_FORMAT		0	/* Output pixel format 0:RGB888 (3 BYTE/pix), 1:RGB565 (1 WORD/pix) */
#define	JD_USE_SCALE	0	/* Use descaling feature for output */
#define JD_TBLCLIP		1	/* Use table for saturation (might be a bit faster but increases 1K bytes of code size) */

struct rgb_jpg_decoder
{
	lpc_stream_in_t *input;
	uint8_t *output;
};

struct JDEC
{
	uint32_t in_width, in_height;	/* Size of the input image (pixel) */
	uint32_t out_width, out_height;	/* Size of the output image (pixel) */
	uint32_t num_mb_x, num_mb_y;	/* Size of the output image (macroblocks) */

	uint32_t dctr;				/* Number of bytes available in the input buffer */
	uint8_t* dptr;				/* Current data read ptr */
	uint8_t* inbuf;			/* Bit stream input buffer */
	uint8_t dmsk;				/* Current bit in the current read byte */
	uint8_t msx, msy;			/* MCU size in unit of block (width, height) */
	uint8_t qtid[3];			/* Quantization table ID of each component */
	int16_t dcv[3];			/* Previous DC element of each component */
	uint16_t nrst;				/* Restart inverval */
	uint8_t* huffbits[2][2];	/* Huffman bit distribution tables [id][dcac] */
	uint16_t* huffcode[2][2];	/* Huffman code word tables [id][dcac] */
	uint8_t* huffdata[2][2];	/* Huffman decoded data tables [id][dcac] */
	long* qttbl[4];			/* Dequaitizer tables [id] */
	void* workbuf;			/* Working buffer for IDCT and RGB output */
	uint8_t* mcubuf;			/* Working buffer for the MCU */
	void* pool;				/* Pointer to available memory pool */
	uint32_t sz_pool;			/* Size of momory pool (bytes available) */
	uint32_t(*infunc)(JDEC*, uint8_t*, uint32_t);/* Pointer to jpeg stream input function */
	rgb_jpg_decoder device;			/* Pointer to I/O device identifiler for the session */
	bool swap = false;
};

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
	uint16_t left, right, top, bottom;
} JRECT;

static uint32_t _jpg_read(JDEC *decoder, uint8_t *buf, uint32_t len)
{
	return (uint32_t)decoder->device.input->read_bytes(buf, len);
}

static bool _jpg_write(JDEC *decoder, rgb_t *input, const JRECT &rect)
{
	uint16_t w = rect.right + 1 - rect.left;
	macroblock_t *macroblocks = (macroblock_t*)decoder->device.output;

	unsigned first_x = (rect.left * decoder->out_width) / decoder->in_width;
	unsigned last_x = (rect.right * decoder->out_width) / decoder->in_width;
	unsigned first_y = (rect.top * decoder->out_height) / decoder->in_height;
	unsigned last_y = (rect.bottom * decoder->out_height) / decoder->in_height;

	for (unsigned mb_x = first_x / MB_SIZE; mb_x <= last_x / MB_SIZE; mb_x++)
	{
		for (unsigned mb_y = first_y / MB_SIZE; mb_y <= last_y / MB_SIZE; mb_y++)
		{
			auto &mb = macroblocks[mb_x * decoder->num_mb_y + mb_y];

			for (int i = 0; i < CHROMA_BLOCK_SIZE; i++)
			{
				for (int j = 0; j < CHROMA_BLOCK_SIZE; j++)
				{
					unsigned pos_x = mb_x * MB_SIZE + i * 2;
					unsigned pos_y = mb_y * MB_SIZE + j * 2;

					// If we are inside the image, don't write outside of mb bounds
					if (pos_x < decoder->out_width && pos_y < decoder->out_height)
					{
						if (pos_x < first_x || pos_x > last_x) continue;
						if (pos_y < first_y || pos_y > last_y) continue;
					}

					pos_x = pos_x * decoder->in_width / decoder->out_width;
					pos_y = pos_y * decoder->in_height / decoder->out_height;

					int x0 = max(0, (int)(min(pos_x + 0, decoder->in_width - 1) - rect.left));
					int y0 = max(0, (int)(min(pos_y + 0, decoder->in_height - 1) - rect.top));

					int x1 = max(0, (int)(min(pos_x + 1, decoder->in_width - 1) - rect.left));
					int y1 = max(0, (int)(min(pos_y + 1, decoder->in_height - 1) - rect.top));

					rgb_t &p00 = input[x0 + y0 * w];
					rgb_t &p10 = input[x1 + y0 * w];
					rgb_t &p01 = input[x0 + y1 * w];
					rgb_t &p11 = input[x1 + y1 * w];

					// Luma
					int block_i = (i * 2) / LUMA_BLOCK_SIZE;
					int block_j = (j * 2) / LUMA_BLOCK_SIZE;
					auto &luma_block = mb.luma[block_i * LUMA_BLOCK_COUNT + block_j];
					int luma_i = (i * 2) - (block_i * LUMA_BLOCK_SIZE);
					int luma_j = (j * 2) - (block_j * LUMA_BLOCK_SIZE);
					luma_block.Y[(luma_i + 0) * LUMA_BLOCK_SIZE + (luma_j + 0)] = p00.r;
					luma_block.Y[(luma_i + 1) * LUMA_BLOCK_SIZE + (luma_j + 0)] = p10.r;
					luma_block.Y[(luma_i + 0) * LUMA_BLOCK_SIZE + (luma_j + 1)] = p01.r;
					luma_block.Y[(luma_i + 1) * LUMA_BLOCK_SIZE + (luma_j + 1)] = p11.r;

					// Cb Cr
					rgb_t avg;
					avg.r = ((int)p00.r + (int)p10.r + (int)p01.r + (int)p11.r) >> 2;
					avg.g = ((int)p00.g + (int)p10.g + (int)p01.g + (int)p11.g) >> 2;
					avg.b = ((int)p00.b + (int)p10.b + (int)p01.b + (int)p11.b) >> 2;

					mb.chroma_u.C[i * CHROMA_BLOCK_SIZE + j] = avg.g;
					mb.chroma_v.C[i * CHROMA_BLOCK_SIZE + j] = avg.b;
				}
			}
		}
	}

	return true;
}

JRESULT jd_prepare(JDEC*, void*, unsigned int);
JRESULT jd_decomp(JDEC*);

void decode_jpeg(lpc_stream_in_t *stream, uint8_t *out, int width, int height)
{
	PROFILER_SCOPE(DECODE_JPEG);

	const int work_buf_size = 3100;
	uint8_t *work = lpc_alloc<uint8_t>(work_buf_size, "JPEG decoder tmp mem");

	JDEC decoder;
	decoder.device.input = stream;
	decoder.device.output = out;
	decoder.out_width = width;
	decoder.out_height = height;
	decoder.num_mb_x = div_round_up(width, MB_SIZE);
	decoder.num_mb_y = div_round_up(height, MB_SIZE);

	JRESULT jres = jd_prepare(&decoder, work, work_buf_size);
	LPC_ASSERT(jres == JDR_OK);

	jres = jd_decomp(&decoder);
	lpc_free(work);
	LPC_ASSERT(jres == JDR_OK);
}


/*----------------------------------------------------------------------------*/
/* TJpgDec - Tiny JPEG Decompressor R0.01c                     (C)ChaN, 2019  */
/*----------------------------------------------------------------------------*/


/*-----------------------------------------------*/
/* Zigzag-order to raster-order conversion table */
/*-----------------------------------------------*/

#define ZIG(n)  Zig[n]

static const uint8_t Zig[64] = {  /* Zigzag-order to raster-order conversion table */
   0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
  12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
  35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
  58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63
};

/*-------------------------------------------------*/
/* Input scale factor of Arai algorithm            */
/* (scaled up 16 bits for fixed point operations)  */
/*-------------------------------------------------*/

#define IPSF(n) Ipsf[n]

static const uint16_t Ipsf[64] = {  /* See also aa_idct.png */
  (uint16_t)(1.00000*8192), (uint16_t)(1.38704*8192), (uint16_t)(1.30656*8192), (uint16_t)(1.17588*8192), (uint16_t)(1.00000*8192), (uint16_t)(0.78570*8192), (uint16_t)(0.54120*8192), (uint16_t)(0.27590*8192),
  (uint16_t)(1.38704*8192), (uint16_t)(1.92388*8192), (uint16_t)(1.81226*8192), (uint16_t)(1.63099*8192), (uint16_t)(1.38704*8192), (uint16_t)(1.08979*8192), (uint16_t)(0.75066*8192), (uint16_t)(0.38268*8192),
  (uint16_t)(1.30656*8192), (uint16_t)(1.81226*8192), (uint16_t)(1.70711*8192), (uint16_t)(1.53636*8192), (uint16_t)(1.30656*8192), (uint16_t)(1.02656*8192), (uint16_t)(0.70711*8192), (uint16_t)(0.36048*8192),
  (uint16_t)(1.17588*8192), (uint16_t)(1.63099*8192), (uint16_t)(1.53636*8192), (uint16_t)(1.38268*8192), (uint16_t)(1.17588*8192), (uint16_t)(0.92388*8192), (uint16_t)(0.63638*8192), (uint16_t)(0.32442*8192),
  (uint16_t)(1.00000*8192), (uint16_t)(1.38704*8192), (uint16_t)(1.30656*8192), (uint16_t)(1.17588*8192), (uint16_t)(1.00000*8192), (uint16_t)(0.78570*8192), (uint16_t)(0.54120*8192), (uint16_t)(0.27590*8192),
  (uint16_t)(0.78570*8192), (uint16_t)(1.08979*8192), (uint16_t)(1.02656*8192), (uint16_t)(0.92388*8192), (uint16_t)(0.78570*8192), (uint16_t)(0.61732*8192), (uint16_t)(0.42522*8192), (uint16_t)(0.21677*8192),
  (uint16_t)(0.54120*8192), (uint16_t)(0.75066*8192), (uint16_t)(0.70711*8192), (uint16_t)(0.63638*8192), (uint16_t)(0.54120*8192), (uint16_t)(0.42522*8192), (uint16_t)(0.29290*8192), (uint16_t)(0.14932*8192),
  (uint16_t)(0.27590*8192), (uint16_t)(0.38268*8192), (uint16_t)(0.36048*8192), (uint16_t)(0.32442*8192), (uint16_t)(0.27590*8192), (uint16_t)(0.21678*8192), (uint16_t)(0.14932*8192), (uint16_t)(0.07612*8192)
};

/*-----------------------------------------------------------------------*/
/* Allocate a memory block from memory pool                              */
/*-----------------------------------------------------------------------*/

static void* alloc_pool ( /* Pointer to allocated memory block (NULL:no memory available) */
  JDEC* jd,   /* Pointer to the decompressor object */
	unsigned int nd   /* Number of bytes to allocate */
)
{
	char *rp = 0;


	nd = (nd + 3) & ~3;     /* Align block size to the word boundary */

	if (jd->sz_pool >= nd) {
		jd->sz_pool -= nd;
		rp = (char *)jd->pool;     /* Get start of available memory pool */
		jd->pool = (void *)(rp + nd);  /* Allocate requierd bytes */
	}

	return (void *)rp; /* Return allocated memory block (NULL:no memory to allocate) */
}

/*-----------------------------------------------------------------------*/
/* Create de-quantization and prescaling tables with a DQT segment       */
/*-----------------------------------------------------------------------*/

static JRESULT create_qt_tbl (  /* 0:OK, !0:Failed */
  JDEC* jd,       /* Pointer to the decompressor object */
  const uint8_t* data,  /* Pointer to the quantizer tables */
  unsigned int ndata      /* Size of input data */
)
{
	unsigned int i;
	uint8_t d, z;
	int32_t *pb;


	while (ndata) { /* Process all tables in the segment */
		if (ndata < 65)
			return JDR_FMT1;  /* Err: table size is unaligned */
		ndata -= 65;
		d = *data++;              /* Get table property */
		if (d & 0xF0)
			return JDR_FMT1;      /* Err: not 8-bit resolution */
		i = d & 3;                /* Get table ID */
		pb = (int32_t *)alloc_pool(jd, 64 * sizeof(int32_t));/* Allocate a memory block for the table */
		if (!pb) return JDR_MEM1;       /* Err: not enough memory */
		jd->qttbl[i] = (long *)pb;            /* Register the table */
		for (i = 0; i < 64; i++) {        /* Load the table */
			z = ZIG(i);             /* Zigzag-order to raster-order conversion */
			pb[z] = (int32_t)((uint32_t)*data++ * IPSF(z)); /* Apply scale factor of Arai algorithm to the de-quantizers */
		}
	}

	return JDR_OK;
}




/*-----------------------------------------------------------------------*/
/* Create huffman code tables with a DHT segment                         */
/*-----------------------------------------------------------------------*/

static JRESULT create_huffman_tbl ( /* 0:OK, !0:Failed */
  JDEC* jd,         /* Pointer to the decompressor object */
  const uint8_t* data,    /* Pointer to the packed huffman tables */
  unsigned int ndata        /* Size of input data */
)
{
	unsigned int i, j, b, np, cls, num;
	uint8_t d, *pb, *pd;
	uint16_t hc, *ph;


	while (ndata) { /* Process all tables in the segment */
		if (ndata < 17)
			return JDR_FMT1;  /* Err: wrong data size */
		ndata -= 17;
		d = *data++;            /* Get table number and class */
		if (d & 0xEE)
			return JDR_FMT1;    /* Err: invalid class/number */
		cls = d >> 4; num = d & 0x0F;   /* class = dc(0)/ac(1), table number = 0/1 */
		pb = (uint8_t *)alloc_pool(jd, 16);      /* Allocate a memory block for the bit distribution table */
		if (!pb) return JDR_MEM1;     /* Err: not enough memory */
		jd->huffbits[num][cls] = pb;
		for (np = i = 0; i < 16; i++) {   /* Load number of patterns for 1 to 16-bit code */
			np += (pb[i] = *data++);    /* Get sum of code words for each code */
		}
		ph = (uint16_t *)alloc_pool(jd, (unsigned int)(np * sizeof(uint16_t)));/* Allocate a memory block for the code word table */
		if (!ph) return JDR_MEM1;     /* Err: not enough memory */
		jd->huffcode[num][cls] = ph;
		hc = 0;
		for (j = i = 0; i < 16; i++) {    /* Re-build huffman code word table */
			b = pb[i];
			while (b--) ph[j++] = hc++;
			hc <<= 1;
		}

		if (ndata < np)
			return JDR_FMT1;  /* Err: wrong data size */
		ndata -= np;
		pd = (uint8_t *)alloc_pool(jd, np);      /* Allocate a memory block for the decoded data */
		if (!pd) return JDR_MEM1;     /* Err: not enough memory */
		jd->huffdata[num][cls] = pd;
		for (i = 0; i < np; i++) {      /* Load decoded data corresponds to each code ward */
			d = *data++;
			if (!cls && d > 11)
				return JDR_FMT1;
			*pd++ = d;
		}
	}

	return JDR_OK;
}




/*-----------------------------------------------------------------------*/
/* Extract N bits from input stream                                      */
/*-----------------------------------------------------------------------*/

static int bitext( /* >=0: extracted data, <0: error code */
	JDEC *jd,   /* Pointer to the decompressor object */
	unsigned int nbit    /* Number of bits to extract (1 to 11) */
)
{
	uint8_t msk, s, *dp;
	unsigned int dc, v, f;


	msk = jd->dmsk; dc = jd->dctr; dp = jd->dptr; /* Bit mask, number of data available, read ptr */
	s = *dp; v = f = 0;

	do {
		if (!msk) {       /* Next byte? */
			if (!dc) {      /* No input data is available, re-fill input buffer */
				dp = jd->inbuf; /* Top of input buffer */
				dc = jd->infunc(jd, dp, JD_SZBUF);
				if (!dc) return 0 - (int)JDR_INP; /* Err: read error or wrong stream termination */
			}
			else {
				dp++;     /* Next data ptr */
			}
			dc--;       /* Decrement number of available bytes */
			if (f) {      /* In flag sequence? */
				f = 0;      /* Exit flag sequence */
				if (*dp != 0)
					return 0 - (int)JDR_FMT1; /* Err: unexpected flag is detected (may be collapted data) */
				*dp = s = 0xFF;     /* The flag is a data 0xFF */
			}
			else {
				s = *dp;        /* Get next data byte */
				if (s == 0xFF) {    /* Is start of flag sequence? */
					f = 1; continue;  /* Enter flag sequence */
				}
			}
			msk = 0x80;   /* Read from MSB */
		}
		v <<= 1;  /* Get a bit */
		if (s & msk) v++;
		msk >>= 1;
		nbit--;
	} while (nbit);

	jd->dmsk = msk; jd->dctr = dc; jd->dptr = dp;

	return (int)v;
}




/*-----------------------------------------------------------------------*/
/* Extract a huffman decoded data from input stream                      */
/*-----------------------------------------------------------------------*/

static int huffext (  /* >=0: decoded data, <0: error code */
  JDEC* jd,       /* Pointer to the decompressor object */
  const uint8_t* hbits, /* Pointer to the bit distribution table */
  const uint16_t* hcode,  /* Pointer to the code word table */
  const uint8_t* hdata  /* Pointer to the data table */
)
{
	uint8_t msk, s, *dp;
	unsigned int dc, v, f, bl, nd;


	msk = jd->dmsk; dc = jd->dctr; dp = jd->dptr; /* Bit mask, number of data available, read ptr */
	s = *dp; v = f = 0;
	bl = 16;  /* Max code length */
	do {
		if (!msk) {   /* Next byte? */
			if (!dc) {  /* No input data is available, re-fill input buffer */
				dp = jd->inbuf; /* Top of input buffer */
				dc = jd->infunc(jd, dp, JD_SZBUF);
				if (!dc) return 0 - (int)JDR_INP; /* Err: read error or wrong stream termination */
			}
			else {
				dp++; /* Next data ptr */
			}
			dc--;   /* Decrement number of available bytes */
			if (f) {    /* In flag sequence? */
				f = 0;    /* Exit flag sequence */
				if (*dp != 0)
					return 0 - (int)JDR_FMT1; /* Err: unexpected flag is detected (may be collapted data) */
				*dp = s = 0xFF;     /* The flag is a data 0xFF */
			}
			else {
				s = *dp;        /* Get next data byte */
				if (s == 0xFF) {    /* Is start of flag sequence? */
					f = 1; continue;  /* Enter flag sequence, get trailing byte */
				}
			}
			msk = 0x80;   /* Read from MSB */
		}
		v <<= 1;  /* Get a bit */
		if (s & msk) v++;
		msk >>= 1;

		for (nd = *hbits++; nd; nd--) { /* Search the code word in this bit length */
			if (v == *hcode++) {    /* Matched? */
				jd->dmsk = msk; jd->dctr = dc; jd->dptr = dp;
				return *hdata;      /* Return the decoded data */
			}
			hdata++;
		}
		bl--;
	} while (bl);

	return 0 - (int)JDR_FMT1; /* Err: code not found (may be collapted data) */
}




/*-----------------------------------------------------------------------*/
/* Apply Inverse-DCT in Arai Algorithm (see also aa_idct.png)            */
/*-----------------------------------------------------------------------*/

static void block_idct (
  int32_t* src, /* Input block data (de-quantized and pre-scaled for Arai Algorithm) */
  uint8_t* dst  /* Pointer to the destination to store the block as byte array */
)
{
	const int32_t M13 = (int32_t)(1.41421 * 4096), M2 = (int32_t)(1.08239 * 4096), M4 = (int32_t)(2.61313 * 4096), M5 = (int32_t)(1.84776 * 4096);
	int32_t v0, v1, v2, v3, v4, v5, v6, v7;
	int32_t t10, t11, t12, t13;
	int i;

	/* Process columns */
	for (i = 0; i < 8; i++) {
		v0 = src[8 * 0];  /* Get even elements */
		v1 = src[8 * 2];
		v2 = src[8 * 4];
		v3 = src[8 * 6];

		t10 = v0 + v2;    /* Process the even elements */
		t12 = v0 - v2;
		t11 = (v1 - v3) * M13 >> 12;
		v3 += v1;
		t11 -= v3;
		v0 = t10 + v3;
		v3 = t10 - v3;
		v1 = t11 + t12;
		v2 = t12 - t11;

		v4 = src[8 * 7];  /* Get odd elements */
		v5 = src[8 * 1];
		v6 = src[8 * 5];
		v7 = src[8 * 3];

		t10 = v5 - v4;    /* Process the odd elements */
		t11 = v5 + v4;
		t12 = v6 - v7;
		v7 += v6;
		v5 = (t11 - v7) * M13 >> 12;
		v7 += t11;
		t13 = (t10 + t12) * M5 >> 12;
		v4 = t13 - (t10 * M2 >> 12);
		v6 = t13 - (t12 * M4 >> 12) - v7;
		v5 -= v6;
		v4 -= v5;

		src[8 * 0] = v0 + v7; /* Write-back transformed values */
		src[8 * 7] = v0 - v7;
		src[8 * 1] = v1 + v6;
		src[8 * 6] = v1 - v6;
		src[8 * 2] = v2 + v5;
		src[8 * 5] = v2 - v5;
		src[8 * 3] = v3 + v4;
		src[8 * 4] = v3 - v4;

		src++;  /* Next column */
	}

	/* Process rows */
	src -= 8;
	for (i = 0; i < 8; i++) {
		v0 = src[0] + (128L << 8);  /* Get even elements (remove DC offset (-128) here) */
		v1 = src[2];
		v2 = src[4];
		v3 = src[6];

		t10 = v0 + v2;        /* Process the even elements */
		t12 = v0 - v2;
		t11 = (v1 - v3) * M13 >> 12;
		v3 += v1;
		t11 -= v3;
		v0 = t10 + v3;
		v3 = t10 - v3;
		v1 = t11 + t12;
		v2 = t12 - t11;

		v4 = src[7];        /* Get odd elements */
		v5 = src[1];
		v6 = src[5];
		v7 = src[3];

		t10 = v5 - v4;        /* Process the odd elements */
		t11 = v5 + v4;
		t12 = v6 - v7;
		v7 += v6;
		v5 = (t11 - v7) * M13 >> 12;
		v7 += t11;
		t13 = (t10 + t12) * M5 >> 12;
		v4 = t13 - (t10 * M2 >> 12);
		v6 = t13 - (t12 * M4 >> 12) - v7;
		v5 -= v6;
		v4 -= v5;

		dst[0] = clamp8((v0 + v7) >> 8);  /* Descale the transformed values 8 bits and output */
		dst[7] = clamp8((v0 - v7) >> 8);
		dst[1] = clamp8((v1 + v6) >> 8);
		dst[6] = clamp8((v1 - v6) >> 8);
		dst[2] = clamp8((v2 + v5) >> 8);
		dst[5] = clamp8((v2 - v5) >> 8);
		dst[3] = clamp8((v3 + v4) >> 8);
		dst[4] = clamp8((v3 - v4) >> 8);
		dst += 8;

		src += 8; /* Next row */
	}
}




/*-----------------------------------------------------------------------*/
/* Load all blocks in the MCU into working buffer                        */
/*-----------------------------------------------------------------------*/

static JRESULT mcu_load(JDEC* jd)
{
	int32_t *tmp = (int32_t *)jd->workbuf; /* Block working buffer for de-quantize and IDCT */
	int b, d, e;
	unsigned int blk, nby, nbc, i, z, id, cmp;
	uint8_t *bp;
	const uint8_t *hb, *hd;
	const uint16_t *hc;
	const int32_t *dqf;


	nby = jd->msx * jd->msy;  /* Number of Y blocks (1, 2 or 4) */
	nbc = 2;          /* Number of C blocks (2) */
	bp = jd->mcubuf;      /* Pointer to the first block */

	for (blk = 0; blk < nby + nbc; blk++) {
		cmp = (blk < nby) ? 0 : blk - nby + 1;  /* Component number 0:Y, 1:Cb, 2:Cr */
		id = cmp ? 1 : 0;           /* Huffman table ID of the component */

		/* Extract a DC element from input stream */
		hb = jd->huffbits[id][0];       /* Huffman table for the DC element */
		hc = jd->huffcode[id][0];
		hd = jd->huffdata[id][0];
		b = huffext(jd, hb, hc, hd);      /* Extract a huffman coded data (bit length) */
		if (b < 0) return (JRESULT)(0 - b);        /* Err: invalid code or input */
		d = jd->dcv[cmp];           /* DC value of previous block */
		if (b) {                /* If there is any difference from previous block */
			e = bitext(jd, b);          /* Extract data bits */
			if (e < 0) return (JRESULT)(0 - e);      /* Err: input */
			b = 1 << (b - 1);         /* MSB position */
			if (!(e & b)) e -= (b << 1) - 1;  /* Restore sign if needed */
			d += e;               /* Get current value */
			jd->dcv[cmp] = (int16_t)d;      /* Save current DC value for next block */
		}
		dqf = (const int32_t *)jd->qttbl[jd->qtid[cmp]];     /* De-quantizer table ID for this component */
		tmp[0] = d * dqf[0] >> 8;       /* De-quantize, apply scale factor of Arai algorithm and descale 8 bits */

		/* Extract following 63 AC elements from input stream */
		for (i = 1; i < 64; tmp[i++] = 0);   /* Clear rest of elements */
		hb = jd->huffbits[id][1];       /* Huffman table for the AC elements */
		hc = jd->huffcode[id][1];
		hd = jd->huffdata[id][1];
		i = 1;          /* Top of the AC elements */
		do {
			b = huffext(jd, hb, hc, hd);    /* Extract a huffman coded value (zero runs and bit length) */
			if (b == 0) break;          /* EOB? */
			if (b < 0) return (JRESULT)(0 - b);      /* Err: invalid code or input error */
			z = (unsigned int)b >> 4;       /* Number of leading zero elements */
			if (z) {
				i += z;             /* Skip zero elements */
				if (i >= 64)
					return JDR_FMT1; /* Too long zero run */
			}
			if (b &= 0x0F) {          /* Bit length */
				d = bitext(jd, b);        /* Extract data bits */
				if (d < 0) return (JRESULT)(0 - d);    /* Err: input device */
				b = 1 << (b - 1);       /* MSB position */
				if (!(d & b)) d -= (b << 1) - 1;/* Restore negative value if needed */
				z = ZIG(i);           /* Zigzag-order to raster-order converted index */
				tmp[z] = d * dqf[z] >> 8;   /* De-quantize, apply scale factor of Arai algorithm and descale 8 bits */
			}
		} while (++i < 64);   /* Next AC element */

		block_idct(tmp, bp);    /* Apply IDCT and store the block to the MCU buffer */

		bp += 64;       /* Next block */
	}

	return JDR_OK;  /* All blocks have been loaded successfully */
}




/*-----------------------------------------------------------------------*/
/* Output an MCU                                                         */
/*-----------------------------------------------------------------------*/

static JRESULT mcu_output (
  JDEC* jd,   /* Pointer to the decompressor object */
  unsigned int x,   /* MCU position in the image (left of the MCU) */
  unsigned int y    /* MCU position in the image (top of the MCU) */
)
{
	unsigned int ix, iy, mx, my, rx, ry;
	uint8_t *py, *pc;
	JRECT rect;

	mx = jd->msx * 8; my = jd->msy * 8;         /* MCU size (pixel) */
	rx = (x + mx <= jd->in_width) ? mx : jd->in_width - x;  /* Output rectangular size (it may be clipped at right/bottom end) */
	ry = (y + my <= jd->in_height) ? my : jd->in_height - y;
	rect.left = x; rect.right = x + rx - 1;       /* Rectangular area in the frame buffer */
	rect.top = y; rect.bottom = y + ry - 1;

	/* Build an MCU from discrete components */
	rgb_t *pixel = (rgb_t*)jd->workbuf;
	for (iy = 0; iy < my; iy++)
	{
		pc = jd->mcubuf;
		py = pc + iy * 8;
		if (my == 16) {   /* Double block height? */
			pc += 64 * 4 + (iy >> 1) * 8;
			if (iy >= 8) py += 64;
		}
		else {      /* Single block height */
			pc += mx * 8 + iy * 8;
		}
		for (ix = 0; ix < mx; ix++)
		{
			// Load data in YCbCr
			pixel->g = pc[0];   /* Get Cb/Cr components */
			pixel->b = pc[64];
			if (mx == 16) {         /* Double block width? */
				if (ix == 8) py += 64 - 8;  /* Jump to next block if double block heigt */
				pc += ix & 1;       /* Increase chroma pointer every two pixels */
			}
			else {            /* Single block width */
				pc++;           /* Increase chroma pointer every pixel */
			}
			pixel->r = *py++;     /* Get Y component */

			pixel++;
		}
	}

	/* Squeeze up pixel table if a part of MCU is to be truncated */
	if (rx < mx) {
		uint8_t *s, *d;

		s = d = (uint8_t *)jd->workbuf;
		for (unsigned y2 = 0; y2 < ry; y2++) {
			for (unsigned x2 = 0; x2 < rx; x2++) {  /* Copy effective pixels */
				*d++ = *s++;
				*d++ = *s++;
				*d++ = *s++;
			}
			s += (mx - rx) * 3; /* Skip truncated pixels */
		}
	}

	/* Output the RGB rectangular */
	return _jpg_write(jd, (rgb_t*)jd->workbuf, rect) ? JDR_OK : JDR_INTR;
}

/*-----------------------------------------------------------------------*/
/* Process restart interval                                              */
/*-----------------------------------------------------------------------*/

static JRESULT restart (
  JDEC* jd,   /* Pointer to the decompressor object */
  uint16_t rstn /* Expected restert sequense number */
)
{
	unsigned int i, dc;
	uint16_t d;
	uint8_t *dp;


	/* Discard padding bits and get two bytes from the input stream */
	dp = jd->dptr; dc = jd->dctr;
	d = 0;
	for (i = 0; i < 2; i++) {
		if (!dc) {  /* No input data is available, re-fill input buffer */
			dp = jd->inbuf;
			dc = jd->infunc(jd, dp, JD_SZBUF);
			if (!dc) return JDR_INP;
		}
		else {
			dp++;
		}
		dc--;
		d = (d << 8) | *dp; /* Get a byte */
	}
	jd->dptr = dp; jd->dctr = dc; jd->dmsk = 0;

	/* Check the marker */
	if ((d & 0xFFD8) != 0xFFD0 || (d & 7) != (rstn & 7)) {
		return JDR_FMT1;  /* Err: expected RSTn marker is not detected (may be collapted data) */
	}

	/* Reset DC offset */
	jd->dcv[2] = jd->dcv[1] = jd->dcv[0] = 0;

	return JDR_OK;
}

/*-----------------------------------------------------------------------*/
/* Analyze the JPEG image and Initialize decompressor object             */
/*-----------------------------------------------------------------------*/

#define LDB_WORD(ptr)   (uint16_t)(((uint16_t)*((uint8_t*)(ptr))<<8)|(uint16_t)*(uint8_t*)((ptr)+1))

JRESULT jd_prepare (
  JDEC* jd,     /* Blank decompressor object */
  void* pool,     /* Working buffer for the decompression session */
  unsigned int sz_pool /* Size of working buffer */
)
{
	uint8_t *seg, b;
	uint16_t marker;
	uint32_t ofs;
	unsigned int n, i, j, len;
	JRESULT rc;


	if (!pool) return JDR_PAR;

	jd->pool = pool;    /* Work memroy */
	jd->sz_pool = sz_pool;  /* Size of given work memory */
	jd->infunc = _jpg_read;  /* Stream input function */
	jd->nrst = 0;     /* No restart interval (default) */

	for (i = 0; i < 2; i++) { /* Nulls pointers */
		for (j = 0; j < 2; j++) {

			jd->huffcode[i][j] = 0;
			jd->huffdata[i][j] = 0;
		}
	}
	for (i = 0; i < 4; jd->qttbl[i++] = 0);

	jd->inbuf = seg = (uint8_t *)alloc_pool(jd, JD_SZBUF);   /* Allocate stream input buffer */
	if (!seg) return JDR_MEM1;

	if (jd->infunc(jd, seg, 2) != 2) return JDR_INP;/* Check SOI marker */
	if (LDB_WORD(seg) != 0xFFD8)
		return JDR_FMT1; /* Err: SOI is not detected */
	ofs = 2;

	for (;;) {
		/* Get a JPEG marker */
		if (jd->infunc(jd, seg, 4) != 4) return JDR_INP;
		marker = LDB_WORD(seg);   /* Marker */
		len = LDB_WORD(seg + 2);  /* Length field */

#if JPGE_VERBOSE
		printf("Marker: %x \tlen: %d \toffset:%d(0x%x)\n", marker, len, ofs, ofs);
#endif

		if (len <= 2 || (marker >> 8) != 0xFF)
			return JDR_FMT1;
		len -= 2;   /* Content size excluding length field */
		ofs += 4 + len; /* Number of bytes loaded */

		switch (marker & 0xFF) {
		case 0xC0:  /* SOF0 (baseline JPEG) */
			/* Load segment data */
			if (len > JD_SZBUF) return JDR_MEM2;
			if (jd->infunc(jd, seg, len) != len) return JDR_INP;

			jd->in_width = LDB_WORD(seg + 3);    /* Image width in unit of pixel */
			jd->in_height = LDB_WORD(seg + 1);   /* Image height in unit of pixel */
			if (seg[5] != 3) return JDR_FMT3; /* Err: Supports only Y/Cb/Cr format */

			/* Check three image components */
			for (i = 0; i < 3; i++) {
				b = seg[7 + 3 * i];             /* Get sampling factor */
				if (!i) { /* Y component */
					if (b != 0x11 && b != 0x22 && b != 0x21) {  /* Check sampling factor */
						return JDR_FMT3;          /* Err: Supports only 4:4:4, 4:2:0 or 4:2:2 */
					}
					jd->msx = b >> 4; jd->msy = b & 15;   /* Size of MCU [blocks] */
				}
				else {  /* Cb/Cr component */
					if (b != 0x11) return JDR_FMT3;     /* Err: Sampling factor of Cr/Cb must be 1 */
				}
				b = seg[8 + 3 * i];             /* Get dequantizer table ID for this component */
				if (b > 3) return JDR_FMT3;         /* Err: Invalid ID */
				jd->qtid[i] = b;
			}
			break;

		case 0xDD:  /* DRI */
			/* Load segment data */
			if (len > JD_SZBUF) return JDR_MEM2;
			if (jd->infunc(jd, seg, len) != len) return JDR_INP;

			/* Get restart interval (MCUs) */
			jd->nrst = LDB_WORD(seg);
			break;

		case 0xC4:  /* DHT */
			/* Load segment data */
			if (len > JD_SZBUF) return JDR_MEM2;
			if (jd->infunc(jd, seg, len) != len) return JDR_INP;

			/* Create huffman tables */
			rc = create_huffman_tbl(jd, seg, len);
			if (rc) return rc;
			break;

		case 0xDB:  /* DQT */
			/* Load segment data */
			if (len > JD_SZBUF) return JDR_MEM2;
			if (jd->infunc(jd, seg, len) != len) return JDR_INP;

			/* Create de-quantizer tables */
			rc = create_qt_tbl(jd, seg, len);
			if (rc) return rc;
			break;

		case 0xDA:  /* SOS */
			/* Load segment data */
			if (len > JD_SZBUF) return JDR_MEM2;
			if (jd->infunc(jd, seg, len) != len) return JDR_INP;

			if (!jd->in_width || !jd->in_height)
				return JDR_FMT1; /* Err: Invalid image size */

			if (seg[0] != 3) return JDR_FMT3;       /* Err: Supports only three color components format */

			/* Check if all tables corresponding to each components have been loaded */
			for (i = 0; i < 3; i++) {
				b = seg[2 + 2 * i]; /* Get huffman table ID */
				if (b != 0x00 && b != 0x11) return JDR_FMT3;  /* Err: Different table number for DC/AC element */
				b = i ? 1 : 0;
				if (!jd->huffbits[b][0] || !jd->huffbits[b][1]) { /* Check dc/ac huffman table for this component */
					return JDR_FMT1;          /* Err: Nnot loaded */
				}
				if (!jd->qttbl[jd->qtid[i]]) {      /* Check dequantizer table for this component */
					return JDR_FMT1;          /* Err: Not loaded */
				}
			}

			/* Allocate working buffer for MCU and RGB */
			n = jd->msy * jd->msx;            /* Number of Y blocks in the MCU */
			if (!n)
				return JDR_FMT1;          /* Err: SOF0 has not been loaded */
			len = n * 64 * 2 + 64;            /* Allocate buffer for IDCT and RGB output */
			if (len < 256) len = 256;         /* but at least 256 byte is required for IDCT */
			jd->workbuf = alloc_pool(jd, len);      /* and it may occupy a part of following MCU working buffer for RGB output */
			if (!jd->workbuf) return JDR_MEM1;      /* Err: not enough memory */
			jd->mcubuf = (uint8_t *)alloc_pool(jd, (unsigned int)((n + 2) * 64));  /* Allocate MCU working buffer */
			if (!jd->mcubuf) return JDR_MEM1;     /* Err: not enough memory */

			/* Pre-load the JPEG data to extract it from the bit stream */
			jd->dptr = seg; jd->dctr = 0; jd->dmsk = 0; /* Prepare to read bit stream */
			if (ofs %= JD_SZBUF) {            /* Align read offset to JD_SZBUF */
				jd->dctr = jd->infunc(jd, seg + ofs, (unsigned int)(JD_SZBUF - ofs));
				jd->dptr = seg + ofs - 1;
			}

			return JDR_OK;    /* Initialization succeeded. Ready to decompress the JPEG image. */

		case 0xC1:  /* SOF1 */
		case 0xC2:  /* SOF2 */
		case 0xC3:  /* SOF3 */
		case 0xC5:  /* SOF5 */
		case 0xC6:  /* SOF6 */
		case 0xC7:  /* SOF7 */
		case 0xC9:  /* SOF9 */
		case 0xCA:  /* SOF10 */
		case 0xCB:  /* SOF11 */
		case 0xCD:  /* SOF13 */
		case 0xCE:  /* SOF14 */
		case 0xCF:  /* SOF15 */
		case 0xD9:  /* EOI */
			return JDR_FMT3;  /* Unsuppoted JPEG standard (may be progressive JPEG) */

		default:  /* Unknown segment (comment, exif or etc..) */
			/* Skip segment data */
			if (jd->infunc(jd, 0, len) != len) {  /* Null pointer specifies to skip bytes of stream */
				return JDR_INP;
			}
		}
	}
}


/*-----------------------------------------------------------------------*/
/* Start to decompress the JPEG picture                                  */
/*-----------------------------------------------------------------------*/

JRESULT jd_decomp(JDEC* jd)
{
	unsigned int x, y, mx, my;
	uint16_t rst, rsc;
	JRESULT rc;

	mx = jd->msx * 8; my = jd->msy * 8;     /* Size of the MCU (pixel) */

	jd->dcv[2] = jd->dcv[1] = jd->dcv[0] = 0; /* Initialize DC values */
	rst = rsc = 0;

	rc = JDR_OK;
	for (y = 0; y < jd->in_height; y += my) {    /* Vertical loop of MCUs */
		for (x = 0; x < jd->in_width; x += mx) { /* Horizontal loop of MCUs */
			if (jd->nrst && rst++ == jd->nrst) {  /* Process restart interval if enabled */
				rc = restart(jd, rsc++);
				if (rc != JDR_OK) return rc;
				rst = 1;
			}
			rc = mcu_load(jd);          /* Load an MCU (decompress huffman coded stream and apply IDCT) */
			if (rc != JDR_OK) return rc;
			rc = mcu_output(jd, x, y); /* Output the MCU (color space conversion, scaling and output) */
			if (rc != JDR_OK) return rc;
		}
	}

	return rc;
}
