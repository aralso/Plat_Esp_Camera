//---------------------------------------------------------------------------
// FILE: 		qmcoder.cpp
//
// Author:
// Wenlong Dong			<wdong@sipi.usc.edu>
// 				http://sipi.usc.edu/~wdong/
//
// July 14, 2000		University of Southern California
// LAST UPDATE:         Jan, 25, 2007
//---------------------------------------------------------------------------

#include <cstdlib>
#include <cstring>
#include <cstdio>

class lpc_cabac_t
{
public:
	lpc_cabac_t(lpc_stream_out_t *stream, int history_length); // Encoder
	lpc_cabac_t(lpc_stream_in_t *stream, int history_length); // Decoder
	~lpc_cabac_t();

	void encode_bytes(const uint8_t *bytes, size_t byte_count);
	void encode_bit(bool symbol);
	void Flush(); //flush output to terminate encoding;

	void decode_bytes(uint8_t *bytes, size_t byte_count);
	bool decode_bit();
	int isEnd() {return bEnd;}; //detecting the end

protected:
	union
	{
		lpc_stream_in_t *stream_in;
		lpc_stream_out_t *stream_out;
	};

private:
	lpc_cabac_t(int history_length);
	int count; // counting of output bitstream

	//private varible
	unsigned long C_register;
	unsigned long A_interval;
	unsigned      ct;
	unsigned long sc;
	bool MPS;
	bool next_MPS;
	unsigned Qe;
	uint8_t BP;
	unsigned Cx;
	int bEnd, bFirst;
	int next_st;
	int cur_st;
	int max_context;
	int context;
	uint8_t *st_table;
	uint8_t *mps_table;

	//private function
	void Code_MPS();
	void Code_LPS();
	void Renorm_e();
	void Byte_out();
	void Output_stacked_zeros();
	void Output_stacked_0xffs();
	void Stuff_0();
	void Clear_final_bits();
	bool AM_decode_Symbol();
	bool Cond_LPS_exchange();
	bool Cond_MPS_exchange();
	void Renorm_d();
	void Byte_in();
	void Unstuff_0();

	inline void QMputc(uint8_t byte)
	{
		if (bFirst)
			stream_out->write_byte(byte);
		else
			bFirst = 1;
	}

public:
	void encode(bool symbol, int prob, bool mps_symbol);
	bool decode(int prob, bool mps_symbol);

	//counting the output bits during encoding;
	int Counting();

	//reset all QM to initial condition;
	void reset();
};

int lsz[256]= {
	0x5a1d, 0x2586, 0x1114, 0x080b, 0x03d8,
	0x01da, 0x0015, 0x006f, 0x0036, 0x001a,
	0x000d, 0x0006, 0x0003, 0x0001, 0x5a7f,
	0x3f25, 0x2cf2, 0x207c, 0x17b9, 0x1182,
	0x0cef, 0x09a1, 0x072f, 0x055c, 0x0406,
	0x0303, 0x0240, 0x01b1, 0x0144, 0x00f5,
	0x00b7, 0x008a, 0x0068, 0x004e, 0x003b,
	0x002c, 0x5ae1, 0x484c, 0x3a0d, 0x2ef1,
	0x261f, 0x1f33, 0x19a8, 0x1518, 0x1177,
	0x0e74, 0x0bfb, 0x09f8, 0x0861, 0x0706,
	0x05cd, 0x04de, 0x040f, 0x0363, 0x02d4,
	0x025c, 0x01f8, 0x01a4, 0x0160, 0x0125,
	0x00f6, 0x00cb, 0x00ab, 0x008f, 0x5b12,
	0x4d04, 0x412c, 0x37d8, 0x2fe8, 0x293c,
	0x2379, 0x1edf, 0x1aa9, 0x174e, 0x1424,
	0x119c, 0x0f6b, 0x0d51, 0x0bb6, 0x0a40,
	0x5832, 0x4d1c, 0x438e, 0x3bdd, 0x34ee,
	0x2eae, 0x299a, 0x2516, 0x5570, 0x4ca9,
	0x44d9, 0x3e22, 0x3824, 0x32b4, 0x2e17,
	0x56a8, 0x4f46, 0x47e5, 0x41cf, 0x3c3d,
	0x375e, 0x5231, 0x4c0f, 0x4639, 0x415e,
	0x5627, 0x50e7, 0x4b85, 0x5597, 0x504f,
	0x5a10, 0x5522, 0x59eb
};

int nlps[256]= {
	1, 14, 16, 18, 20,   23, 25, 28, 30, 33,
	35,  9, 10, 12, 15,   36, 38, 39, 40, 42,
	43, 45, 46, 48, 49,   51, 52, 54, 56, 57,
	59, 60, 62, 63, 32,   33, 37, 64, 65, 67,
	68, 69, 70, 72, 73,   74, 75, 77, 78, 79,
	48, 50, 50, 51, 52,   53, 54, 55, 56, 57,
	58, 59, 61, 61, 65,   80, 81, 82, 83, 84,
	86, 87, 87, 72, 72,   74, 74, 75, 77, 77,
	80, 88, 89, 90, 91,   92, 93, 86, 88, 95,
	96, 97, 99, 99, 93,   95,101,102,103,104,
	99,105,106,107,103,  105,108,109,110,111,
	110,112,112
};

int nmps[256]= {
	1,  2,  3,  4,  5,    6,  7,  8,  9, 10,
	11, 12, 13, 13, 15,   16, 17, 18, 19, 20,
	21, 22, 23, 24, 25,   26, 27, 28, 29, 30,
	31, 32, 33, 34, 35,    9, 37, 38, 39, 40,
	41, 42, 43, 44, 45,   46, 47, 48, 49, 50,
	51, 52, 53, 54, 55,   56, 57, 58, 59, 60,
	61, 62, 63, 32, 65,   66, 67, 68, 69, 70,
	71, 72, 73, 74, 75,   76, 77, 78, 79, 48,
	81, 82, 83, 84, 85,   86, 87, 71, 89, 90,
	91, 92, 93, 94, 86,   96, 97, 98, 99,100,
	93,102,103,104, 99,  106,107,103,109,107,
	111,109,111
};

int swit[256]= {
	1,0,0,0,0,    0,0,0,0,0,
	0,0,0,0,1,    0,0,0,0,0,
	0,0,0,0,0,    0,0,0,0,0,
	0,0,0,0,0,    0,1,0,0,0,
	0,0,0,0,0,    0,0,0,0,0,
	0,0,0,0,0,    0,0,0,0,0,
	0,0,0,0,1,    0,0,0,0,0,
	0,0,0,0,0,    0,0,0,0,0,
	1,0,0,0,0,    0,0,0,1,0,
	0,0,0,0,0,    1,0,0,0,0,
	0,0,0,0,0,    1,0,0,0,0,
	1,0,1
};

lpc_cabac_t::lpc_cabac_t(int history_length)
{
	max_context = 1 << history_length;
	context = 0;

	st_table	= lpc_alloc<uint8_t>(max_context, "CABAC context");
	mps_table	= lpc_alloc<uint8_t>(max_context, "CABAC context");
	reset();
}

lpc_cabac_t::lpc_cabac_t(lpc_stream_out_t *stream, int history_length)
	: lpc_cabac_t(history_length)
{
	stream_out = stream;

	sc = 0;
	A_interval = 0x10000;
	C_register = 0;
	ct = 11;

	count = -1;
	BP = 0;
	bFirst = 0;
}

lpc_cabac_t::lpc_cabac_t(lpc_stream_in_t *stream, int history_length)
	: lpc_cabac_t(history_length)
{
	stream_in = stream;

	count = 0;

	MPS = 0;
	A_interval = 0x10000;
	C_register = 0;
	bEnd = 0;

	Byte_in( );
	C_register <<= 8;
	Byte_in( );
	C_register <<= 8;
	Cx = (unsigned) ((C_register & 0xffff0000) >> 16);

	ct = 0;
}


lpc_cabac_t::~lpc_cabac_t()
{
	lpc_free(st_table);
	lpc_free(mps_table);
}


void lpc_cabac_t::reset()
{
	memset(st_table, 0, max_context);
	memset(mps_table, 0, max_context);
}

void lpc_cabac_t::encode_bytes(const uint8_t *bytes, size_t byte_count)
{
	for (size_t i = 0; i < byte_count; i++)
	{
		uint8_t byte = bytes[i];
		for (int b = 0; b < 8; b++)
		{
			bool bit = (byte >> (7-b)) & 1;
			encode_bit(bit);
		}
	}
}

void lpc_cabac_t::encode_bit(bool symbol)
{
	LPC_ASSERT(context < max_context);

	next_st = cur_st = st_table[context];
	next_MPS = MPS = mps_table[context];
	Qe  = lsz[ st_table[context] ];

	if (MPS == symbol)
		Code_MPS();
	else
		Code_LPS();

	st_table[context] = next_st;
	mps_table[context] = next_MPS;

	context = (context << 1) | (unsigned)symbol;
	context &= (max_context - 1);
}


void lpc_cabac_t::encode(bool symbol, int prob, bool mps_symbol)
{
	next_st = cur_st = 0;
	next_MPS = MPS = mps_symbol;
	Qe  = prob;

	if (MPS == symbol)
		Code_MPS();
	else
		Code_LPS();
}


void lpc_cabac_t::Flush()
{
	Clear_final_bits();
	C_register <<= ct;
	Byte_out();
	C_register <<= 8;
	Byte_out();

	QMputc(BP);
	QMputc(0xff); count++;
	QMputc(0xff); count++;
}


void lpc_cabac_t::decode_bytes(uint8_t *bytes, size_t byte_count)
{
	for (size_t i = 0; i < byte_count; i++)
	{
		uint8_t byte = 0;
		for (int b = 0; b < 8; b++)
		{
			bool bit = decode_bit();
			byte |= (bit << (7-b));
		}

		bytes[i] = byte;
	}
}

bool lpc_cabac_t::decode_bit()
{
	next_st = cur_st = st_table[context];
	next_MPS = MPS = mps_table[context];
	Qe = lsz[ st_table[context] ];
	bool bit = AM_decode_Symbol();
	st_table[context] = next_st;
	mps_table[context] = next_MPS;

	context = (context << 1) | (unsigned)bit;
	context &= (max_context - 1);

	return bit;
}


bool lpc_cabac_t::decode(int prob, bool mps_symbol)
{
	next_st = cur_st = 0;
	next_MPS = MPS = mps_symbol;
	Qe = prob;
	return AM_decode_Symbol();
}


void lpc_cabac_t::Code_LPS()
{
	A_interval -= Qe;

	if (!(A_interval < Qe))
	{
		C_register += A_interval;
		A_interval = Qe;
	}

	if (swit[cur_st] == 1)
	{
		next_MPS = 1 - MPS;
	}
	next_st = nlps[cur_st];

	Renorm_e();
}


void lpc_cabac_t::Code_MPS()
{
	A_interval -= Qe;

	if (A_interval < 0x8000)
	{
		if (A_interval < Qe)
		{
			C_register += A_interval;
			A_interval = Qe;
		}
		next_st = nmps[cur_st];
		Renorm_e();
	}
}


void lpc_cabac_t::Renorm_e()
{
	while (A_interval < 0x8000)
	{
		A_interval <<= 1;
		C_register <<= 1;
		ct--;

		if (ct == 0)
		{
			Byte_out();
			ct = 8;
		}
	}
}


void lpc_cabac_t::Byte_out()
{
	unsigned t = C_register>>19;

	if (t > 0xff)
	{
		BP++;
		Stuff_0();
		Output_stacked_zeros();
		QMputc(BP); count++;
		BP = t;
	}
	else
	{
		if (t == 0xff)
		{
			sc++;
		}
		else
		{
			Output_stacked_0xffs();
			QMputc(BP); count++;
			BP = t;
		}
	}
	C_register &= 0x7ffff;
}


void lpc_cabac_t::Output_stacked_zeros()
{
	while (sc > 0)
	{
		QMputc(BP); count++;
		BP = 0;
		sc--;
	}
}


void lpc_cabac_t::Output_stacked_0xffs()
{
	while (sc > 0)
	{
		QMputc(BP); count++;
		BP = 0xFF;
		QMputc(BP); count++;
		BP = 0;
		sc--;
	}
}


void lpc_cabac_t::Stuff_0()
{
	if (BP == 0xff)
	{
		QMputc(BP); count++;
		BP = 0;
	}
}


void lpc_cabac_t::Clear_final_bits()
{
	unsigned long t;
	t = C_register + A_interval - 1;
	t &= 0xffff0000;

	if (t < C_register) t += 0x8000;

	C_register = t;
}


bool lpc_cabac_t::AM_decode_Symbol()
{
	bool D;

	A_interval -= Qe;

	if (Cx < A_interval)
	{
		if (A_interval < 0x8000)
		{
			D = Cond_MPS_exchange();
			Renorm_d();
		}
		else
			D = MPS;
	}
	else
	{
		D = Cond_LPS_exchange();
		Renorm_d();
	}

	return D;
}


bool lpc_cabac_t::Cond_LPS_exchange()
{
	bool D;
	unsigned C_low;

	if (A_interval < Qe)
	{
		D = MPS;
		Cx -= A_interval;
		C_low = C_register & 0x0000ffff;

		C_register = ((unsigned long)Cx<<16) + (unsigned long)C_low;
		A_interval = Qe;
		next_st = nmps[ cur_st ];
	}
	else
	{
		D = 1 - MPS;
		Cx -= A_interval;
		C_low = C_register & 0x0000ffff;
		C_register = ((unsigned long)Cx << 16) + (unsigned long)C_low;
		A_interval = Qe;

		if ( swit[ cur_st ]==1 )
		{
			next_MPS = 1-MPS;
		}
		next_st = nlps[ cur_st ];
	}

	return D;
}


bool lpc_cabac_t::Cond_MPS_exchange( )
{
	bool D;

	if (A_interval < Qe)
	{
		D = 1 - MPS;
		if (swit[cur_st] == 1)
		{
			next_MPS = 1 - MPS;
		}
		next_st = nlps[cur_st];
	}
	else
	{
		D = MPS;
		next_st = nmps[cur_st];
	}

	return D;
}


void lpc_cabac_t::Renorm_d( )
{
	while (A_interval<0x8000)
	{
		if (ct==0)
		{
			if (bEnd == 0) Byte_in();
			ct = 8;
		}
		A_interval <<= 1;
		C_register <<= 1;
		ct--;
	}

	Cx = (unsigned) ((C_register & 0xffff0000) >> 16);
}


void lpc_cabac_t::Byte_in( )
{
	unsigned char B;
	B = stream_in->read_byte(), count++;

	if (B == 0xff)
	{
		Unstuff_0();
	}
	else
	{
		C_register += (unsigned) B << 8;
	}
}


void
lpc_cabac_t::Unstuff_0( )
{
	unsigned char B;
	B = stream_in->read_byte(), count++;

	if( B == 0 )
	{
		C_register |= 0xff00;
	}
	else
	{
		if( B == 0xff )
		{
			//cerr << "\nEnd marker has been met!\n";
			bEnd = 1;
		}
	}
}


int lpc_cabac_t::Counting()
{
	if (ct == 0)
	{
		return count*8;
	}
	else
	{
		return count*8+8-ct;
	}
}
