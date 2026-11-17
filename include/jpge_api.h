#pragma once

#include <stddef.h>
#include <stdint.h>

namespace jpge
{
        typedef unsigned char  uint8;
        typedef signed short   int16;
        typedef signed int     int32;
        typedef unsigned short uint16;
        typedef unsigned int   uint32;
        typedef size_t   uint;

        enum subsampling_t { Y_ONLY = 0, H1V1 = 1, H2V1 = 2, H2V2 = 3 };

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

                int m_quality;
                subsampling_t m_subsampling;
        };

        class output_stream {
        public:
                virtual ~output_stream() { };
                virtual bool put_buf(const void* Pbuf, int len) = 0;
                virtual uint get_size() const = 0;
        };

        class jpeg_encoder {
        public:
                jpeg_encoder();
                ~jpeg_encoder();
                bool init(output_stream* pStream, int width, int height, int src_channels, const params& comp_params = params());
                bool process_scanline(const void* pScanline);
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
