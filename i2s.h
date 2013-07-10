/*
 * I2S interface for Teensy 3.0
 * Fork this on github https://github.com/hughpyle/teensy-i2s
 *
 * Copyright (c) 2013 by Hugh Pyle and contributors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */
 
#ifndef __I2S_H__
#define __I2S_H__

#include <inttypes.h> 
#include <WProgram.h>

// Audio configuration.  Edit these here if you need to.
#define I2S_FRAME_SIZE         2            // Number of frames, 2=stereo
#define I2S_IO_BIT_DEPTH       16           // Number of bits per sample in the physical data (8, 16 or 32)
#define I2S_BUFFER_BIT_DEPTH   16           // Number of bits per sample in the DMA buffer (8, 16 or 32)

// Clock type constants
#define I2S_CLOCK_EXTERNAL     0            // The bit clock is provided by an external device (e.g. the codec)
#define I2S_CLOCK_8K_INTERNAL  1            // The bit clock is 8kHz, internally generated
#define I2S_CLOCK_32K_INTERNAL 2            // The bit clock is 32kHz, internally generated
#define I2S_CLOCK_44K_INTERNAL 3            // The bit clock is 44.1kHz, internally generated
#define I2S_CLOCK_48K_INTERNAL 4            // The bit clock is 48kHz, internally generated


// Pin patterns
// Teensy 3.0 hardware has several ways to configure its I2S pins:
//      pin     alt4            alt6
//      3                       I2S0_TXD0           (transmit data, also on 22)
//      4                       I2S0_TX_FS          (transmit word clock, also on 23, 25)
//      9                       I2S0_TX_BCLK        (transmit bit clock, also on 24, 32)
//      11      I2S0_RX_BCLK    I2S0_MCLK           (receive bit clock, also on 27; or master clock, also on 28)
//      12      I2S0_RX_FS                          (receive word clock, also on 29)
//      13      I2S0_RXD0                           (receive data)
//      22                      I2S0_TXD0           (transmit data, also on 22)
//      23                      I2S0_TX_FS          (also on 4, 25)
//      24                      I2S0_TX_BCLK        (also on 9, 32)
//      25                      I2S0_TX_FS          (also on 4, 23)
//      27      I2S0_RX_BCLK                        (also on 11)
//      28      I2S0_MCLK                           (also on 11)
//      29      I2S0_RX_FS                          (also on 12)
//      32      I2S0_TX_BCLK                        (also on 9, 24)
// Pins 24 onward are pads on the bottom of the board, not pins on the edges.
// 
// Some combinations of these are defined in the macros I2S_PIN_PATTERN_<N>.
// Not all combinations of TX and RX can be used together (you need to decide which role for pin 11).
#define I2S_TX_PIN_PATTERN_1   0x01         // Transmit pins 3, 4, 9 (no MCLK)
#define I2S_TX_PIN_PATTERN_2   0x02         // Transmit pins 3, 4, 9, 11 (MCLK on 11)
#define I2S_TX_PIN_PATTERN_3   0x03         // Transmit pins 22, 23, 9 (no MCLK)
#define I2S_TX_PIN_PATTERN_4   0x04         // Transmit pins 22, 23, 9, 11 (MCLK on 11)
#define I2S_TX_PIN_PATTERN_5   0x05         // Transmit pins 3, 4, 24 (no MCLK)
#define I2S_TX_PIN_PATTERN_6   0x06         // Transmit pins 3, 4, 24, 28 (MCLK on 28)

#define I2S_RX_PIN_PATTERN_1   0x10         // Receive pins 11, 12, 13 (no MCLK)
#define I2S_RX_PIN_PATTERN_2   0x20         // Receive pins 11, 12, 13, 28 (MCLK on 28)
#define I2S_RX_PIN_PATTERN_3   0x30         // Receive pins 11, 12, 13, 27 (MCLK on 11)
#define I2S_RX_PIN_PATTERN_4   0x40         // Receive pins 27, 29, 13 (no MCLK)
#define I2S_RX_PIN_PATTERN_5   0x50         // Receive pins 27, 29, 13, 28 (MCLK on 28)
#define I2S_RX_PIN_PATTERN_6   0x60         // Receive pins 27, 29, 13, 11 (MCLK on 11)

// You should define I2S_PIN_PATTERN to be the appropriate pin pattern for your hardware setup.
// Otherwise the default is this
#define I2S_PIN_PATTERN        I2S_TX_PIN_PATTERN_1 + I2S_RX_PIN_PATTERN_1


// DMA buffer size (in samples).
// Using ping-pong DMA, this determines your latency.
// If you need super-low latency, set this smaller (or use I2S without DMA).
#define DMA_BUFFER_SIZE        128

// Use round-robin DMA channel priorities?  If not, they're explicitly set
#define ROUNDROBIN

// Data type for the API
#if I2S_BUFFER_BIT_DEPTH==8
#define _I2S_SAMPLE_T          int8_t
#elif I2S_BUFFER_BIT_DEPTH==16
#define _I2S_SAMPLE_T          int16_t
#else
#define _I2S_SAMPLE_T          int32_t
#endif


class I2S_class
{
    private:
        // Flags
        uint8_t clock;      /* one of I2S_CLOCK_xxx */
        uint8_t receive;    /* 1 or 0 */
        bool useDMA;
        volatile bool _dma_using_Buffer_A;
        void (*fnI2SCallback)( _I2S_SAMPLE_T *pBuf );                      // the I2S callback (buffer size = I2S_FRAME_SIZE)
        void (*fnDMACallback)( _I2S_SAMPLE_T *pBuf, uint16_t numSamples ); // the DMA callback (buffer size = DMA_BUFFER_SIZE)
        
        void init();
        void io_init();
        void clock_init();
        void i2s_transmit_init();
        void i2s_receive_init();
        void dma_buffer_init();
        void dma_transmit_init();
        void dma_receive_init();
        void dma_start();
        void dma_stop();
        
    public:
        /* Don't construct your own, there are two ready-made instances, one for receive and one for transmit */
        I2S_class(uint8_t isRx);
        
        /*
         * @brief       Initialize the I2S interface for use without DMA.
         *
         * @param[in]   clk     The clock type and speed, one of I2S_CLOCK_xxx
         * @param[in]   fptr    The callback function that your sketch implements.
         *                      This will be called with a pointer to a buffer
         *                      where you will read or write I2S_FRAME_SIZE of _I2S_SAMPLE_T audio data.
         * @return      none.
         */
        void begin(uint8_t clk, void (*fptr)( _I2S_SAMPLE_T *pBuf ));
        
        /*
         * @brief       Initialize the I2S interface for use with DMA.
         *
         * @param[in]   clk     The clock type and speed, one of I2S_CLOCK_xxx
         * @param[in]   fptr    The callback function that your sketch implements.
         *                      This will be called with a pointer to a buffer
         *                      where you will read or write numSamples of _I2S_SAMPLE_T audio data.
         * @return      none.
         *
         * TODO !!!receive with DMA is not yet implemented!!! (transmit is ok)
         */
        void begin(uint8_t clk, void (*fptr)( _I2S_SAMPLE_T *pBuf, uint16_t numSamples ));

        /*
         * @brief   Start the I2S interface.  (You must have initialized first).
         * @return  none.
         */
        void start();
        
        /*
         * @brief   Stop the I2S interface.  (You must have initialized first).
         * @return  none.
         */
        void stop();

        /* internal */
        inline void i2s_tx_callback(void);
        inline void i2s_rx_callback(void);
        inline void dma_tx_callback(void);
        inline void dma_rx_callback(void);
};


extern I2S_class I2STx0;
extern I2S_class I2SRx0;

#endif
