/*
 * I2S interface for teensy 3.0
 * (cc) https://creativecommons.org/licenses/by/3.0/ by Hugh Pyle and other contributors
 */
 
#ifndef I2S_H
#define I2S_H

#include <inttypes.h> 
#include <WProgram.h>

// Config
#define DMA_BUFFER_SIZE 128

#define I2S_AUDIO_NUM_CHANS  2                  // Not respected on Tx yet
#define I2S_AUDIO_BIT_DEPTH  32                 // Not respected on Tx yet

// Clock types
// Note: TX and RX share the same clock.
#define I2S_CLOCK_EXTERNAL     0
#define I2S_CLOCK_8K_INTERNAL  1
#define I2S_CLOCK_32K_INTERNAL 2
#define I2S_CLOCK_44K_INTERNAL 3
#define I2S_CLOCK_48K_INTERNAL 4

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
#define I2S_TX_PIN_PATTERN_1   0x01           // Transmit pins 3, 4, 9 (no MCLK)
#define I2S_TX_PIN_PATTERN_2   0x02           // Transmit pins 3, 4, 9, 11 (MCLK on 11)
#define I2S_TX_PIN_PATTERN_3   0x03           // Transmit pins 22, 23, 9 (no MCLK)
#define I2S_TX_PIN_PATTERN_4   0x04           // Transmit pins 22, 23, 9, 11 (MCLK on 11)
#define I2S_TX_PIN_PATTERN_5   0x05           // Transmit pins 3, 4, 24 (no MCLK)
#define I2S_TX_PIN_PATTERN_6   0x06           // Transmit pins 3, 4, 24, 28 (MCLK on 28)

#define I2S_RX_PIN_PATTERN_1   0x10           // Receive pins 11, 12, 13 (no MCLK)
#define I2S_RX_PIN_PATTERN_2   0x20           // Receive pins 11, 12, 13, 28 (MCLK on 28)
#define I2S_RX_PIN_PATTERN_3   0x30           // Receive pins 11, 12, 13, 27 (MCLK on 11)
#define I2S_RX_PIN_PATTERN_4   0x40           // Receive pins 27, 29, 13 (no MCLK)
#define I2S_RX_PIN_PATTERN_5   0x50           // Receive pins 27, 29, 13, 28 (MCLK on 28)
#define I2S_RX_PIN_PATTERN_6   0x60           // Receive pins 27, 29, 13, 11 (MCLK on 11)

class I2S_class
{
    private:
        // Flags
        bool receive;
        bool useDMA;
        unsigned char clockType;
        unsigned char pinPattern;
        volatile bool _dma_using_Buffer_A;
        void (*fnDMACallback)( int16_t *pBuf, int16_t len );
        
        void init();
        void io_init();
        void clock_init(unsigned char clk);
        void i2s_transmit_init();
        void i2s_receive_init();
        void dma_buffer_init();
        void dma_transmit_init();
        void dma_receive_init();
        void dma_start();
        void dma_stop();
        
    public:
        /* Don't construct your own, there are two ready-made instances, one for receive and one for transmit */
        I2S_class(bool isRx);
        
        /*
         * @brief Initialize the I2S interface for use without DMA.  You must implement the i2s0_tx_isr() and/or i2s0_rx_isr() callback functions.
         * @param[in]   clk         Clock type, one of I2S_CLOCK_xxx
         * @param[in]   pinpattern  Pin pattern, one of I2S_TX_PIN_PATTERN_n or I2S_RX_PIN_PATTERN_n
         * @return none.
        */
        void begin(unsigned char clk, unsigned char pinpattern);
        
        /*
         * @brief Initialize the I2S interface for use with DMA.  You must implement the callback function that you pass in to start().
         * @param[in]   clk         Clock type, one of I2S_CLOCK_xxx
         * @param[in]   pinpattern  Pin pattern, one of I2S_TX_PIN_PATTERN_n or I2S_RX_PIN_PATTERN_n
         * @param[in]   fptr        Your callback function.  This will be called, with a pointer to a buffer where you should write int16_t audio data.
         * @return none.
        */
        void begin(unsigned char clk, unsigned char pinpattern, void (*fptr)( int16_t *pBuf, int16_t len ));

        /*
         * @brief Start the I2S interface.  NOT YET IMPLEMENTED
         * @return none.
        */
        void start();
        
        /*
         * @brief Stop the I2S interface.  NOT YET IMPLEMENTED
         * @return none.
        */
        void stop();

        /* internal */
        inline void dma_tx_callback(void);
        inline void dma_rx_callback(void);
};


extern I2S_class I2STx0;
extern I2S_class I2SRx0;

#endif

