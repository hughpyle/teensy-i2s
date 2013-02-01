/*
 * I2S interface for teensy 3.0
 * (cc) https://creativecommons.org/licenses/by/3.0/ by Hugh Pyle
 */
 
#ifndef I2S_H
#define I2S_H

#include <inttypes.h> 
#include <WProgram.h>

// Config
#define DMA_BUFFER_SIZE 128

// Clock types for i2s_switch_clock
#define I2S_CLOCK_EXTERNAL     0
#define I2S_CLOCK_8K_INTERNAL  1
#define I2S_CLOCK_32K_INTERNAL 2
#define I2S_CLOCK_44K_INTERNAL 3
#define I2S_CLOCK_48K_INTERNAL 4


class I2S_class
{
    private:
        // Flags
        bool useDMA;
        unsigned char clockType;
        volatile int _dma_Playing_Buffer_A;
        void (*fnDMACallback)( int16_t *pBuf, int16_t len );
        
        void init();
        void io_init();
        void switch_clock(unsigned char clk);
        void startWithDMA();
        void startWithoutDMA();
        void dma_init();
        void dma_transmit_init();
        void dma_play();
        void dma_stop();
        void clearAudioBuffers();
        
    public:
        /*
         * @brief Initialize the I2S interface for use without DMA.  You must implement the i2s0_tx_isr() callback function.
         * @param[in]   clk    Clock type.
         * @return none.
        */
        void start(unsigned char clk);
        
        /*
         * @brief Initialize the I2S interface for use with DMA.  You must implement the callback function.
         * @param[in]   clk    Clock type.
         * @param[in]   fptr   Your callback function.  This will be called, with a pointer to a buffer where you should write int16_t audio data.
         * @return none.
        */
        void start(unsigned char clk, void (*fptr)( int16_t *pBuf, int16_t len ));
        
        /*
         * @brief Stop the I2S interface.  NOT YET IMPLEMENTED
         * @return none.
        */
        void stop();
        
        /* internal */
        void dma_callback(void);
};


extern I2S_class I2Simpl;


#endif

