/*
 * I2S interface for teensy 3.0
 * (cc) https://creativecommons.org/licenses/by/3.0/ by Hugh Pyle
 */
 
#ifndef I2S_H
#define I2S_H

#include <inttypes.h> 

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
        // Buffers with 16 bit audio samples
        int16_t _dma_Buffer_A[DMA_BUFFER_SIZE];
        int16_t _dma_Buffer_B[DMA_BUFFER_SIZE];
        int16_t _dma_Buffer_S[DMA_BUFFER_SIZE];  // to be used during mute
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
        /* */
        // Construct without DMA, caller must implement i2s0_tx_isr().
        void start(unsigned char clk);
        
        /* */
        // Construct with DMA, caller must implement the callback function
        void start(unsigned char clk, void (*fptr)( int16_t *pBuf, int16_t len ));
        
        /* */
        void stop();
        
        /* internal */
        void dma_callback(void);
};


extern I2S_class I2Simpl;


#endif

