#ifndef I2S_H
#define I2S_H

#ifdef __cplusplus
extern "C" {
#endif


// Config
#define DMA_BUFFER_SIZE 128

// Clock types for i2s_switch_clock
#define I2S_CLOCK_EXTERNAL     0
#define I2S_CLOCK_8K_INTERNAL  1
#define I2S_CLOCK_32K_INTERNAL 2
#define I2S_CLOCK_44K_INTERNAL 3
#define I2S_CLOCK_48K_INTERNAL 4

// Flags
extern volatile int Playing_Buff_A;

// Audio buffers with q15_t samples
extern int16_t Audio_Source_Blk_A[DMA_BUFFER_SIZE], Audio_Source_Blk_B[DMA_BUFFER_SIZE];


/*
 * I2S control methods
 */
 
int i2s_init(unsigned char clk);
void i2s_start(unsigned char useDMA);
void i2s_io_init(void);
void i2s_switch_clock(unsigned char clk);

/*
 * DMA for the I2S interface
 */

void dma_init(void);
void dma_play(void);
void dma_stop(void);

extern void dma_fill( int16_t *pBuf, int16_t len );


#ifdef __cplusplus
}
#endif
#endif
