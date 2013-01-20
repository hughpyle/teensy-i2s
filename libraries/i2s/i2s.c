#include <mk20dx128.h>
#include "i2s.h"

// Setup functions for i2s

// Timings assume 96MHz system clock (overclocked Teensy 3.0)

// You must implement dma_ch0_isr()


void i2s_io_init(void)
{
    // set pin mux to output i2s
    // using the ALT6 pin selections
    // SCK  -> Teensy 9 (ALT6 I2S0_TX_BCLK) (PTC3)
    // MOSI -> Teensy 3 (ALT6 I2S0_TXD0) (PTA12)
    // DACL -> Teensy 4 (ALT6 I2S0_TX_FS) (PTA13)
    // xxxx -> Teensy 11 (ALT6 I2S0_MCLK) (PTC6) -- available if you need it (12.288MHz)
    
    PORTC_PCR3  &= PORT_PCR_MUX_MASK;
    PORTC_PCR3  |= PORT_PCR_MUX(0x06);

    PORTA_PCR12 &= PORT_PCR_MUX_MASK;
    PORTA_PCR12 |= PORT_PCR_MUX(0x06);
    
    PORTA_PCR13 &= PORT_PCR_MUX_MASK;
    PORTA_PCR13 |= PORT_PCR_MUX(0x06);
    
//  PORTC_PCR6  &= PORT_PCR_MUX_MASK;
//  PORTC_PCR6  |= PORT_PCR_MUX(0x06);
}


void i2s_switch_clock(unsigned char clk)
{
    // The clock enable bit should be set by software [in the SIM] at the beginning of
    // the module initialization routine to enable the module clock before initialization of any of
    // the I2S/SAI registers.

    // Disable system clock to the I2S module
    //SIM_SCGC6 &= ~(SIM_SCGC6_I2S);
    SIM_SCGC6 |= SIM_SCGC6_I2S;

    if(clk==I2S_CLOCK_EXTERNAL)
    {
        // Select input clock 0
        I2S0_MCR |= I2S_MCR_MICS(0);
        // Configure to input the bit-clock from pin
        I2S0_MCR &= ~(I2S_MCR_MOE);
    }
    else
    {
        // Select input clock 0 and output enable
        I2S0_MCR |= I2S_MCR_MICS(0) | I2S_MCR_MOE;
        
        // 8k, 12k, 16k, 32k, etc all clock the I2S module at 12.288 MHz
        // 11025Hz, 22050, 44100 clock the I2S module at 11.2896 MHz
        switch( clk )
        {
            case I2S_CLOCK_44K_INTERNAL:
                // Divide to get the 11.2896 MHz from 96MHz (96* (2/17))
                I2S0_MDR = I2S_MDR_FRACT(1) | I2S_MDR_DIVIDE(16);
                break;
            case I2S_CLOCK_8K_INTERNAL:
            case I2S_CLOCK_32K_INTERNAL:
            case I2S_CLOCK_48K_INTERNAL:
            default:
                // Divide to get the 12.2880 MHz from 96MHz (96* (16/125))
                I2S0_MDR = I2S_MDR_FRACT(15) | I2S_MDR_DIVIDE(124);
                break;
        }
    }

    // re-enable system clock to the I2S module
    SIM_SCGC6 |= SIM_SCGC6_I2S;
}



int i2s_init(unsigned char clk)
{
    int i;
    i2s_io_init();
    i2s_switch_clock( clk );
 
    // transmit disable
    I2S0_TCSR &= ~(I2S_TCSR_TE);
    I2S0_TCR3 = 0;
    
    // Transmitter remains enabled until (and TE set) the end of the current frame
    for( i=0; i<1000 && (I2S0_TCSR & I2S_TCSR_TE); i++ );
    if( I2S0_TCSR & I2S_TCSR_TE )
        return -1;

    // transmit configuration register 2
    if(clk==I2S_CLOCK_EXTERNAL)
    {
        I2S0_TCR2 =
              I2S_TCR2_SYNC(0)   // asynchronous mode
            | I2S_TCR2_BCI        // external bit clock
        ;
    }
    else
    {
        I2S0_TCR2 =
              I2S_TCR2_SYNC(0)      // asynchronous mode
            | I2S_TCR2_DIV(15)      // divide internal master clock to generate bit clock
            | I2S_TCR2_BCD          // bit clock direction: generated internally in Master mode
            | I2S_TCR2_MSEL(0)      // I2S0_MCLK as source
        ;
    }

    // transmit configuration register 3
    I2S0_TCR3 =
        I2S_TCR3_TCE;       // transmit data channel is enabled
                            // First word sets the word flag
        
    if(clk==I2S_CLOCK_EXTERNAL)
    {
        I2S0_TCR4 =
            I2S_TCR4_FRSZ(1)    // frame size 2 words (1 plus one)
          | I2S_TCR4_SYWD(3)    // length of frame sync in bit clocks (3 plus one)
          | I2S_TCR4_MF         // MSB first
          ;
    }
    else
    {
        I2S0_TCR4 =
            I2S_TCR4_FRSZ(1)    // frame size 2 words (1 plus one)
          | I2S_TCR4_SYWD(3)    // length of frame sync in bit clocks (3 plus one)
          | I2S_TCR4_MF         // MSB first
          | I2S_TCR4_FSD        // Frame sync is generated internally (master)
          | I2S_TCR4_FSE        // Frame sync one bit before the frame
          ;
    }
    
    I2S0_TCR5 =
        I2S_TCR5_WNW(15)        // 16 bits per word excxept for first frame
      | I2S_TCR5_W0W(15)        // 16 bits per word there too
      ;

    // transmit enable
    I2S0_TCSR |=
        I2S_TCSR_TE     |       // Transmit Enable
        I2S_TCSR_BCE    |       // Bit Clock Enable
        I2S_TCSR_FRDE;          // FIFO Request DMA Enable

    return 0;
}


/*
 * DMA
 */
 
// 16 bit audio samples - not volatile because only read by interrupts/hw
int16_t Audio_Source_Blk_A[DMA_BUFFER_SIZE], Audio_Source_Blk_B[DMA_BUFFER_SIZE];

// to be used during mute 
int16_t Audio_Silence[DMA_BUFFER_SIZE];                                        
    
volatile int Playing_Buff_A; 


void clearAudioBuffers(void)
{
    int i;
    int16_t *p1,*p2,*p3;
    p1 = Audio_Source_Blk_A;
    p2 = Audio_Source_Blk_B;
    p3 = Audio_Silence;
    for (i=0 ; i < DMA_BUFFER_SIZE; i++) 
    {
        *p1++ = i+0;   // mute initially
        *p2++ = i+0;   // mute initially
        *p3++ = i+0;   // mute forever
    }  
}


#define LOOP2

void dma_transmit_init(void)  
{
    // Enable clock to the DMAMUX module
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX;
    
    // configure DMA_MUX
    DMAMUX0_CHCFG0 = 0;
    DMAMUX0_CHCFG0 = DMAMUX_CHCFG_SOURCE(DMAMUX_SOURCE_I2S0_TX);
    DMAMUX0_CHCFG0 |= DMAMUX_CHCFG_ENBL /* | DMAMUX_CHCFG_TRIG */;

    // Enable IRQ on the DMA channel 0
    // NVIC_ENABLE_IRQ(IRQ_DMA_ERROR);
    NVIC_ENABLE_IRQ(IRQ_DMA_CH0);

    // Set inactive
    DMA_TCD0_CSR &= ~(DMA_CSR_ACTIVE);
    
    // Set channel priorities (each must be unique)
    DMA_DCHPRI3 = 0;
    DMA_DCHPRI2 = 1;
    DMA_DCHPRI1 = 2;
    DMA_DCHPRI0 = 3;  // cannot be pre-empted, can pre-empt, highest priority

#ifdef LOOP1
    // Control register
    DMA_CR = 0              // Normal
//      | DMA_CR_EDBG_MASK  // Stall DMA transfers when debugger is halted (avoid noise)
//      | DMA_CR_EMLM_MASK  // minor looping
        ;
   
    // fill the TCD regs
    DMA_TCD0_SADDR          = (uint32_t) Audio_Source_Blk_A ;       // alternated with Audio_Source_Blk_B by our interrupt handler
    DMA_TCD0_SOFF           = 2;                                    // 2 byte offset 
    DMA_TCD0_ATTR           = DMA_ATTR_SMOD(0)
                            | DMA_ATTR_SSIZE(DMA_ATTR_SIZE_16BIT)
                            | DMA_ATTR_DMOD(0)
                            | DMA_ATTR_DSIZE(DMA_ATTR_SIZE_16BIT);  // Transfer one word at a time
    DMA_TCD0_NBYTES_MLNO    = 2;                                    // one  16bit sample every minor loop
    DMA_TCD0_SLAST          = 0;//-(DMA_BUFFER_SIZE*2);             // source address will always be newly written before each new start
    DMA_TCD0_DADDR          = (uint32_t) &I2S0_TDR0;                // Destination is the I2S data register
    DMA_TCD0_DOFF           = 0;                                    // Zero offset
    DMA_TCD0_DLASTSGA       = 0;                                    // no scatter/gather
    DMA_TCD0_CITER_ELINKNO  = DMA_BUFFER_SIZE & DMA_CITER_MASK;     // total samples (128)
    DMA_TCD0_BITER_ELINKNO  = DMA_BUFFER_SIZE & DMA_BITER_MASK;     // no chan links, total samples (128)
    DMA_TCD0_CSR            = DMA_CSR_INTMAJOR;                     // interrupt on major loop completion
#endif

#ifdef LOOP2
    // Control register
    DMA_CR = DMA_CR_EMLM  // minor looping
        ;
   
    // fill the TCD regs
    DMA_TCD0_NBYTES_MLOFFYES = DMA_NBYTES_SMLOE
                            | DMA_NBYTES_MLOFFYES_MLOFF(2 - DMA_BUFFER_SIZE*2*2*2)
                            | DMA_NBYTES_MLOFFYES_NBYTES(2*2);
    DMA_TCD0_ATTR           = DMA_ATTR_SSIZE(DMA_ATTR_SIZE_16BIT)
                            | DMA_ATTR_DSIZE(DMA_ATTR_SIZE_16BIT);  // Transfer one word at a time, no modulo
    DMA_TCD0_SADDR          = (uint32_t) Audio_Source_Blk_A ;       // alternated with Audio_Source_Blk_B by our interrupt handler
    DMA_TCD0_SOFF           = DMA_BUFFER_SIZE*2*2;
    DMA_TCD0_SLAST          = -(DMA_BUFFER_SIZE*2*2*3 - 2);0;
    DMA_TCD0_DADDR          = (uint32_t) &I2S0_TDR0;                // Destination is the I2S data register
    DMA_TCD0_DOFF           = 0;                                    // Zero offset
    DMA_TCD0_DLASTSGA       = 0;                                    // no scatter/gather
    DMA_TCD0_CITER_ELINKNO  = (DMA_BUFFER_SIZE*2) & DMA_CITER_MASK;     // total samples (128)
    DMA_TCD0_BITER_ELINKNO  = (DMA_BUFFER_SIZE*2) & DMA_BITER_MASK;     // no chan links, total samples (128)
    DMA_TCD0_CSR            = DMA_CSR_INTMAJOR | DMA_CSR_INTHALF;
#endif

    // enable DMA channel 0 requests
    //DMA_ERQ = DMA_ERQ_ERQ0;
    DMA_SERQ = DMA_SERQ_SERQ(0);
  
    // Set active
    DMA_TCD0_CSR |= DMA_CSR_ACTIVE;

    // To initiate from software, set DMA_CSR[start]
    //DMA_TCD0_CSR |= DMA_CSR_START;
}



/*
void hal_dma_init_for_i2s(uint buf_rx, uint buf_tx, uint block_n_sample, uint sample_n_byte)
{
    uint size_bit;
    DMA_TCD tcd;    
    switch(sample_n_byte)
    {
        case 1: size_bit = DMA_SIZE_8_BIT; break;
        case 2: size_bit = DMA_SIZE_16_BIT; break;
        case 4: size_bit = DMA_SIZE_32_BIT; break;
        default: size_bit = DMA_SIZE_32_BIT; break;
    }
    
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
    DMAMUX_CHCFG0 = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(DMA_SRC_I2S_T);
    
    DMA_CR |= DMA_CR_EMLM_MASK;
    DMA_CSR(0) = DMA_CSR_INTHALF_MASK | DMA_CSR_INTMAJOR_MASK;
    nvic_enable_irq(IRQ_DMA0);
    
    tcd.channel = 0;
    tcd.nbytes = DMA_NBYTES_MLOFFYES_SMLOE_MASK |
                 DMA_NBYTES_MLOFFYES_MLOFF(sample_n_byte - block_n_sample*2*sample_n_byte*2)|
                 DMA_NBYTES_MLOFFYES_NBYTES(sample_n_byte*2);
    tcd.attr = DMA_ATTR_SSIZE(size_bit) | DMA_ATTR_DSIZE(size_bit);
    tcd.saddr = buf_tx;
    tcd.soff = block_n_sample*2*sample_n_byte;
    tcd.slast = -(block_n_sample*2*sample_n_byte*3 - sample_n_byte);
    tcd.daddr = (uint)(&I2S0_TX0);
    tcd.doff = 0;
    tcd.dlast_sga = 0;
    tcd.citer = block_n_sample*2;
    tcd.biter = block_n_sample*2;
    _dma_init(&tcd);
    
    // enable DMA channel
    DMA_SERQ = DMA_SERQ_SERQ(0);
}
*/




void dma_init(void)  
{
    clearAudioBuffers();
    dma_transmit_init();
    Playing_Buff_A = 1;
}

void dma_play(void) 
{
    clearAudioBuffers();
    DMA_ERQ |= DMA_ERQ_ERQ0;  
}

void dma_stop(void) 
{
    DMA_TCD0_SADDR = (uint32_t) Audio_Silence; // silence feed for DMA0
    clearAudioBuffers();
    DMA_ERQ &= ~DMA_ERQ_ERQ0;
}

