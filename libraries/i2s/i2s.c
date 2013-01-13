#include <mk20dx128.h>
#include "i2s.h"

// Setup functions for i2s

// Timings assume 96MHz system clock (overclocked Teensy 3.0)

// You must implement dma_ch0_isr()



void i2s_io_init(void)
{
/*
    // set pin mux to output i2s
    PORTE_PCR6  &= PORT_PCR_MUX_MASK;
    PORTE_PCR7  &= PORT_PCR_MUX_MASK;
    PORTE_PCR10 &= PORT_PCR_MUX_MASK;
    PORTE_PCR11 &= PORT_PCR_MUX_MASK;
    PORTE_PCR12 &= PORT_PCR_MUX_MASK;
    PORTE_PCR6  |= PORT_PCR_MUX(0x04);
    PORTE_PCR7  |= PORT_PCR_MUX(0x04);
    PORTE_PCR10 |= PORT_PCR_MUX(0x04);
    PORTE_PCR11 |= PORT_PCR_MUX(0x04);
    PORTE_PCR12 |= PORT_PCR_MUX(0x04);
*/
}


void i2s_switch_clock(unsigned char clk)
{
    // The clock enable bit should be set by software [in the SIM] at the beginning of
    // the module initialization routine to enable the module clock before initialization of any of
    // the I2S/SAI registers.

    // Disable system clock to the I2S module
    //SIM_SCGC6 &= ~(SIM_SCGC6_I2S);
    SIM_SCGC6 |= SIM_SCGC6_I2S;

    // page 124
    // MCR[MICS]  = 00 (system clock)
    //              01 (osc0erclk - the RTC external reference clock)
    //              11 (mcgpllclk or mcgfllclk)
    // TCR2[MSEL] = 00 (bus clock)
    //              01 (I2S0_MCLK)

    // *** To change MICS, first reset the divider.

    if(clk==I2S_CLOCK_EXTERNAL)
    {
        I2S0_MCR |= I2S_MCR_MICS(0);   //I2S clock source is system clock
        // Configure to input the bit-clock from pin
        I2S0_MCR &= ~(I2S_MCR_MOE);
    }
    else
    {
        // Input Clock Select
        I2S0_MCR |= I2S_MCR_MICS(0);   //I2S clock source is system clock
        if(clk==I2S_CLOCK_44K_INTERNAL)
        {
            // Divide to get the 11.2896 MHz from 96MHz (96* (2/17))
            I2S0_MDR = I2S_MDR_DIVIDE(16) | I2S_MDR_FRACT(1);
        }
        else  /* I2S_CLOCK_48K_INTERNAL */
        {
            // Divide to get the 12.2880 MHz from 96MHz (96* (16/125))
            I2S0_MDR = I2S_MDR_DIVIDE(124) | I2S_MDR_FRACT(15);
        }
        // TCR2[MSEL] = 01 for I2S0_MCLK
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
    
    // Transmitter remains enabled until (and TE set) the end of the current frame
    for( i=0; i<1000 && (I2S0_TCSR & I2S_TCSR_TE); i++ );
    if( I2S0_TCSR & I2S_TCSR_TE )
        return -1;

    // Transmit enable
    I2S0_TCSR |= I2S_TCSR_TE;
    
    // transmit configuration register 2
    I2S0_TCR2 =
          I2S_TCR2_SYNC(00)   // asynchronous mode
       |  I2S_TCR2_BCI        // external bit clock
//     |  I2S_TCR2_BCP        // external bit clock is active low with drive outputs on falling edge, sample inputs on rising edge
//     |  I2S_TCR2_DIV(n) // divide internal master clock to generare bit clock
        ;

    // transmit configuration register 3
    I2S0_TCR3 =
        I2S_TCR3_TCE;       // transmit data channel is enabled

    I2S0_TCR4 =
        I2S_TCR4_FRSZ(2)    // frame size 2 words
      | I2S_TCR4_SYWD(2)    // sync width
      | I2S_TCR4_MF         // MSB first
      ;
    
    I2S0_TCR5 =
        I2S_TCR5_WNW(15)    // 16 bits per word excxept for first frame
      | I2S_TCR5_W0W(15)    // 16 bits per word there too
      ;

    // transmit enable
    I2S0_TCSR |=
        I2S_TCSR_TE     |    // Transmit Enable
        I2S_TCSR_BCE    |    // Bit Clock Enable
        I2S_TCSR_FRDE;       // FIFO Request DMA Enable

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
        *p1++ = 0;   // mute initially
        *p2++ = 0;   // mute initially
        *p3++ = 0;   // mute forever
    }  
}

void dma_mux_init(void)
{
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX;                // Enable clock to the DMAMUX module
    DMAMUX0_CHCFG0 = 0;                           // disable the channel to configure it 
    DMAMUX0_CHCFG0 = DMAMUX_CHCFG_SOURCE(15) ;    // I2S0 Transmit
    DMAMUX0_CHCFG0 |= DMAMUX_CHCFG_ENBL;          // enable the mux
}

void dma_transmit_init(void)  
{
    // Enable IRQ on the DMA channel 0
    // NVIC_ENABLE_IRQ(IRQ_DMA_ERROR);
    NVIC_ENABLE_IRQ(IRQ_DMA_CH0);

    // Set inactive
    DMA_TCD0_CSR &= ~(DMA_CSR_ACTIVE);
    
    // Control register
    DMA_CR = 0              // Normal
//      | DMA_CR_EDBG_MASK  // Stall DMA transfers when debugger is halted (avoid noise)
//      | DMA_CR_EMLM_MASK  // minor looping
        ;
 
    // Set channel priority
    DMA_DCHPRI3 = 0;
    DMA_DCHPRI2 = 0;
    DMA_DCHPRI1 = 0;
    DMA_DCHPRI0 = 15;  // cannot be pre-empted, can pre-empt, highest priority
  
    // fill the TCD regs
    DMA_TCD0_SADDR          = (uint32_t) Audio_Source_Blk_A ; // alternated with Audio_Source_Blk_B
    DMA_TCD0_SOFF           = 2;                              // 2 byte offset 
    DMA_TCD0_ATTR           = DMA_ATTR_SMOD(0) | DMA_ATTR_SSIZE(1) | DMA_ATTR_DMOD(0) | DMA_ATTR_DSIZE(1);   // no circular addressing S&D, 16 bit S&D 
    DMA_TCD0_NBYTES_MLNO    = 2;                              // one  16bit sample every minor loop 
    DMA_TCD0_SLAST          = 0;//-(DMA_BUFFER_SIZE*2);         // source address will always be newly written before each new start  DMA_TCD0_DADDR  
    DMA_TCD0_DADDR          = (uint32_t) &I2S0_TDR0;    // the FTM Channel 0 duty value
    DMA_TCD0_DOFF           = 0;
    DMA_TCD0_CITER_ELINKNO  = DMA_BUFFER_SIZE;         // total samples ( 128 )
    DMA_TCD0_BITER_ELINKNO  = DMA_BUFFER_SIZE;         // no chan links, total samples ( 128 )
    DMA_TCD0_DLASTSGA       = 0;                       // no final last adjustment ( does not move )
    DMA_TCD0_CSR            = DMA_CSR_INTMAJOR;        // interrupt when done  
  
    // configure DMA_MUX to trigger DMA channel 0  with FTM2 CH1 
    dma_mux_init();
  
    // enable chan0 for hardware trigger
    DMA_ERQ = DMA_ERQ_ERQ0;
  
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




 
