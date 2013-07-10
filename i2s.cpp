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
 
#include <i2s.h>
#include <mk20dx128.h>
#include "core_pins.h"

// There's one instance of the class for Tx, another for Rx
I2S_class I2STx0(0);
I2S_class I2SRx0(1);

// Buffers for 16 bit audio samples.
// Static (not class members) because we want to put them in the DMAMEM area.
// DMA:
static _I2S_SAMPLE_T DMAMEM _dma_Rx_Buffer_A[DMA_BUFFER_SIZE];
static _I2S_SAMPLE_T DMAMEM _dma_Rx_Buffer_B[DMA_BUFFER_SIZE];
static _I2S_SAMPLE_T DMAMEM _dma_Tx_Buffer_A[DMA_BUFFER_SIZE];
static _I2S_SAMPLE_T DMAMEM _dma_Tx_Buffer_B[DMA_BUFFER_SIZE];
// I2S:
static _I2S_SAMPLE_T _i2s_Rx_Buffer[I2S_FRAME_SIZE];
static _I2S_SAMPLE_T _i2s_Tx_Buffer[I2S_FRAME_SIZE];



I2S_class::I2S_class(uint8_t isRx)
{
    receive = isRx;
}

/* Initialize the I2S interface for use without DMA. */
void I2S_class::begin(uint8_t clk, void (*fptr)( _I2S_SAMPLE_T *pBuf))
{
    clock = clk;
    useDMA = false;
    fnI2SCallback = fptr;
    init();
}

/* Initialize the I2S interface for use with DMA. */
void I2S_class::begin(uint8_t clk, void (*fptr)( _I2S_SAMPLE_T *pBuf, uint16_t numSamples ))
{
    clock = clk;
    useDMA = true;
    fnDMACallback = fptr;
    init();
}


void I2S_class::start()
{
    if( useDMA )
    {
        // When FIFO needs data it generates a DMA request.
        if( receive )
        {
            // Receive enable
            I2S0_RCSR |= I2S_RCSR_RE            // Receive Enable
                       | I2S_RCSR_BCE           // Bit Clock Enable
                       | I2S_RCSR_FRDE          // FIFO Request DMA Enable
                       | I2S_RCSR_FR            // FIFO Reset
                       ;
        }
        else
        {
            // Transmit enable
            I2S0_TCSR |= I2S_TCSR_TE            // Transmit Enable
                       | I2S_TCSR_BCE           // Bit Clock Enable
                       | I2S_TCSR_FRDE          // FIFO Request DMA Enable
                       | I2S_TCSR_FR            // FIFO Reset
                       ;
        }
    }
    else
    {
        // When FIFO needs data it generates an interrupt.
        if( receive )
        {
            // Receive enable
            NVIC_ENABLE_IRQ(IRQ_I2S0_RX);
            I2S0_RCSR |= I2S_RCSR_RE            // Receive Enable
                       | I2S_RCSR_BCE           // Bit Clock Enable
                       | I2S_RCSR_FRIE          // FIFO Request Interrupt Enable
                       | I2S_RCSR_FR            // FIFO Reset
                       ;
        }
        else
        {
            // Transmit enable
            NVIC_ENABLE_IRQ(IRQ_I2S0_TX);
            I2S0_TCSR |= I2S_TCSR_TE            // Transmit Enable
                       | I2S_TCSR_BCE           // Bit Clock Enable
                       | I2S_TCSR_FRIE          // FIFO Request Interrupt Enable
                       | I2S_TCSR_FR            // FIFO Reset
                       ;
        }
    }
}

void I2S_class::stop()
{
    if( useDMA )
    {
        if( receive )
        {
            NVIC_DISABLE_IRQ(IRQ_DMA_CH1);
        }
        else
        {
            NVIC_DISABLE_IRQ(IRQ_DMA_CH0);
        }
    }
    else
    {
        if( receive )
        {
            NVIC_DISABLE_IRQ(IRQ_I2S0_RX);
        }
        else
        {
            NVIC_DISABLE_IRQ(IRQ_I2S0_TX);
        }
    }
}



void I2S_class::io_init(void)
{
    // Pins for transmit
    switch( ( I2S_PIN_PATTERN) & 0x0F )
    {
        case I2S_TX_PIN_PATTERN_1:
            CORE_PIN3_CONFIG  = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TXD0
            CORE_PIN4_CONFIG  = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TX_FS
            CORE_PIN9_CONFIG  = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TX_BCLK
            break;
        case I2S_TX_PIN_PATTERN_2:
            CORE_PIN3_CONFIG  = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TXD0
            CORE_PIN4_CONFIG  = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TX_FS
            CORE_PIN9_CONFIG  = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TX_BCLK
            CORE_PIN11_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_MCLK
            break;
        case I2S_TX_PIN_PATTERN_3:
            CORE_PIN22_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TXD0
            CORE_PIN23_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TX_FS
            CORE_PIN9_CONFIG  = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TX_BCLK
            break;
        case I2S_TX_PIN_PATTERN_4:
            CORE_PIN22_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TXD0
            CORE_PIN23_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TX_FS
            CORE_PIN9_CONFIG  = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TX_BCLK
            CORE_PIN11_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_MCLK
            break;
        case I2S_TX_PIN_PATTERN_5:
            CORE_PIN3_CONFIG  = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TXD0
            CORE_PIN4_CONFIG  = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TX_FS
            CORE_PIN24_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TX_BCLK
            break;
        case I2S_TX_PIN_PATTERN_6:
            CORE_PIN3_CONFIG  = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TXD0
            CORE_PIN4_CONFIG  = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TX_FS
            CORE_PIN24_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TX_BCLK
            CORE_PIN28_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_MCLK
            break;
        default:
            break;
    }
 
    // Pins for receive
    switch( (I2S_PIN_PATTERN) & 0xF0 )
    {
        case I2S_RX_PIN_PATTERN_1:
            CORE_PIN11_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RX_BCLK
            CORE_PIN12_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RX_FS
            CORE_PIN13_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RXD0
            break;
        case I2S_RX_PIN_PATTERN_2:
            CORE_PIN11_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RX_BCLK
            CORE_PIN12_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RX_FS
            CORE_PIN13_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RXD0
            CORE_PIN28_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_MCLK
            break;
        case I2S_RX_PIN_PATTERN_3:
            CORE_PIN11_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_MCLK
            CORE_PIN12_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RX_FS
            CORE_PIN13_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RXD0
            CORE_PIN27_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RX_BCLK
            break;
        case I2S_RX_PIN_PATTERN_4:
            CORE_PIN27_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RX_BCLK
            CORE_PIN29_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RX_FS
            CORE_PIN13_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RXD0
            break;
        case I2S_RX_PIN_PATTERN_5:
            CORE_PIN27_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RX_BCLK
            CORE_PIN29_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RX_FS
            CORE_PIN13_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RXD0
            CORE_PIN28_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_MCLK
            break;
        case I2S_RX_PIN_PATTERN_6:
            CORE_PIN27_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RX_BCLK
            CORE_PIN29_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RX_FS
            CORE_PIN13_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RXD0
            CORE_PIN11_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_MCLK
            break;
        default:
            break;
    }
}


void I2S_class::clock_init()
{
    // Disable system clock to the I2S module
    //SIM_SCGC6 &= ~(SIM_SCGC6_I2S);
    SIM_SCGC6 |= SIM_SCGC6_I2S;

    if(clock==I2S_CLOCK_EXTERNAL)
    {
        // Select input clock 0
        // Configure to input the bit-clock from pin, bypasses the MCLK divider
        I2S0_MCR = I2S_MCR_MICS(0);
        I2S0_MDR = 0;
    }
    else
    {
        // Select input clock 0 and output enable
        I2S0_MCR = I2S_MCR_MICS(0) | I2S_MCR_MOE;
        
        // 8k, 12k, 16k, 32k, etc all clock the I2S module at 12.288 MHz
        // 11025Hz, 22050, 44100 clock the I2S module at 11.2896 MHz
        switch( clock )
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



void I2S_class::init()
{
    io_init();
    clock_init();
 
    if( receive )
        i2s_receive_init();
    else
        i2s_transmit_init();
        
    if( useDMA )
    {
        dma_buffer_init();
        if( receive )
            dma_receive_init();
        else
            dma_transmit_init();
    }

}

// Configures the number of words in each frame. The value written should be one less than the number of
// words in the frame (for example, write 0 for one word per frame). The maximum supported frame size is
// 16 words.
#define FRSZ (I2S_FRAME_SIZE-1)

// Configures the length of the frame sync in number of bit clocks. The value written must be one less than
// the number of bit clocks. For example, write 0 for the frame sync to assert for one bit clock only. The sync
// width cannot be configured longer than the first word of the frame.
#define SYWD (I2S_IO_BIT_DEPTH-1)


void I2S_class::i2s_transmit_init()
{
    // transmit disable while we configure everything
    I2S0_TCSR &= ~(I2S_TCSR_TE);
    
    // Transmitter remains enabled until (and TE set) the end of the current frame
    for( int i=0; i<1000 && (I2S0_TCSR & I2S_TCSR_TE); i++ );
    if( I2S0_TCSR & I2S_TCSR_TE )
        return;

    I2S0_TMR = 0;                           // No word mask
    // --------------------------------------------------------------------------------
    I2S0_TCR1  = I2S_TCR1_TFW(FRSZ);        // set FIFO watermark
    // --------------------------------------------------------------------------------
    I2S0_TCR2  = I2S_TCR2_SYNC(0);          // use asynchronous mode
    I2S0_TCR2 |= I2S_TCR2_BCP;              // BCLK polarity: active low
    if( clock != I2S_CLOCK_EXTERNAL )
    {
        I2S0_TCR2 |= I2S_TCR2_MSEL(1);          // use mc1 (notbus clock as BCLK source
        I2S0_TCR2 |= I2S_TCR2_DIV(3);           // divide internal master clock to generate bit clock
        I2S0_TCR2 |= I2S_TCR2_BCD;              // BCLK is generated internally (master mode)
    }
    // --------------------------------------------------------------------------------    
    I2S0_TCR3  = I2S_TCR3_TCE;              // transmit data channel is enabled
    // --------------------------------------------------------------------------------
    I2S0_TCR4  = I2S_TCR4_FRSZ(FRSZ);       // frame size in words (plus one)
    I2S0_TCR4 |= I2S_TCR4_SYWD(SYWD);       // number of bits in frame sync (plus one)
    I2S0_TCR4 |= I2S_TCR4_MF;               // MSB (most significant bit) first
    I2S0_TCR4 |= I2S_TCR4_FSE;              // Frame sync one bit before the frame
    if( clock != I2S_CLOCK_EXTERNAL )
    {
        I2S0_TCR4 |= I2S_TCR4_FSD;              // WCLK is generated internally (master mode)
    }
    // --------------------------------------------------------------------------------
    I2S0_TCR5  = I2S_TCR5_W0W(SYWD);        // bits per word, first frame
    I2S0_TCR5 |= I2S_TCR5_WNW(SYWD);        // bits per word, nth frame
    I2S0_TCR5 |= I2S_TCR5_FBT(0x0f);        // index shifted for FIFO (TODO depend on I2S_BUFFER_BIT_DEPTH)
}

void I2S_class::i2s_receive_init()
{
    // receive disable while we configure everything
    I2S0_RCSR &= ~(I2S_RCSR_RE);

    // Receiver remains enabled until (and TE set) the end of the current frame
    for( int i=0; i<1000 && (I2S0_RCSR & I2S_RCSR_RE); i++ );
    if( I2S0_RCSR & I2S_RCSR_RE )
        return;

    I2S0_RMR = 0;                           // No word mask
    // --------------------------------------------------------------------------------
    I2S0_RCR1  = I2S_RCR1_RFW(FRSZ);        // set FIFO watermark
    // --------------------------------------------------------------------------------
    I2S0_RCR2  = I2S_RCR2_SYNC(1);          // synchronous with the transmitter
    I2S0_RCR2 |= I2S_RCR2_BCP;              // BCLK polarity: active low
    if( clock != I2S_CLOCK_EXTERNAL )
    {
        I2S0_RCR2 |= I2S_RCR2_MSEL(0);          // use MCLK as BCLK source
        I2S0_RCR2 |= I2S_RCR2_DIV(7);           // (DIV + 1) * 2, 12.288 MHz / 4 = 3.072 MHz
        I2S0_RCR2 |= I2S_RCR2_BCD;              // BCLK is generated internally in Master mode
    }
    // --------------------------------------------------------------------------------
    I2S0_RCR3  = I2S_RCR3_RCE;              // receive data channel is enabled
    // --------------------------------------------------------------------------------
    I2S0_RCR4  = I2S_RCR4_FRSZ(FRSZ);       // frame size in words (plus one)
    I2S0_RCR4 |= I2S_RCR4_SYWD(SYWD);       // bit width of a word (plus one)
    I2S0_RCR4 |= I2S_RCR4_MF;               // MSB (most significant bit) first
    I2S0_RCR4 |= I2S_RCR4_FSE;              // Frame sync one bit before the frame
    if( clock != I2S_CLOCK_EXTERNAL )
    {
        I2S0_RCR4 |= I2S_RCR4_FSD;              // WCLK is generated internally (master mode)
    }
    // --------------------------------------------------------------------------------
    I2S0_RCR5  = I2S_RCR5_W0W(SYWD);        // bits per word, first frame
    I2S0_RCR5 |= I2S_RCR5_WNW(SYWD);        // bits per word, nth frame
    I2S0_RCR5 |= I2S_RCR5_FBT(0x0f);        // index shifted for FIFO (TODO depend on I2S_BUFFER_BIT_DEPTH)
}


/* I2S class-instance callbacks */

void I2S_class::i2s_tx_callback(void)
{
    if(!(I2S0_TCSR & I2S_TCSR_FRF))  return;
    
    // Call your function to get the data into our buffer
    fnI2SCallback( _i2s_Tx_Buffer );

    // Copy the data from our buffer into FIFO
    if( I2S_FRAME_SIZE>0 ) I2S0_TDR0 = (uint32_t)(_i2s_Tx_Buffer[0]);
    if( I2S_FRAME_SIZE>1 ) I2S0_TDR0 = (uint32_t)(_i2s_Tx_Buffer[1]);
    if( I2S_FRAME_SIZE>2 ) I2S0_TDR0 = (uint32_t)(_i2s_Tx_Buffer[2]);
    if( I2S_FRAME_SIZE>3 ) I2S0_TDR0 = (uint32_t)(_i2s_Tx_Buffer[3]);
    
    //for( uint8_t i=0; i<I2S_FRAME_SIZE-1; i++ )
    //{
        //I2S0_TDR0 = (uint32_t)(_I2S_SAMPLE_T)(_i2s_Tx_Buffer[i]);
    //}
    
    if(I2S0_TCSR & I2S_TCSR_FEF)  I2S0_TCSR |= I2S_TCSR_FEF; // clear if underrun
    if(I2S0_TCSR & I2S_TCSR_SEF)  I2S0_TCSR |= I2S_TCSR_SEF; // clear if frame sync error
}

void I2S_class::i2s_rx_callback(void)
{  
    // Copy the data from FIFO into our buffer
    if( I2S_FRAME_SIZE>0 ) _i2s_Rx_Buffer[0] = (_I2S_SAMPLE_T)I2S0_RDR0;
    if( I2S_FRAME_SIZE>1 ) _i2s_Rx_Buffer[1] = (_I2S_SAMPLE_T)I2S0_RDR0;
    if( I2S_FRAME_SIZE>2 ) _i2s_Rx_Buffer[2] = (_I2S_SAMPLE_T)I2S0_RDR0;
    if( I2S_FRAME_SIZE>3 ) _i2s_Rx_Buffer[3] = (_I2S_SAMPLE_T)I2S0_RDR0;

    // Call your function to handle the data
    fnI2SCallback( _i2s_Rx_Buffer );
    
   if(I2S0_RCSR & I2S_RCSR_FEF)  I2S0_RCSR |= I2S_RCSR_FEF; // clear if underrun
   if(I2S0_RCSR & I2S_RCSR_SEF)  I2S0_RCSR |= I2S_RCSR_SEF; // clear if frame sync error
}


/* I2S ISR (used when you're not using DMA) */

void i2s0_tx_isr(void)
{
    I2STx0.i2s_tx_callback();
}

void i2s0_rx_isr(void)
{
    I2SRx0.i2s_rx_callback();
}



/*
 * DMA
 * DMA channel 0 is used for transmit, and channel 1 for receive.
 */

void I2S_class::dma_buffer_init(void)
{
    if(receive)
    {
        memset( _dma_Rx_Buffer_A, 0, DMA_BUFFER_SIZE * sizeof(_I2S_SAMPLE_T) );
        memset( _dma_Rx_Buffer_B, 0, DMA_BUFFER_SIZE * sizeof(_I2S_SAMPLE_T) );
    }
    else
    {
        memset( _dma_Tx_Buffer_A, 0, DMA_BUFFER_SIZE * sizeof(_I2S_SAMPLE_T) );
        memset( _dma_Tx_Buffer_B, 0, DMA_BUFFER_SIZE * sizeof(_I2S_SAMPLE_T) );
    }
    _dma_using_Buffer_A = 1;
}



void I2S_class::dma_transmit_init(void)  
{
    // Enable clock to the DMAMUX module
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX;
    // And clock to the DMA module
    SIM_SCGC7 |= SIM_SCGC7_DMA;
    
    // configure DMA_MUX
    // DMAMUX0_CHCFG0 = 0;
    DMAMUX0_CHCFG0 = DMAMUX_SOURCE_I2S0_TX;

    // Enable IRQ on the DMA channel 0
    NVIC_ENABLE_IRQ(IRQ_DMA_CH0);

    // Set inactive
    DMA_TCD0_CSR &= ~(DMA_CSR_ACTIVE);
    
#ifndef ROUNDROBIN
    // Set channel priorities (each must be unique)
    DMA_DCHPRI3 = 0;
    DMA_DCHPRI2 = 1;
    DMA_DCHPRI1 = 2;
    DMA_DCHPRI0 = 3;  // cannot be pre-empted, can pre-empt, highest priority
#endif

    // Control register
    DMA_CR = 0              // Normal
           | DMA_CR_EMLM    // Enable minor looping
#ifdef ROUNDROBIN
           | DMA_CR_ERCA    // Enable round-robin channel arbitration
#endif
           ;
   
    // fill the TCD regs
    DMA_TCD0_SADDR          = (const volatile void *) _dma_Tx_Buffer_A ;            // alternated with _dma_Buffer_B by our interrupt handler
    DMA_TCD0_SOFF           = 2;                                    // 2 byte source offset after each transfer
    DMA_TCD0_ATTR           = DMA_ATTR_SMOD(0)                      // No source modulo
                            | DMA_ATTR_SSIZE(DMA_ATTR_SIZE_16BIT)   // Source data 16 bit
                            | DMA_ATTR_DMOD(2)                      // Destination modulo 2
                            | DMA_ATTR_DSIZE(DMA_ATTR_SIZE_16BIT);  // Destination 16 bit
    DMA_TCD0_NBYTES_MLNO    = 2;                                    // Transfer two bytes in each service request
    DMA_TCD0_SLAST          = 0;//-(DMA_BUFFER_SIZE*2);             // Source address will always be newly written before each new start
    DMA_TCD0_DADDR          = (volatile void *) &I2S0_TDR0;        // Destination is the I2S data register
    DMA_TCD0_DOFF           = 0;                                    // No destination offset after each transfer
    DMA_TCD0_DLASTSGA       = 0;                                    // No scatter/gather
    DMA_TCD0_CITER_ELINKNO  = DMA_BUFFER_SIZE & DMA_CITER_MASK;     // Major loop iteration count = total samples (128)
    DMA_TCD0_BITER_ELINKNO  = DMA_BUFFER_SIZE & DMA_BITER_MASK;     // Major loop iteration count = total samples (128), no channel links
    DMA_TCD0_CSR            = DMA_CSR_INTMAJOR                      // Interrupt on major loop completion
                            | DMA_CSR_BWC(3);                       // DMA bandwidth control

    // enable DMA channel 0 requests
    // DMA_ERQ = DMA_ERQ_ERQ0;
    DMA_SERQ = DMA_SERQ_SERQ(0);

    // enable DMAMUX
    DMAMUX0_CHCFG0 |= DMAMUX_ENABLE /* | DMAMUX_TRIG */;

    // Set active
    DMA_TCD0_CSR |= DMA_CSR_ACTIVE;

    // To initiate from software, set DMA_CSR[start]
    //DMA_TCD0_CSR |= DMA_CSR_START;
}

void I2S_class::dma_receive_init(void)  
{
    // Enable clock to the DMAMUX module
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX;
    // And clock to the DMA module
    SIM_SCGC7 |= SIM_SCGC7_DMA;
    
    // configure DMA_MUX
    // DMAMUX0_CHCFG1 = 0;
    DMAMUX0_CHCFG1 = DMAMUX_SOURCE_I2S0_RX;

    // Enable IRQ on the DMA channel 1
    NVIC_ENABLE_IRQ(IRQ_DMA_CH1);

    // Set inactive
    DMA_TCD1_CSR &= ~(DMA_CSR_ACTIVE);
    
#ifndef ROUNDROBIN
    // Set channel priorities (each must be unique)
    DMA_DCHPRI3 = 0;
    DMA_DCHPRI2 = 1;
    DMA_DCHPRI1 = 2;
    DMA_DCHPRI0 = 3;  // cannot be pre-empted, can pre-empt, highest priority
#endif

    // Control register
    DMA_CR = 0              // Normal
           | DMA_CR_EMLM    // Enable minor looping
#ifdef ROUNDROBIN
           | DMA_CR_ERCA    // Enable round-robin channel arbitration
#endif
           ;
   
    // fill the TCD regs
    DMA_TCD1_SADDR          = (const volatile void *) &I2S0_RDR0;  // Source is the I2S data register
    DMA_TCD1_SOFF           = 0;                                    // No source offset after each transfer
    DMA_TCD1_ATTR           = DMA_ATTR_SMOD(2)                      // No source modulo
                            | DMA_ATTR_SSIZE(DMA_ATTR_SIZE_16BIT)   // Source data 16 bit
                            | DMA_ATTR_DMOD(0)                      // No destination modulo
                            | DMA_ATTR_DSIZE(DMA_ATTR_SIZE_16BIT);  // Destination 16 bit
    DMA_TCD1_NBYTES_MLNO    = 2;                                    // Transfer two bytes in each service request
    DMA_TCD1_SLAST          = 0;//-(DMA_BUFFER_SIZE*2);             // Source address will always be newly written before each new start
    DMA_TCD1_DADDR          = (volatile void *) _dma_Rx_Buffer_A ;  // Alternated with _dma_Buffer_B by our interrupt handler
    DMA_TCD1_DOFF           = 2;                                    // 2 bytes destination offset after each transfer
    DMA_TCD1_DLASTSGA       = 0;                                    // No scatter/gather
    DMA_TCD1_CITER_ELINKNO  = DMA_BUFFER_SIZE & DMA_CITER_MASK;     // Major loop iteration count = total samples (128)
    DMA_TCD1_BITER_ELINKNO  = DMA_BUFFER_SIZE & DMA_BITER_MASK;     // Major loop iteration count = total samples (128), no channel links
    DMA_TCD1_CSR            = DMA_CSR_INTMAJOR                      // Interrupt on major loop completion
                            | DMA_CSR_BWC(3);                       // DMA bandwidth control

    // enable DMA channel 1 requests
    // DMA_ERQ = DMA_ERQ_ERQ1;
    DMA_SERQ = DMA_SERQ_SERQ(1);

    // enable DMAMUX
    DMAMUX0_CHCFG1 |= DMAMUX_ENABLE /* | DMAMUX_TRIG */;

    // Set active
    DMA_TCD1_CSR |= DMA_CSR_ACTIVE;

    // To initiate from software, set DMA_CSR[start]
    //DMA_TCD1_CSR |= DMA_CSR_START;
}



void I2S_class::dma_start(void) 
{
    // Enable the appropriate channel
    DMA_SERQ = DMA_SERQ_SERQ( receive );
}

void I2S_class::dma_stop(void) 
{
    // Clear Enable-flag of the appropriate channel
    DMA_CERQ = DMA_CERQ_CERQ( receive );
}


/* DMA class-instance callbacks */

void I2S_class::dma_tx_callback(void)
{
    _I2S_SAMPLE_T *dmaBuf;
    _I2S_SAMPLE_T *yourBuf;
    if (_dma_using_Buffer_A)
    {
        _dma_using_Buffer_A = 0;
        dmaBuf = _dma_Tx_Buffer_B;
        yourBuf = _dma_Tx_Buffer_A;
    }
    else
    {
        _dma_using_Buffer_A = 1;
        dmaBuf = _dma_Tx_Buffer_A;
        yourBuf = _dma_Tx_Buffer_B;
    }
    // DMA will play from one buffer
    DMA_TCD0_SADDR = (const volatile void *)dmaBuf;
    // while you fill the other
    fnDMACallback( yourBuf, DMA_BUFFER_SIZE );
}

void I2S_class::dma_rx_callback(void)
{
    _I2S_SAMPLE_T *dmaBuf;
    _I2S_SAMPLE_T *yourBuf;
    if (_dma_using_Buffer_A)
    {
        _dma_using_Buffer_A = 0;
        dmaBuf = _dma_Rx_Buffer_B;
        yourBuf = _dma_Rx_Buffer_A;
    }
    else
    {
        _dma_using_Buffer_A = 1;
        dmaBuf = _dma_Rx_Buffer_A;
        yourBuf = _dma_Rx_Buffer_B;
    }
    // DMA will read into one buffer
    DMA_TCD1_DADDR = (volatile void *)dmaBuf;
    // while you read the other
    fnDMACallback( yourBuf, DMA_BUFFER_SIZE );
}


/* DMA ISR */
 
void dma_ch0_isr(void)                  // DMA channel 0 for Tx
{
    I2STx0.dma_tx_callback();
    DMA_CINT = DMA_CINT_CINT(0);        // Clear the interrupt
}

void dma_ch1_isr(void)                  // DMA channel 1 for Rx
{
    I2SRx0.dma_rx_callback();
    DMA_CINT = DMA_CINT_CINT(1);        // Clear the interrupt
}
