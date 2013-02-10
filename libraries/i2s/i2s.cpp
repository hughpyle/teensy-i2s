/*
 * I2S interface for teensy 3.0
 * (cc) https://creativecommons.org/licenses/by/3.0/ by Hugh Pyle and other contributors
 */
 
#include <i2s.h>
#include <mk20dx128.h>
#include "core_pins.h"

// There's one instance of the class for Tx, another for Rx
I2S_class I2STx0(0);
I2S_class I2SRx0(1);

// Buffers for 16 bit audio samples.
// Static (not class members) because we want to put them in the DMAMEM area.
static int16_t DMAMEM _dma_Rx_Buffer_A[DMA_BUFFER_SIZE];
static int16_t DMAMEM _dma_Rx_Buffer_B[DMA_BUFFER_SIZE];
static int16_t DMAMEM _dma_Tx_Buffer_A[DMA_BUFFER_SIZE];
static int16_t DMAMEM _dma_Tx_Buffer_B[DMA_BUFFER_SIZE];

// Use round-robin DMA channel priorities?  If not, they're explicitly set
#define ROUNDROBIN


I2S_class::I2S_class(bool isRx)
{
    receive = isRx;
}

void I2S_class::begin(unsigned char clk, unsigned char pinpattern)
{
    clockType = clk;
    pinPattern = pinpattern;
    useDMA = false;
    init();
}

void I2S_class::begin(unsigned char clk, unsigned char pinpattern, void (*fptr)( int16_t *pBuf, int16_t len ))
{
    clockType = clk;
    pinPattern = pinpattern;
    useDMA = true;
    fnDMACallback = fptr;
    init();
}


void I2S_class::start()
{
    if( useDMA )
    {
        if( receive )
        {
            // Receive enable
            I2S0_RCSR |= I2S_RCSR_RE            // Receive Enable
                       | I2S_RCSR_BCE           // Bit Clock Enable
                       ;
        }
        else
        {
            // Transmit enable.  When FIFO needs data it generates a DMA request.
            I2S0_TCSR |= I2S_TCSR_TE            // Transmit Enable
                       | I2S_TCSR_BCE           // Bit Clock Enable
                       | I2S_TCSR_FRDE          // FIFO Request DMA Enable
                       ;
        }
    }
    else
    {
        if( receive )
        {
            // Receive enable
            NVIC_ENABLE_IRQ(IRQ_I2S0_RX);
            I2S0_RCSR |= I2S_RCSR_RE            // Receive Enable
                       | I2S_RCSR_BCE           // Bit Clock Enable
                       ;
        }
        else
        {
            // Transmit enable.  When FIFO needs data it generates an interrupt.
            NVIC_ENABLE_IRQ(IRQ_I2S0_TX);
            I2S0_TCSR |= I2S_TCSR_TE            // Transmit Enable
                       | I2S_TCSR_BCE           // Bit Clock Enable
                       | I2S_TCSR_FRIE          // FIFO Request Interrupt Enable
                       ;
        }
    }
}

void I2S_class::stop()
{
    if( useDMA )
    {
        /* TODO */
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
    switch( pinPattern & 0x0F )
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
    switch( pinPattern & 0xF0 )
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


void I2S_class::clock_init(unsigned char clk)
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



void I2S_class::init()
{
    io_init();
    clock_init( clockType );
 
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


void I2S_class::i2s_transmit_init()
{
    // transmit disable while we configure everything
    I2S0_TCSR &= ~(I2S_TCSR_TE);
    I2S0_TCR3 = 0;
    
    // Transmitter remains enabled until (and TE set) the end of the current frame
    for( int i=0; i<1000 && (I2S0_TCSR & I2S_TCSR_TE); i++ );
    if( I2S0_TCSR & I2S_TCSR_TE )
        return;

    // transmit configuration register 2
    if(clockType==I2S_CLOCK_EXTERNAL)
    {
        I2S0_TCR2 =
              I2S_TCR2_SYNC(0)   // asynchronous mode
            | I2S_TCR2_BCP          // bit clock polarity: active low
        ;
    }
    else
    {
        I2S0_TCR2 =
              I2S_TCR2_SYNC(0)      // asynchronous mode
            | I2S_TCR2_DIV(7)       // divide internal master clock to generate bit clock
            | I2S_TCR2_BCD          // bit clock direction: generated internally in Master mode
            | I2S_TCR2_MSEL(0)      // bus clock as source
            | I2S_TCR2_BCP          // bit clock polarity: active low
        ;
    }

    // transmit configuration register 3
    I2S0_TMR = 0;
    I2S0_TCR3 =
        I2S_TCR3_TCE;       // transmit data channel is enabled
                            // First word sets the word flag
        
    // Always describe the frames as 32-bit even when the codec is only using 16 significant bits of data
    if(clockType==I2S_CLOCK_EXTERNAL)
    {
        I2S0_TCR4 =
            I2S_TCR4_FRSZ(1)    // frame size 2 words (plus one)
          | I2S_TCR4_SYWD(31)   // length of frame sync in bit clocks (plus one)
          | I2S_TCR4_MF         // MSB first
          | I2S_TCR4_FSE        // Frame sync one bit before the frame
          ;
    }
    else
    {
        I2S0_TCR4 =
            I2S_TCR4_FRSZ(1)    // frame size 2 words (1 plus one)
          | I2S_TCR4_SYWD(31)   // length of frame sync in bit clocks (plus one) => word clock with 50/50 duty cycle
          | I2S_TCR4_MF         // MSB first
          | I2S_TCR4_FSD        // Frame sync is generated internally (master)
          | I2S_TCR4_FSE        // Frame sync one bit before the frame
          ;
    }
    
    I2S0_TCR5 =
        I2S_TCR5_WNW(31)        // 32 bits per word except for first frame
      | I2S_TCR5_W0W(31)        // 32 bits per word there too
      | I2S_TCR5_FBT(0x0f)      // first Bit Shifted = 31, so data is aligned to the MSB of the FIFO register
      ;

}

void I2S_class::i2s_receive_init()
{
    // receive disable while we configure everything
    I2S0_RCSR &= ~(I2S_RCSR_RE);
    
    // From loglow  (master mode)
    uint8_t nChans = I2S_AUDIO_NUM_CHANS - 1;
    uint8_t nBits  = I2S_AUDIO_BIT_DEPTH - 1;

    SIM_SCGC6 |= SIM_SCGC6_I2S;          // enable clock to the I2S module
    I2S0_MDR  |= I2S_MDR_FRACT(15);      // output = input * (FRACT + 1) / (DIVIDE + 1)
    I2S0_MDR  |= I2S_MDR_DIVIDE(124);    // 12.288 MHz = 96 MHz * (16 / 125)
    I2S0_MCR  |= I2S_MCR_MOE;            // enable MCLK pin as output
    // --------------------------------------------------------------------------------
    I2S0_RCR1 |= I2S_RCR1_RFW(nChans);   // set FIFO watermark
    // --------------------------------------------------------------------------------
    I2S0_RCR2 |= I2S_RCR2_MSEL(1);       // use MCLK as BCLK source
    I2S0_RCR2 |= I2S_RCR2_SYNC(0);       // use asynchronous mode
    I2S0_RCR2 |= I2S_RCR2_DIV(1);        // (DIV + 1) * 2, 12.288 MHz / 4 = 3.072 MHz
    I2S0_RCR2 |= I2S_RCR2_BCD;           // generate BCLK, master mode
    I2S0_RCR2 |= I2S_TCR2_BCP;           // BCLK is active low
    // --------------------------------------------------------------------------------
    I2S0_RCR3 |= I2S_RCR3_RCE;           // enable receive channel
    // --------------------------------------------------------------------------------
    I2S0_RCR4 |= I2S_RCR4_FRSZ(nChans);  // frame size in words
    I2S0_RCR4 |= I2S_RCR4_SYWD(nBits);   // bit width of WCLK
    I2S0_RCR4 |= I2S_RCR4_MF;            // MSB (most significant bit) first
    I2S0_RCR4 |= I2S_RCR4_FSD;           // generate WCLK, master mode
    I2S0_RCR4 |= I2S_RCR4_FSE;           // extra bit before frame starts
    // --------------------------------------------------------------------------------
    I2S0_RCR5 |= I2S_RCR5_W0W(nBits);    // bits per word, first frame
    I2S0_RCR5 |= I2S_RCR5_WNW(nBits);    // bits per word, nth frame
    I2S0_RCR5 |= I2S_RCR5_FBT(nBits);    // index shifted for FIFO
    // --------------------------------------------------------------------------------
    I2S0_RCSR |= I2S_RCSR_BCE;           // enable the BCLK output
    I2S0_RCSR |= I2S_RCSR_RE;            // enable receive globally
    I2S0_RCSR |= I2S_RCSR_FRIE;          // enable FIFO request interrupt
}



/*
 * DMA
 */
 


void I2S_class::dma_buffer_init(void)
{
    if(receive)
    {
    memset( _dma_Rx_Buffer_A, 0, DMA_BUFFER_SIZE * sizeof(int16_t) );
    memset( _dma_Rx_Buffer_B, 0, DMA_BUFFER_SIZE * sizeof(int16_t) );
    }
    else
    {
    memset( _dma_Tx_Buffer_A, 0, DMA_BUFFER_SIZE * sizeof(int16_t) );
    memset( _dma_Tx_Buffer_B, 0, DMA_BUFFER_SIZE * sizeof(int16_t) );
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
    DMAMUX0_CHCFG0 = 0;
    DMAMUX0_CHCFG0 = DMAMUX_SOURCE_I2S0_TX;

    // Enable IRQ on the DMA channel 0
    // NVIC_ENABLE_IRQ(IRQ_DMA_ERROR);
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
//      | DMA_CR_EDBG_MASK  // Stall DMA transfers when debugger is halted (avoid noise)
      | DMA_CR_EMLM  // Enable minor looping
#ifdef ROUNDROBIN
        | DMA_CR_ERCA
#endif
        ;
   
    // fill the TCD regs
    DMA_TCD0_SADDR          = (const volatile void *) _dma_Tx_Buffer_A ;            // alternated with _dma_Buffer_B by our interrupt handler
    DMA_TCD0_SOFF           = 2;                                    // 2 byte offset 
    DMA_TCD0_ATTR           = DMA_ATTR_SMOD(0)                      // No source modulo
                            | DMA_ATTR_SSIZE(DMA_ATTR_SIZE_16BIT)   // Source data 16 bit
                            | DMA_ATTR_DMOD(0)                      // No destination modulo
                            | DMA_ATTR_DSIZE(DMA_ATTR_SIZE_16BIT);  // Destination 16 bit
    DMA_TCD0_NBYTES_MLNO    = 2;                                    // Transfer two bytes in each service request
    DMA_TCD0_SLAST          = 0;//-(DMA_BUFFER_SIZE*2);             // source address will always be newly written before each new start
    DMA_TCD0_DADDR          = (volatile void *) &I2S0_TDR0;                // Destination is the I2S data register
    DMA_TCD0_DOFF           = 0;                                    // No destination offset after each write
    DMA_TCD0_DLASTSGA       = 0;                                    // No scatter/gather
    DMA_TCD0_CITER_ELINKNO  = DMA_BUFFER_SIZE & DMA_CITER_MASK;     // major loop iteration count = total samples (128)
    DMA_TCD0_BITER_ELINKNO  = DMA_BUFFER_SIZE & DMA_BITER_MASK;     // major loop iteration count = total samples (128), no channel links
    DMA_TCD0_CSR            = DMA_CSR_INTMAJOR                      // interrupt on major loop completion
                            | DMA_CSR_BWC(3);                       // DMA bandwidth control

    // enable DMA channel 0 requests
    DMA_ERQ = DMA_ERQ_ERQ0;
    DMA_SERQ = DMA_SERQ_SERQ(0);

    // enable DMAMIX
    DMAMUX0_CHCFG0 |= DMAMUX_ENABLE /* | DMAMUX_TRIG */;

    // Set active
    DMA_TCD0_CSR |= DMA_CSR_ACTIVE;

    // To initiate from software, set DMA_CSR[start]
    //DMA_TCD0_CSR |= DMA_CSR_START;
}

void I2S_class::dma_receive_init(void)  
{
}



void I2S_class::dma_start(void) 
{
    DMA_SERQ = DMA_SERQ_SERQ(0);
}

void I2S_class::dma_stop(void) 
{
    DMA_CERQ = DMA_CERQ_CERQ(0);
}


void I2S_class::dma_tx_callback(void)
{
    int16_t *dmaBuf;
    int16_t *yourBuf;
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
    // Play from one buffer
    DMA_TCD0_SADDR = (const volatile void *)dmaBuf;
    // while you fill the other
    fnDMACallback( yourBuf, DMA_BUFFER_SIZE );
}

void I2S_class::dma_rx_callback(void)
{
    int16_t *dmaBuf;
    int16_t *yourBuf;
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
    // Read into one buffer
    DMA_TCD1_SADDR = (const volatile void *)dmaBuf;
    // while you read the other
    fnDMACallback( yourBuf, DMA_BUFFER_SIZE );
}



/* DMA ISR */
 
// DMA channel 0 for Tx
void dma_ch0_isr(void)
{
    I2STx0.dma_tx_callback();
    DMA_CINT = DMA_CINT_CINT(0);                 // Clear the interrupt
}

// DMA channel 1 for Rx
void dma_ch1_isr(void)
{
    I2SRx0.dma_rx_callback();
    DMA_CINT = DMA_CINT_CINT(1);                 // Clear the interrupt
}
