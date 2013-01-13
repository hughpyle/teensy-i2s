
#ifndef I2S_H
#define I2S_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------ Hardware ----------------------------- */
/* ------ Move this stuff to mk20dx128.h ------- */

// DMA
#define INT_DMA0                         16

/* CR - DMA Control Register */
#define DMA_CR_CX                       ((uint32_t)(1<<17))     // Cancel Transfer
#define DMA_CR_ECX                      ((uint32_t)(1<<16))     // Error Cancel Transfer
#define DMA_CR_EMLM                     ((uint32_t)0x80)        // Enable Minor Loop Mapping
#define DMA_CR_CLM                      ((uint32_t)0x40)        // Continuous Link Mode
#define DMA_CR_HALT                     ((uint32_t)0x20)        // Halt DMA Operations
#define DMA_CR_HOE                      ((uint32_t)0x10)        // Halt On Error
#define DMA_CR_ERCA                     ((uint32_t)0x04)        // Enable Round Robin Channel Arbitration
#define DMA_CR_EDBG                     ((uint32_t)0x02)        // Enable Debug

/* ES - Error Status */

/* ERQ - Enable Request Register */
#define DMA_ERQ_ERQ0                    ((uint32_t)0x1)         // Enable DMA Request 0
#define DMA_ERQ_ERQ1                    ((uint32_t)0x2)         // Enable DMA Request 1
#define DMA_ERQ_ERQ2                    ((uint32_t)0x4)         // Enable DMA Request 2
#define DMA_ERQ_ERQ3                    ((uint32_t)0x8)         // Enable DMA Request 3

/* EEI - Enable Error Interrupt Register */

/* CEEI - Clear Enable Error Interrupt Register */

/* SEEI - Set Enable Error Interrupt Register */

/* CERQ - Clear Enable Request Register */

/* SERQ - Set Enable Request Register */

/* CDNE - Clear DONE Status Bit Register */

/* SSRT - Set START Bit Register */

/* CERR - Clear Error Register */

/* CINT - Clear Interrupt Request Register */
#define DMA_CINT_NOP                    (uint8_t)0x80      // No Operation (ignore the other bits in this register)
#define DMA_CINT_CINT(n)                (uint8_t)0x40      // Clear Interrupt Request
#define DMA_CINT_CINT(n)                (uint8_t)(n & 3)   // Clear Interrupt Request

/* INT - Interrupt Request Register */

/* ERR - Error Register */

/* HRS - Hardware Request Status Register */

/* TCD CSR bits */
#define DMA_CSR_START                            ((uint16_t)0x1)
#define DMA_CSR_INTMAJOR                         ((uint16_t)0x2)
#define DMA_CSR_INTHALF                          ((uint16_t)0x4)
#define DMA_CSR_DREQ                             ((uint16_t)0x8)
#define DMA_CSR_ESG                              ((uint16_t)0x10)
#define DMA_CSR_MAJORELINK                       ((uint16_t)0x20)
#define DMA_CSR_ACTIVE                           ((uint16_t)0x40)
#define DMA_CSR_DONE                             ((uint16_t)0x80)
#define DMA_CSR_MAJORLINKCH(n)                   ((uint16_t)(n & 0x3)<<8)       // Link Channel Number
#define DMA_CSR_BWC(n)                           ((uint16_t)(n & 0x3)<<14)      // Bandwidth Control

/* TCD ATTR - Transfer Attributes */
#define DMA_ATTR_DSIZE(n)                        ((uint16_t)(n & 0x07))         // Destination Data Transfer Size
#define DMA_ATTR_DMOD(n)                         ((uint16_t)(n & 0x1f)<<3)      // Destination Address Modulo
#define DMA_ATTR_SSIZE(n)                        ((uint16_t)(n & 0x07)<<8)      // Source Data Transfer Size
#define DMA_ATTR_SMOD(n)                         ((uint16_t)(n & 0x1f)<<11)     // Source Address Modulo



/* DMAMUX Channel Configuration bits */
#define DMAMUX_CHCFG_ENBL                ((uint8_t)0x80)         // DMA Channel Enable
#define DMAMUX_CHCFG_TRIG                ((uint8_t)0x40)         // DMA Channel Trigger Enable
#define DMAMUX_CHCFG_SOURCE(n)           ((uint8_t)(n & 0x3f))      // DMA Channel Source



/* I2S */

/* TCSR bits */
#define I2S_TCSR_TE           (uint32_t)0x80000000    // Transmitter Enable
#define I2S_TCSR_STOPE        (uint32_t)0x40000000    // Transmitter Enable in Stop mode
#define I2S_TCSR_DBGE         (uint32_t)0x20000000    // Transmitter Enable in Debug mode
#define I2S_TCSR_BCE          (uint32_t)0x10000000    // Bit Clock Enable
#define I2S_TCSR_FR           (uint32_t)0x02000000    // FIFO Reset
#define I2S_TCSR_SR           (uint32_t)0x01000000    // Software Reset
#define I2S_TCSR_WSF          (uint32_t)0x00100000    // Word Start Flag
#define I2S_TCSR_SEF          (uint32_t)0x00080000    // Sync Error Flag
#define I2S_TCSR_FEF          (uint32_t)0x00040000    // FIFO Error Flag (underrun)
#define I2S_TCSR_FWF          (uint32_t)0x00020000    // FIFO Warning Flag (empty)
#define I2S_TCSR_FRF          (uint32_t)0x00010000    // FIFO Request Flag
#define I2S_TCSR_WSIE         (uint32_t)0x00001000    // Word Start Interrupt Enable
#define I2S_TCSR_SEIE         (uint32_t)0x00000800    // Sync Error Interrupt Enable
#define I2S_TCSR_FEIE         (uint32_t)0x00000400    // FIFO Error Interrupt Enable
#define I2S_TCSR_FWIE         (uint32_t)0x00000200    // FIFO Warning Interrupt Enable
#define I2S_TCSR_FRIE         (uint32_t)0x00000100    // FIFO Request Interrupt Enable
#define I2S_TCSR_FWDE         (uint32_t)0x00000001    // FIFO Warning DMA Enable
#define I2S_TCSR_FRDE         (uint32_t)0x00000000    // FIFO Request DMA Enable

#define I2S_TCR1_TFW(n)       (uint32_t)(n & 0x03)   // Transmit FIFO watermark


/* TCR2 bits */
#define I2S_TCR2_DIV(n)       (uint32_t)((n)&0x0f)          // Bit clock divide by (DIV+1)*2
#define I2S_TCR2_BCD          (uint32_t)0x1000000           // Bit clock direction
#define I2S_TCR2_BCP          (uint32_t)0x2000000           // Bit clock polarity
#define I2S_TCR2_MSEL(n)      (uint32_t)((n & 3)<<26)       // MCLK select
#define I2S_TCR2_BCI          (uint32_t)0x10000000          // Bit clock input
#define I2S_TCR2_BCS          (uint32_t)0x20000000          // Bit clock swap
#define I2S_TCR2_SYNC(n)      (uint32_t)((n & 3)<<30)       // 0=async 1=sync with receiver

/* TCR3 bits */
#define I2S_TCR3_WDFL(n)      ((uint32_t)(n & 0x0f))        // word flag configuration
#define I2S_TCR3_TCE          (uint32_t)0x10000             // transmit channel enable

/* TCR4 bits */
#define I2S_TCR4_FSD          (uint32_t)0x1                 // Frame Sync Direction
#define I2S_TCR4_FSP          (uint32_t)0x2                 // Frame Sync Polarity
#define I2S_TCR4_FSE          (uint32_t)0x8                 // Frame Sync Early
#define I2S_TCR4_MF           (uint32_t)0x10                // MSB First
#define I2S_TCR4_SYWD(n)      (uint32_t)((n & 0x1f)<<8)     // Sync Width
#define I2S_TCR4_FRSZ(n)      (uint32_t)((n & 0x0f)<<16)    // Frame Size

/* TCR5 bits */
#define I2S_TCR5_FBT(n)       (uint32_t)((n & 0x1f)<<8)     // First Bit Shifted
#define I2S_TCR5_W0W(n)       (uint32_t)((n & 0x1f)<<16)    // Word 0 Width
#define I2S_TCR5_WNW(n)       (uint32_t)((n & 0x1f)<<24)    // Word N Width

/* TDR Bit Fields */
#define I2S_TDR_TDR(n)        (uint32_t)(n)

/* TFR Bit Fields */
#define I2S_TFR_RFP(n)        (uint32_t)(n & 5)             // read FIFO pointer
#define I2S_TFR_WFP(n)        (uint32_t)((n & 5)<<16)       // write FIFO pointer

/* TMR Bit Fields */
#define I2S_TMR_TWM(n)        (uint32_t)(n & 0xFFFFFFFF)

// I2S0_MCR bits
#define I2S_MCR_DUF           (uint32_t)(1<<31)             // Divider Update Flag
#define I2S_MCR_MOE           (uint32_t)(1<<30)             // MCLK Output Enable
#define I2S_MCR_MICS(n)       (uint32_t)((n & 3)<<24)       // MCLK Input Clock Select

// I2S0_MDR bits
#define I2S_MDR_FRACT(n)      (uint32_t)((n & 0xff)<<12)    // MCLK Fraction
#define I2S_MDR_DIVIDE(n)     (uint32_t)((n & 0xfff))       // MCLK Divide

/* ------ End Hardware ------------------------- */


// Config
#define I2S_WORD_LENGTH 16
#define I2S_SAMPLE_RATE 48000
#define DMA_BUFFER_SIZE 128

// Clock types for i2s_switch_clock
#define I2S_CLOCK_48K_INTERNAL 0
#define I2S_CLOCK_44K_INTERNAL 1
#define I2S_CLOCK_EXTERNAL     2

// Flags
extern volatile int Playing_Buff_A;

// Audio buffers with q15_t samples
extern int16_t Audio_Source_Blk_A[DMA_BUFFER_SIZE], Audio_Source_Blk_B[DMA_BUFFER_SIZE];


/*
 * I2S control methods
 */
 
void i2s_io_init(void);
void i2s_switch_clock(unsigned char clk);
int i2s_init(unsigned char clk);

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
