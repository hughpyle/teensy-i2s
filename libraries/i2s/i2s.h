
#ifndef I2S_H
#define I2S_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------ Hardware ----------------------------- */
/* ------ Move this stuff to mk20dx128.h ------- */

/* I2S */

/* TCSR bits */
#define I2S_TCSR_TE                     (uint32_t)0x80000000    // Transmitter Enable
#define I2S_TCSR_STOPE                  (uint32_t)0x40000000    // Transmitter Enable in Stop mode
#define I2S_TCSR_DBGE                   (uint32_t)0x20000000    // Transmitter Enable in Debug mode
#define I2S_TCSR_BCE                    (uint32_t)0x10000000    // Bit Clock Enable
#define I2S_TCSR_FR                     (uint32_t)0x02000000    // FIFO Reset
#define I2S_TCSR_SR                     (uint32_t)0x01000000    // Software Reset
#define I2S_TCSR_WSF                    (uint32_t)0x00100000    // Word Start Flag
#define I2S_TCSR_SEF                    (uint32_t)0x00080000    // Sync Error Flag
#define I2S_TCSR_FEF                    (uint32_t)0x00040000    // FIFO Error Flag (underrun)
#define I2S_TCSR_FWF                    (uint32_t)0x00020000    // FIFO Warning Flag (empty)
#define I2S_TCSR_FRF                    (uint32_t)0x00010000    // FIFO Request Flag (Data Ready)
#define I2S_TCSR_WSIE                   (uint32_t)0x00001000    // Word Start Interrupt Enable
#define I2S_TCSR_SEIE                   (uint32_t)0x00000800    // Sync Error Interrupt Enable
#define I2S_TCSR_FEIE                   (uint32_t)0x00000400    // FIFO Error Interrupt Enable
#define I2S_TCSR_FWIE                   (uint32_t)0x00000200    // FIFO Warning Interrupt Enable
#define I2S_TCSR_FRIE                   (uint32_t)0x00000100    // FIFO Request Interrupt Enable
#define I2S_TCSR_FWDE                   (uint32_t)0x00000001    // FIFO Warning DMA Enable
#define I2S_TCSR_FRDE                   (uint32_t)0x00000000    // FIFO Request DMA Enable

#define I2S_TCR1_TFW(n)                 (uint32_t)(n & 0x03)   // Transmit FIFO watermark


/* TCR2 bits */
#define I2S_TCR2_DIV(n)       ((uint32_t)n & 0xff)          // Bit clock divide by (DIV+1)*2
#define I2S_TCR2_BCD          ((uint32_t)0x1000000)         // Bit clock direction
#define I2S_TCR2_BCP          ((uint32_t)0x2000000)         // Bit clock polarity
#define I2S_TCR2_MSEL(n)      ((uint32_t)(n & 3)<<26)       // MCLK select, 0=bus clock, 1=I2S0_MCLK
#define I2S_TCR2_BCI          ((uint32_t)0x10000000)        // Bit clock input
#define I2S_TCR2_BCS          ((uint32_t)0x20000000)        // Bit clock swap
#define I2S_TCR2_SYNC(n)      ((uint32_t)(n & 3)<<30)       // 0=async 1=sync with receiver

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


/* DMA */

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
#define DMA_ERQ_ERQ0                    ((uint32_t)1<<0)         // Enable DMA Request 0
#define DMA_ERQ_ERQ1                    ((uint32_t)1<<1)         // Enable DMA Request 1
#define DMA_ERQ_ERQ2                    ((uint32_t)1<<2)         // Enable DMA Request 2
#define DMA_ERQ_ERQ3                    ((uint32_t)1<<3)         // Enable DMA Request 3

/* EEI - Enable Error Interrupt Register */
#define DMA_EEI_EEI0                    ((uint32_t)1<<0)        // Enable Error Interrupt 0
#define DMA_EEI_EEI1                    ((uint32_t)1<<1)        // Enable Error Interrupt 1
#define DMA_EEI_EEI2                    ((uint32_t)1<<2)        // Enable Error Interrupt 2
#define DMA_EEI_EEI3                    ((uint32_t)1<<3)        // Enable Error Interrupt 3

/* CEEI - Clear Enable Error Interrupt Register */
#define DMA_CEEI_CEEI(n)                ((uint8_t)(n & 3)<<0)   // Clear Enable Error Interrupt
#define DMA_CEEI_CAEE                   ((uint8_t)1<<6)         // Clear All Enable Error Interrupts
#define DMA_CEEI_NOP                    ((uint8_t)1<<7)         // NOP

/* SEEI - Set Enable Error Interrupt Register */
#define DMA_SEEI_SEEI(n)                ((uint8_t)(n & 3)<<0)   // Set Enable Error Interrupt
#define DMA_SEEI_SAEE                   ((uint8_t)1<<6)         // Set All Enable Error Interrupts
#define DMA_SEEI_NOP                    ((uint8_t)1<<7)         // NOP

/* CERQ - Clear Enable Request Register */
#define DMA_CERQ_CERQ(n)                ((uint8_t)(n & 3)<<0)   // Clear Enable Request
#define DMA_CERQ_CAER                   ((uint8_t)1<<6)         // Clear All Enable Requests
#define DMA_CERQ_NOP                    ((uint8_t)1<<7)         // NOP

/* SERQ - Set Enable Request Register */
#define DMA_SERQ_SERQ(n)                ((uint8_t)(n & 3)<<0)   // Set Enable Request
#define DMA_SERQ_SAER                   ((uint8_t)1<<6)         // Set All Enable Requests
#define DMA_SERQ_NOP                    ((uint8_t)1<<7)         // NOP

/* CDNE - Clear DONE Status Bit Register */
#define DMA_CDNE_CDNE(n)                ((uint8_t)(n & 3)<<0)   // Clear Done Bit
#define DMA_CDNE_CADN                   ((uint8_t)1<<6)         // Clear All Done Bits
#define DMA_CDNE_NOP                    ((uint8_t)1<<7)         // NOP

/* SSRT - Set START Bit Register */
#define DMA_SSRT_SSRT(n)                ((uint8_t)(n & 3)<<0)   // Set Start Bit
#define DMA_SSRT_SAST                   ((uint8_t)1<<6)         // Set All Start Bits
#define DMA_SSRT_NOP                    ((uint8_t)1<<7)         // NOP

/* CERR - Clear Error Register */
#define DMA_CERR_CERR(n)                ((uint8_t)(n & 3)<<0)   // Clear Error Indicator
#define DMA_CERR_CAEI                   ((uint8_t)1<<6)         // Clear All Error Indicators
#define DMA_CERR_NOP                    ((uint8_t)1<<7)         // NOP

/* CINT - Clear Interrupt Request Register */
#define DMA_CINT_CINT(n)                ((uint8_t)(n & 3)<<0)   // Clear Interrupt Request
#define DMA_CINT_CAIR                   ((uint8_t)1<<6)         // Clear All Interrupt Requests
#define DMA_CINT_NOP                    ((uint8_t)1<<7)         // NOP

/* INT - Interrupt Request Register */
#define DMA_INT_INT0                    ((uint32_t)1<<0)        // Interrupt Request 0
#define DMA_INT_INT1                    ((uint32_t)1<<1)        // Interrupt Request 1
#define DMA_INT_INT2                    ((uint32_t)1<<2)        // Interrupt Request 2
#define DMA_INT_INT3                    ((uint32_t)1<<3)        // Interrupt Request 3

/* ERR - Error Register */
#define DMA_ERR_ERR0                    ((uint32_t)1<<0)        // Error in Channel 0
#define DMA_ERR_ERR1                    ((uint32_t)1<<1)        // Error in Channel 1
#define DMA_ERR_ERR2                    ((uint32_t)1<<2)        // Error in Channel 2
#define DMA_ERR_ERR3                    ((uint32_t)1<<3)        // Error in Channel 3

/* HRS - Hardware Request Status Register */
#define DMA_HRS_HRS0                    ((uint32_t)1<<0)        // Hardware Request Status Channel 0
#define DMA_HRS_HRS1                    ((uint32_t)1<<1)        // Hardware Request Status Channel 1
#define DMA_HRS_HRS2                    ((uint32_t)1<<2)        // Hardware Request Status Channel 2
#define DMA_HRS_HRS3                    ((uint32_t)1<<3)        // Hardware Request Status Channel 3

/* DMA_DCHPRI - Channel n Priority Register */
#define DMA_DCHPRI_CHPRI(n)             ((uint8_t)(n & 3)<<0)   // Channel Arbitration Priority
#define DMA_DCHPRI_DPA                  ((uint8_t)1<<6)         // Disable PreEmpt Ability
#define DMA_DCHPRI_ECP                  ((uint8_t)1<<7)         // Enable PreEmption


/* TCD ATTR - Transfer Attributes */
#define DMA_ATTR_DSIZE(n)               ((uint16_t)(n & 0x07))         // Destination Data Transfer Size
#define DMA_ATTR_DMOD(n)                ((uint16_t)(n & 0x1f)<<3)      // Destination Address Modulo
#define DMA_ATTR_SSIZE(n)               ((uint16_t)(n & 0x07)<<8)      // Source Data Transfer Size
#define DMA_ATTR_SMOD(n)                ((uint16_t)(n & 0x1f)<<11)     // Source Address Modulo

#define DMA_ATTR_SIZE_8BIT              0
#define DMA_ATTR_SIZE_16BIT             1
#define DMA_ATTR_SIZE_32BIT             2
#define DMA_ATTR_SIZE_16BYTE            4
#define DMA_ATTR_SIZE_32BYTE            5

/* TCD Signed Minor Loop Offset */
#define DMA_NBYTES_SMLOE                ((uint32_t)1<<31)               // Source Minor Loop Offset Enable
#define DMA_NBYTES_DMLOE                ((uint32_t)1<<30)               // Destination Minor Loop Offset Enable
#define DMA_NBYTES_MLOFFNO_NBYTES(n)    ((uint32_t)(n))                 // NBytes transfer count when minor loop disabled
#define DMA_NBYTES_MLOFFYES_NBYTES(n)   ((uint32_t)(n & 0x1F))          // NBytes transfer count when minor loop enabled
#define DMA_NBYTES_MLOFFYES_MLOFF(n)    ((uint32_t)(n & 0xFFFFF)<<10)   // Offset 

/* TCD CSR bits */
#define DMA_CSR_START                   ((uint16_t)0x1)
#define DMA_CSR_INTMAJOR                ((uint16_t)0x2)
#define DMA_CSR_INTHALF                 ((uint16_t)0x4)
#define DMA_CSR_DREQ                    ((uint16_t)0x8)
#define DMA_CSR_ESG                     ((uint16_t)0x10)
#define DMA_CSR_MAJORELINK              ((uint16_t)0x20)
#define DMA_CSR_ACTIVE                  ((uint16_t)0x40)
#define DMA_CSR_DONE                    ((uint16_t)0x80)
#define DMA_CSR_MAJORLINKCH(n)          ((uint16_t)(n & 3)<<8)       // Link Channel Number
#define DMA_CSR_BWC(n)                  ((uint16_t)(n & 3)<<14)      // Bandwidth Control

#define DMA_CITER_MASK                  ((uint16_t)0x7FFF)     // Loop count mask
#define DMA_CITER_ELINK                 ((uint16_t)1<<15)      // Enable channel linking on minor-loop complete

#define DMA_BITER_MASK                  ((uint16_t)0x7FFF)     // Loop count mask
#define DMA_BITER_ELINK                 ((uint16_t)1<<15)      // Enable channel linking on minor-loop complete


/* DMAMUX Channel Configuration bits */
#define DMAMUX_CHCFG_ENBL                ((uint8_t)0x80)         // DMA Channel Enable
#define DMAMUX_CHCFG_TRIG                ((uint8_t)0x40)         // DMA Channel Trigger Enable
#define DMAMUX_CHCFG_SOURCE(n)           ((uint8_t)(n & 0x3f))      // DMA Channel Source

/* DMAMUX Request Source IDs */
#define DMAMUX_SOURCE_UART0_RX           2
#define DMAMUX_SOURCE_UART0_TX           3
#define DMAMUX_SOURCE_UART1_RX           4
#define DMAMUX_SOURCE_UART1_TX           5
#define DMAMUX_SOURCE_UART2_RX           6
#define DMAMUX_SOURCE_UART2_TX           7
#define DMAMUX_SOURCE_I2S0_RX            14
#define DMAMUX_SOURCE_I2S0_TX            15
#define DMAMUX_SOURCE_SPI0_RX            16
#define DMAMUX_SOURCE_SPI0_TX            17
#define DMAMUX_SOURCE_I2C0               22
#define DMAMUX_SOURCE_FTM0_CH0           24
#define DMAMUX_SOURCE_FTM0_CH1           25
#define DMAMUX_SOURCE_FTM0_CH2           26
#define DMAMUX_SOURCE_FTM0_CH3           27
#define DMAMUX_SOURCE_FTM0_CH4           28
#define DMAMUX_SOURCE_FTM0_CH5           29
#define DMAMUX_SOURCE_FTM0_CH6           30
#define DMAMUX_SOURCE_FTM0_CH7           31
#define DMAMUX_SOURCE_FTM1_CH0           32
#define DMAMUX_SOURCE_FTM1_CH1           33
#define DMAMUX_SOURCE_ADC0               40
#define DMAMUX_SOURCE_CMP0               42
#define DMAMUX_SOURCE_CMP1               43
#define DMAMUX_SOURCE_CMT                47
#define DMAMUX_SOURCE_PDB                48
#define DMAMUX_SOURCE_PORTA              49
#define DMAMUX_SOURCE_PORTB              50
#define DMAMUX_SOURCE_PORTC              51
#define DMAMUX_SOURCE_PORTD              52
#define DMAMUX_SOURCE_PORTE              53

/* ------ End Hardware ------------------------- */


// Config
#define I2S_WORD_LENGTH 16
#define I2S_SAMPLE_RATE 48000
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
 
void i2s_io_init(void);
void i2s_switch_clock(unsigned char clk);
int i2s_init(unsigned char clk, unsigned char useDMA);

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
