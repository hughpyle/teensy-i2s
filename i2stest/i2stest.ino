// This is for Teensy 3.0
// using ARM Cortex M4 math routines
#define ARM_MATH_CM4
#include <arm_math.h>

/* I2S digital audio */
#include <i2s.h>

/* Wolfson audio codec controlled by I2C */
/* Library here: https://github.com/hughpyle/machinesalem-arduino-libs/tree/master/WM8731 */
#include <Wire.h>
#include <WM8731.h>


/*
  SDA -> Teensy pin 18
  SCL -> Teensy pin 19
  SCK -> Teensy 11 (ALT6 I2S0_MCLK) (PTC6/LLWU_P10)
  MOSI -> Teensy 3 (ALT6 I2S0_TXD0) (PTA12) // 22
*/

const int isDMA = 0;
const int codecIsMaster = 1;


void setup()
{
  initsinevalue(); 
  Serial.println( "Initializing" );
  
  delay(2000); 
  Serial.println( "Initializing." );

  delay(1000);
  unsigned char interface = WM8731_INTERFACE_FORMAT(I2S) | WM8731_INTERFACE_WORDLEN(bits16);
  if( codecIsMaster )
  {
    interface |= WM8731_INTERFACE_MASTER;
  }  
  WM8731.begin( low, WM8731_SAMPLING_RATE(hz48000), interface );
  WM8731.setActive();
  WM8731.setOutputVolume( 255 );
  Serial.println( "Initialized I2C" );
  
  delay(1000);
  if( codecIsMaster )
  {
    // External clock, 48k sample rate
    Serial.println( "Initializing for external clock" );
    Serial.println( i2s_init(I2S_CLOCK_EXTERNAL, isDMA), DEC );  
  }
  else
  {
    // Internal clock, directly writing to the i2s FIFO
    Serial.println( "Initializing for internal clock" );
    Serial.println( i2s_init(I2S_CLOCK_48K_INTERNAL, isDMA), DEC );  
  }
  Serial.println( "Initialized I2S." );  

  if( isDMA )
  {  
    delay(1000);
    dma_init();
    Serial.println( "Initialized DMA." );  
  }
}




// audio data
q15_t audx = 0;
q15_t audy = 0.9 * 32767;
q15_t audd = 2.0 * sin(PI/100) * 32767;
q15_t audf = 1.0 * 32767;
uint32_t a = 0;
uint32_t b = 0;
uint32_t p = 0;
uint32_t nnn=0;
void initsinevalue()
{
  audx = 0;
  audy = 32767;
  a = __PKHBT( audf, audd, 16 );
  b = __PKHBT( audx, audy, 16 );
}
void nextsinevalue() 
{
  // b = 0xACCF0010; audx=0xACCF; return;
  //  http://cabezal.com/misc/minsky-circles.html
  p = __SMUAD(a,b)<<1;
  b = (b>>16) + (p & 0xFFFF0000);
  p = __SMUSD(a,b)<<1;
  b = (b>>16) + (p & 0xFFFF0000);
  audx = (q15_t)(p & 0xFFFF);
  nnn++;
  if(nnn>48000) {nnn=0;initsinevalue();};
}

/* --------------------- Direct I2S data transfer, we get the FIFO callback ----- */

/*
Writes to a TDR are ignored if the corresponding bit of TCR3[TCE] is clear or if the
FIFO is full. If the Transmit FIFO is empty, the TDR must be written at least three bit
clocks before the start of the next unmasked word to avoid a FIFO underrun.
*/

/*
The transmit data ready flag is set when the number of entries in any of the enabled
transmit FIFOs is less than or equal to the transmit FIFO watermark configuration and is
cleared when the number of entries in each enabled transmit FIFO is greater than the
transmit FIFO watermark configuration.
*/

int8_t ever_called_tx = 0;
void i2s0_tx_isr(void)
{
  // Clear the FIFO error flag
  if(I2S0_TCSR & I2S_TCSR_FEF)  // underrun
     I2S0_TCSR |= I2S_TCSR_FEF; // clear

  if(I2S0_TCSR & I2S_TCSR_SEF)  // frame sync error
     I2S0_TCSR |= I2S_TCSR_SEF; // clear

  // Send two words of data (the FIFO buffer is just 2 words)
  I2S0_TDR0 = (uint32_t)b; //audx;
  //nextsinevalue();
  I2S0_TDR0 = (uint32_t)b; //audx;
  nextsinevalue();

//  Serial.println( I2S0_TCSR, HEX );  
}





/* ----------------------- DMA transfer ------------------------- */


void dma_fill( int16_t *pBuf, int16_t len )
{
  while( len>0 )
  {
    *pBuf++ = audx;
    nextsinevalue();
    len--;
  }
  Serial.println("fills");
}


//extern pointer  event_dma_rdy;
void dma_ch0_isr(void)
{
  int16_t *pBuf;
 
  DMA_CINT = DMA_CINT_CINT(0);                 // use the Clear Intr. Request register 

  Serial.println("ISR");
    
  if (Playing_Buff_A)
  {                        // finished playing buffer A
    Playing_Buff_A = 0;
    DMA_TCD0_SADDR          = (uint32_t) Audio_Source_Blk_B;
    pBuf = (int16_t *)Audio_Source_Blk_A;
  }
  else
  {
    Playing_Buff_A = 1;
    DMA_TCD0_SADDR          = (uint32_t) Audio_Source_Blk_A;
    pBuf = (int16_t *)Audio_Source_Blk_B;
  } 
   
  // DMA finished playback an ready for a new buffer
  //_event_set(event_dma_rdy,0x01);
  dma_fill( pBuf, DMA_BUFFER_SIZE );
}




/* --------------------- main loop ------------------ */
void loop()
{
  uint32_t es;
  
  delay(1000);
  Serial.println( "Waiting." );  
  
  if( isDMA )
  {
    delay(1000);
    dma_play();
    Serial.println( "DMA playing." );
    es = DMA_ERR;
    if(es>0) Serial.println( es, DEC );  // DMA error status
  
    delay(1000);
    dma_stop();
    Serial.println( "DMA stopped." );  
  }
  else
  {
    // Bang some data at the I2S port directly
    delay(1000);
    Serial.println( "I2S playing." );
  }
}


/*
// DMA ping-pong ISR

void hal_fill_tx_buf(s32 *p_r, s32 *p_l, uint buf_n_sample)
{
    static int index = 0;
    static int data_index = 0;
    int i;
    s32 *p_r_tx;
    s32 *p_l_tx;

    // get buffer pointer
    if(index == 0)
    {
      p_r_tx = (int*)i2s_buf.buf_i2s_r_tx;
      p_l_tx = (int*)i2s_buf.buf_i2s_l_tx;
    }
    else
    {
      p_r_tx = (int*)(i2s_buf.buf_i2s_r_tx+I2S_BLOCK_N_SAMPLES*I2S_SAMPLE_N_BYTE);
      p_l_tx = (int*)(i2s_buf.buf_i2s_l_tx+I2S_BLOCK_N_SAMPLES*I2S_SAMPLE_N_BYTE);
    }

    // set content in the buffer
    for( i=0; i<I2S_BLOCK_N_SAMPLES; i++ )
    {
      *p_r_tx++ = p_r[data_index];
      *p_l_tx++ = p_l[data_index];
      data_index++;
      if(data_index >= buf_n_sample)
        data_index = 0;
    }
    index ^= 1;
}

*/

void dma_error_isr(void)
{
  DMA_CINT = DMA_CINT_CINT(0);

  Serial.println("Error ISR");
}

