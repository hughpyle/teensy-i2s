// This is for Teensy 3.0
// using ARM Cortex M4 math routines
#define ARM_MATH_CM4
#include <arm_math.h>

/* I2S digital audio */
#include <i2s.h>

/* Wolfson audio codec controlled by I2C */
#include <Wire.h>
#include <WM8731.h>

/*
  SDA -> Teensy pin 18
  SCL -> Teensy pin 19
  SCK -> Teensy 11 (ALT6 I2S0_MCLK) (PTC6/LLWU_P10)
  MOSI -> Teensy 3 (ALT6 I2S0_TXD0) (PTA12)
*/

const int dodma = 0;



void setup()
{
  Serial.println( "Initializing" );
  
  delay(2000);  
  Serial.println( "Initializing." );
  
  delay(1000);
  if( dodma )
  {
    unsigned char interface = 1; WM8731_INTERFACE_FORMAT(I2S) | WM8731_INTERFACE_WORDLEN(bits16) | WM8731_INTERFACE_MASTER;
    WM8731.begin( low, WM8731_SAMPLING_RATE(hz48000), interface );
  }
  else
  {
    unsigned char interface = WM8731_INTERFACE_FORMAT(I2S) | WM8731_INTERFACE_WORDLEN(bits16);
    WM8731.begin( low, WM8731_SAMPLING_RATE(hz48000), interface );
  }
  WM8731.setActive();
  Serial.println( "Initialized I2C" );
  
  delay(1000);
  Serial.println( "Initializing...?" );

  delay(1000);
  if( dodma )
  {
    // DMA and external clock, 48k sample rate
    Serial.println( i2s_init(I2S_CLOCK_EXTERNAL), DEC );  
  }
  else
  {
    // Internal clock, 48k
    Serial.println( i2s_init(I2S_CLOCK_48K_INTERNAL), DEC );  
  }
  Serial.println( "Initialized I2S." );  

  if( dodma )
  {  
    delay(1000);
    dma_init();
    Serial.println( "Initialized DMA." );  
  }
}



void loop()
{
  uint32_t es;
  
  delay(1000);
  Serial.println( "Waiting." );  
  
  if( dodma )
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


void dma_fill( int16_t *pBuf, int16_t len )
{
  while( len>0 )
  {
    *pBuf++ = 0;
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
