// This is for Teensy 3.0
// using ARM Cortex M4 math routines
#define ARM_MATH_CM4
#include <arm_math.h>

#include <i2s.h>


void setup()
{
  Serial.println( "Initializing" );
  delay(2000);
  
  Serial.println( "Initializing." );
  delay(1000);  
  Serial.println( "Initializing.." );
  delay(1000);
  Serial.println( "Initializing...?" );
  delay(1000);
  Serial.println( "f" );
  Serial.println( i2s_init(I2S_CLOCK_48K_INTERNAL), DEC );  
  Serial.println( "I2S initialized." );  
  

  delay(1000);
  dma_init();
  Serial.println( "DMA initialized." );  
}



void loop()
{
  delay(1000);
  Serial.println( "Waiting." );  
  
  delay(1000);
  dma_play();
  Serial.println( "DMA playing." );  

  delay(1000);
  dma_stop();
  Serial.println( "DMA stopped." );  
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
