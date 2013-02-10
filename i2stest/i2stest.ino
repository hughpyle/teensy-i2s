/*
  I2S & DMA digital audio demonstrator for Teensy 3.0
  Interfaces using Wolfson WM8731 codec.  For example using the Mikro proto board.
  
  SDA -> Teensy pin 18
  SCL -> Teensy pin 19
  SCK -> Teensy 11 (ALT6 I2S0_MCLK) (PTC6/LLWU_P10)
  MOSI -> Teensy 3 (ALT6 I2S0_TXD0) (PTA12) // can also be switched to pin 22
*/


/* Wolfson audio codec controlled by I2C */
/* Library here: https://github.com/hughpyle/machinesalem-arduino-libs/tree/master/WM8731 */
#include <Wire.h>
#include <WM8731.h>


/* I2S digital audio */
#include <i2s.h>

void dma_fill( int16_t *pBuf, int16_t len );


/* ---- begin ---- */

const int isDMA = 1;
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
  WM8731.setOutputVolume( 127 );
  Serial.println( "Initialized I2C" );
  
  delay(1000);
  unsigned char clock = I2S_CLOCK_48K_INTERNAL;
  if( codecIsMaster )
  {
    clock = I2S_CLOCK_EXTERNAL;
  }
  
  if( isDMA )
  {
    I2STx0.begin( clock, I2S_TX_PIN_PATTERN_1, dma_fill );
    I2STx0.start();
    Serial.println( "Initialized I2S with DMA" );
  }
  else
  {
    I2STx0.begin( clock, I2S_TX_PIN_PATTERN_1 );
    I2STx0.start();
    Serial.println( "Initialized I2S without DMA" );
  }
}


// audio data
int16_t audf = 0;
int16_t audx = 0;
int16_t audy = 0.9 * 32767;
int16_t audd = 2.0 * sin(PI*440/48000) * 32767;
int     nnn=0;

void initsinevalue()
{
  audf = 45 + (rand() % 48);                                // midi note number
  float f = (440.0 / 32) * pow(2, ((float)audf - 9) / 12);  // Hz.  For realz, use a lookup table.
  audd = 2.0 * sin(PI*f/48000) * 32767;                     // delta (q15_t)
  audx = 0;
  audy = 0.9 * 32767;                                       // start somewhere near full-scale
}

void nextsinevalue() 
{
  nnn++;
  if(nnn>48000) {nnn=0;initsinevalue();};                                // reset every second
//  if(nnn>24000){nnn=0;audx=audx<<1;if(audx==0)audx=1;b=audx;};return;  // marching blip
//  audx+=4;if(nnn>512){nnn=0;audx=-2048;};b=audx;return;                // stair
//  b = 0xACCF0010; audx=0xACCF; return;                                 // const pattern
  audx+=((audd*audy)>>15)&0xFFFFu; audy-=((audd*audx)>>15)&0xFFFFu;      // sinewaves http://cabezal.com/misc/minsky-circles.html
}



/* --------------------- Direct I2S data transfer, we get callback to put 2 words into the FIFO ----- */

void i2s0_tx_isr(void)
{
  // Clear the FIFO error flag
  if(I2S0_TCSR & I2S_TCSR_FEF)  // underrun
     I2S0_TCSR |= I2S_TCSR_FEF; // clear

  if(I2S0_TCSR & I2S_TCSR_SEF)  // frame sync error
     I2S0_TCSR |= I2S_TCSR_SEF; // clear

  // Send two words of data (the FIFO buffer is just 2 words, left & right)
  I2S0_TDR0 = (uint32_t)audy;
  I2S0_TDR0 = (uint32_t)audy;
  nextsinevalue();

  // Serial.println( I2S0_TCSR, HEX );
}


/* ----------------------- DMA transfer, we get callback to fill one of the ping-pong buffers ------ */
void dma_fill( int16_t *pBuf, int16_t len )
{
  while( len>0 )
  {
    *pBuf++ = audx;
    *pBuf++ = audy;
    nextsinevalue();
    len--;
    len--;
  }
  // Serial.println(audf,DEC);
}


/* --------------------- main loop ------------------ */
void loop()
{
}

