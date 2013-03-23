/*
  I2S & DMA digital audio demonstrator for Teensy 3.0
  Interfaces using Wolfson WM8731 codec.
  
  To use the Mikro proto board (as master): set clock type to I2S_CLOCK_EXTERNAL
  Note: this board doesn't have line-in connections.
      SCK  -> Teensy 9  (I2S0_TX_BCLK)
      MISO -> not connected for this transmit-only example
      MOSI -> Teensy 3  (I2S0_TXD0).  Can also be switched to pin 22.
      ADCL -> not connected for this transmit-only example
      DACL -> Teensy 4  (I2S0_TX_FS).  Can also be switched to pin 23 or 25
      SDA  -> Teensy 18 (I2C0_SDA)
      SCL  -> Teensy 19 (I2C0_SCL)
      3.3V -> Teensy 3.3v
      GND  -> Teensy GND
  
  To use the openmusiclabs audio codec shield (as slave): set clock type to I2S_CLOCK_44K_INTERNAL.
  Note: this board cannot be used as master, only as slave.
      J1#2 GND
      J1#4 3.3v      (or 5V, but then you must NOT connect the analog pots A0/A1 to teensy pins directly)
      J1#5 3.3v
      J2#1 SCL(A5)   -> Teensy 19 (I2C0_SCL)
      J2#2 SDA(A4)   -> Teensy 18 (I2C0_SDA)
      J3#3 SCK(D13)  -> Teensy 9  (I2S0_TX_BCLK)
      J3#4 MISO(D12) n/c
      J3#5 MOSI(D11) -> Teensy 3  (I2S0_TXD0)
      J3#6 SS(D10)   -> Teensy 4  (I2S0_TX_FS)
      J4#3 CLKOUT(D5)    -> Teensy 11 (I2S0_MCLK)
*/


/* Wolfson audio codec controlled by I2C */
/* Library here: https://github.com/hughpyle/machinesalem-arduino-libs/tree/master/WM8731 */
#include <Wire.h>
#include <WM8731.h>


/* I2S digital audio */
#include <i2s.h>
const uint8_t clocktype = I2S_CLOCK_44K_INTERNAL;


// audio data
int16_t audf, audx, audy, audd;
int32_t nnn=0;

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



/* ----------------------- DMA transfer, we get callback to fill one of the ping-pong buffers ------ */
void dma_callback( int16_t *pBuf, uint16_t len )
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


/* ----------------------- begin -------------------- */

void setup()
{
  initsinevalue();
  Serial.println( "Initializing" );
  
  delay(2000); 
  Serial.println( "Initializing." );

  delay(1000);
  unsigned char interface = WM8731_INTERFACE_FORMAT(I2S) | WM8731_INTERFACE_WORDLEN(bits16);
  if( clocktype==I2S_CLOCK_EXTERNAL )
  {
    interface |= WM8731_INTERFACE_MASTER;
  }
  WM8731.begin( low, WM8731_SAMPLING_RATE(hz44100), interface );
  WM8731.setActive();
  WM8731.setOutputVolume( 127 );
  Serial.println( "Initialized I2C" );
  
  delay(1000);
  I2STx0.begin( clocktype, dma_callback );
  Serial.println( "Initialized I2S with DMA" );
  
  I2STx0.start();
}


/* --------------------- main loop ------------------ */
void loop()
{
}

