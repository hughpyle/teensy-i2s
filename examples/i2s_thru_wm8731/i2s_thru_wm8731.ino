/* TODO synchronous receive only if we choose it explicitly */

/*
  I2S digital audio demonstrator for Teensy 3.0
  Interfaces using Wolfson WM8731 codec.
  
  This example is a "play through delay" test,   using I2S (not DMA).
  Reads input into a circular buffer, writes output from the same buffer,
  If the buffer size is 2, this is "straight through".
*/

/*
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
*/

/*  
  To use the openmusiclabs audio codec shield (as slave): set clock type to I2S_CLOCK_44K_INTERNAL.
  Note: this board cannot be used as master, only as slave.
      J1#2 GND
      J1#4 3.3v      (or 5V, but then you must NOT connect the analog pots A0/A1 to teensy pins directly)
      J1#5 3.3v
      J2#1 SCL(A5)   -> Teensy 19 (I2C0_SCL)
      J2#2 SDA(A4)   -> Teensy 18 (I2C0_SDA)
      J3#3 SCK(D13)  -> Teensy 9  (I2S0_TX_BCLK)
      J3#4 MISO(D12) -> Teensy 13 (I2S0_RXD0)
      J3#5 MOSI(D11) -> Teensy 3  (I2S0_TXD0)
      J3#6 SS(D10)   -> Teensy 4  (I2S0_TX_FS)
      J4#3 CLKOUT(D5)    -> Teensy 11 (I2S0_MCLK)
      
Receive pins 11, 12, 13 (no MCLK)
Rx pin 12 // I2S0_RX_FS
Rx pin 11 // I2S0_RX_BCLK

*/

#if 0

// Settings for MikroE prototype board
#define CLOCK_TYPE                  (I2S_CLOCK_EXTERNAL)
#define CODEC_INTERFACE_FLAGS       (WM8731_INTERFACE_FORMAT(I2S) | WM8731_INTERFACE_WORDLEN(bits16) | WM8731_INTERFACE_MASTER)
#define CODEC_BITRATE               (WM8731_SAMPLING_RATE(hz48000))
#define CODEC_ANALOG_FLAGS          (WM8731_ANALOG_DACSEL | WM8731_ANALOG_MICBOOST | WM8731_ANALOG_INSEL)

#else

// Settings for OML audio codec shield
#define CLOCK_TYPE                  (I2S_CLOCK_44K_INTERNAL)
#define CODEC_INTERFACE_FLAGS       (WM8731_INTERFACE_FORMAT(I2S) | WM8731_INTERFACE_WORDLEN(bits16) ) 
#define CODEC_BITRATE               (WM8731_SAMPLING_RATE(hz44100))
#define CODEC_ANALOG_FLAGS          (WM8731_ANALOG_DACSEL)

#endif


/* Wolfson audio codec controlled by I2C */
/* Library here: https://github.com/hughpyle/machinesalem-arduino-libs/tree/master/WM8731 */
#include <Wire.h>
#include <WM8731.h>


/* I2S digital audio */
#include <i2s.h>


/* Circular buffer for audio samples, interleaved left & right channel */
const uint16_t buffersize = 2; // 2048;
volatile int16_t buffer[buffersize];
uint16_t nTX = 0;
uint16_t nRX = 0;


/* --------------------- Direct I2S Receive, we get callback to read 2 words from the FIFO ----- */

void i2s_rx_callback( int16_t *pBuf )
{
  // Read the data
  buffer[nRX++] = pBuf[0];
  buffer[nRX++] = pBuf[1];
  if( nRX>=buffersize ) nRX=0;
}


/* --------------------- Direct I2S Transmit, we get callback to put 2 words into the FIFO ----- */

void i2s_tx_callback( int16_t *pBuf )
{
  // Send the data
  pBuf[0] = buffer[nTX++];
  pBuf[1] = buffer[nTX++];
  if( nTX>=buffersize ) nTX=0;
}


/* ----------------------- begin -------------------- */

void setup()
{
  Serial.println( "Initializing" );
  
  delay(2000); 
  Serial.println( "Initializing." );

  delay(1000);
  WM8731.begin( low, CODEC_BITRATE, CODEC_INTERFACE_FLAGS );
  WM8731.setActive();
  WM8731.setInputVolume( 63 );
  WM8731.setOutputVolume( 127 );
  WM8731.set( WM8731_ANALOG, CODEC_ANALOG_FLAGS );
  Serial.println( "Initialized I2C Codec" );
  
  delay(1000);
  I2SRx0.begin( CLOCK_TYPE, i2s_rx_callback );
  Serial.println( "Initialized I2S RX without DMA" );
  
  I2STx0.begin( CLOCK_TYPE, i2s_tx_callback );
  Serial.println( "Initialized I2S TX without DMA" );
  
  // Before starting tx/rx, set the buffer pointers
  // Receiver gets data just behind the transmit pointer (i.e. buffersize-2 "ahead")
  nRX = 0;
  nTX = 2;
  
  I2STx0.start();
  I2SRx0.start();
}


/* --------------------- main loop ------------------ */
void loop()
{
    /* do nothing */
}

