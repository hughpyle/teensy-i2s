# I2S for Teensy3

The I2S protocol is designed for high-quality digital audio between devices.

[Teensy3](http://www.pjrc.com/teensy/), programmed with Arduino, uses a Kinetis K20 processor (ARM Cortex M4) that has hardware support for one I2S channel.
This library can drive the I2S either directly or using DMA.   DMA is much preferred, because the CPU isn't kept busy dealing with the I/O.
Currently only "transmit" functions are implemented.

The example sketch uses the WM8731 stereo codec.  Requires this [library](https://github.com/hughpyle/machinesalem-arduino-libs/tree/master/WM8731).
The only sample rate currently tested is 48 kHz / 16 bit.

For more status, see the [forum](http://forum.pjrc.com/threads/15748-Teensy3-I2S-with-DMA).
