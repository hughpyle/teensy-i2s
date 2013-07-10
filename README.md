# I2S for Teensy3

The I2S protocol is designed for high-quality digital audio between devices.

[Teensy3](http://www.pjrc.com/teensy/), programmed with Arduino, uses a Kinetis K20 processor (ARM Cortex M4) that has hardware support for one I2S channel.
This library can drive the I2S either directly or using DMA.   DMA is much preferred, because the CPU isn't kept busy dealing with the I/O.

The example sketches use the WM8731 stereo codec with this [library](https://github.com/hughpyle/machinesalem-arduino-libs/tree/master/WM8731).
Examples show configuration for the [MikroE proto board](http://www.mikroe.com/add-on-boards/audio-voice/audio-codec-proto/) (48kHz, master) and for the [Open Music Labs codec shield](http://www.openmusiclabs.com/projects/codec-shield/) (44.1kHz, slave), which both use WM8731.

Current status:

* Transmit and Receive implemented with and without DMA.
* Current examples are tested with 16-bit audio the Open Codec Labs shield.  The output doesn't sound stereo, although the signals seem right.  If anyone can help diagose this, please let me know!
* Patches and suggestions please!

For more status, see the [forum](http://forum.pjrc.com/threads/15748-Teensy3-I2S-with-DMA).
