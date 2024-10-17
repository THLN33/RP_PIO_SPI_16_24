Hello, 

I also use a Raspberry Pi Pico card (RP2040) with an LCD screen managed by the ILI9488 circuit. DMA transfers do not work with the original TFT_eSPI library. The image is displayed with wrong colors.

Indeed, the ILI9488 circuit wired in 4-line SPI (IM[2:0]=111) only accepts the following modes:
- RGB 1-1-1 mode or 2^(3*1) = 8 colors per pixel (one level per primary color)
- RGB 6-6-6 mode or 2^(3*6) = 262K colors per pixel (2⁶ = 64 levels per primary color)

The transfer of this last mode is necessarily done in groups of 3 bytes (24 bits).
The other modes (RGB 5-6-5 and 8-8-8) are only available in parallel connection.

The TFT_eSPI library works with a 16-bit RGB 5-6-5 mode. To correctly display an image, the processor must convert the color data to RGB 6-6-6 mode for each pixel and transfer 3 bytes to the ILI9488 graphics controller.
The idea is to use the PIO to do this conversion, which makes it possible to use DMA for transfers. The processor is thus relieved of this heavy task.

The TFT_eSPI library already uses a PIO to transmit data to the graphics circuit with RGB 6-6-6 coded data, the initial PIO program already occupies the maximum 32 instructions, it is not possible to modify it, it is already well optimized.
The solution is to use the second PIO to perform the RGB 5-6-5 to RGB 6-6-6 conversion and the transfer to the circuit.
To perform this test, I used Bodmer's library, and picked up some pieces of his code.

When using both PIOs, you have to make sure to correctly reconfigure the GPIOs to the right PIO.

If DMA is used, you have to detect and wait for the end of the last transfer before sending the next one. The dma_handler() function allows this.

With a PIO frequency of 125 MHz, the transfer of a complete image is done in about 62ms.
 - number of bytes for a complete image: 480 x 320 = 153600
 - number of cycles per transmitted pixel: (24 + 1) = 25
 - the clock division: 2 

The necessary transmission time = 153600 * (2*25) / 125 Mhz = 61.4 ms

This works fine, so it remains to integrate it into the TFT_eSPI library if it is useful to you.

Happy tinkering, See you soon!
