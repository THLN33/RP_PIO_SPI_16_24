
#include "def.h"

/*
from : https://github.com/Bodmer/TFT_eSPI
*/


#include "ILI9488_Regs.h"

// This is the command sequence that initialises the ILI9488 driver
//
// This setup information uses simple 8-bit SPI writecommand() and writedata() functions
//
// See ST7735_Setup.h file for an alternative format

#include "def.h"


static uint16_t addr_col, addr_row;


bool _vpDatum, _vpOoB;
uint16_t _xDatum = 0, _yDatum = 0;
uint16_t _vpX = 0, _vpY = 0;
uint16_t _xWidth, _yHeight, _vpW, _vpH, _width, _height;
uint8_t rotation = 0;

#define tft_Write_32C(C,D) \
    tft_Write_16((uint16_t) (C)); \
    tft_Write_16((uint16_t) (D))

#ifdef USE_SPI

#define SPI_BUSY_CHECK  while(spi_is_busy(TFT_SPI_PORT))

/***************************************************************************************
** Function name:           begin_tft_write (was called spi_begin)
** Description:             Start tft_spi transaction for writes and select TFT
***************************************************************************************/
static inline void begin_tft_write(void) {
    //tft_spi.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, TFT_SPI_MODE));
    spi_init(TFT_SPI_PORT, SPI_FREQUENCY);    
    spi_set_format(TFT_SPI_PORT, 8, (spi_cpol_t)(TFT_SPI_MODE >> 1), (spi_cpha_t)(TFT_SPI_MODE & 0x1), SPI_MSB_FIRST);
    CS_L;
}

/***************************************************************************************
** Function name:           end_tft_write (was called spi_end)
** Description:             End transaction for write and deselect TFT
***************************************************************************************/
static inline void end_tft_write(void) {
      CS_H;
      //tft_spi.endTransaction();
      spi_deinit(TFT_SPI_PORT);
  }



void tft_Write_8(uint8_t data) {
    spi_write_blocking(TFT_SPI_PORT, &data, 1);
}


void tft_Write_16(uint16_t data) {
    //spi_write16_blocking(TFT_SPI_PORT, &data, 2);
    uint8_t h = data>>8;
    uint8_t l = data;
    spi_write_blocking(TFT_SPI_PORT, &h, 1);
    spi_write_blocking(TFT_SPI_PORT, &l, 1);
}

#else 

extern void pioinit2(uint32_t clock_freq);

extern void begin_tft_write(void);
extern void end_tft_write(void);

extern void tft_Write_8(uint8_t data);
extern void tft_Write_16(uint16_t data);

extern void writecommand(uint16_t c);
extern void writedata(uint8_t d);

#endif









void ILI9488_Hardware_Init(void) {

    gpio_init(TFT_CS);
    gpio_set_dir(TFT_CS, GPIO_OUT);
    gpio_put(TFT_CS, HIGH);    

    gpio_init(TFT_LED);
    gpio_set_dir(TFT_LED, GPIO_OUT);
    gpio_put(TFT_LED, HIGH);

    gpio_init(TFT_DC);
    gpio_set_dir(TFT_DC, GPIO_OUT);
    gpio_put(TFT_DC, HIGH);

    gpio_init(TFT_RST);    
    gpio_set_dir(TFT_RST, GPIO_OUT);
    gpio_put(TFT_RST, HIGH);


#ifdef USE_SPI
    gpio_set_function(TFT_SCLK, GPIO_FUNC_SPI);
    gpio_set_function(TFT_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(TFT_MISO, GPIO_FUNC_SPI);
#else
    pioinit2(PIO_SPI_FREQUENCY);

    //#error You should use an interface
#endif
}



void ILI9488_HardReset(void) {

    writecommand(0x00); // Put SPI bus in known state for TFT with CS tied low
    gpio_put(TFT_RST, HIGH);
    delay(5);
    gpio_put(TFT_RST, LOW);
    delay(20);
    gpio_put(TFT_RST, HIGH);
}


void ILI9488_Init_Registers(void) {
    // Configure ILI9488 display
    writecommand(0xE0);     // Positive Gamma Control
    writedata(0x00);        // 00
    writedata(0x03);        // 07
    writedata(0x09);        // 10
    writedata(0x08);        // 09
    writedata(0x16);        // 17
    writedata(0x0A);        // 0B
    writedata(0x3F);        // 41
    writedata(0x78);        // 89
    writedata(0x4C);        // 4B
    writedata(0x09);        // 0A
    writedata(0x0A);        // 0C
    writedata(0x08);        // 0E
    writedata(0x16);        // 18
    writedata(0x1A);        // 1B
    writedata(0x0F);        // 0F

    writecommand(0XE1);     // Negative Gamma Control
    writedata(0x00);        // 00
    writedata(0x16);        // 17
    writedata(0x19);        // 1A
    writedata(0x03);        // 04
    writedata(0x0F);        // 0E
    writedata(0x05);        // 06
    writedata(0x32);        // 2F
    writedata(0x45);        // 45
    writedata(0x46);        // 43
    writedata(0x04);        // 02
    writedata(0x0E);        // 0A
    writedata(0x0D);        // 09
    writedata(0x35);        // 32
    writedata(0x37);        // 36
    writedata(0x0F);        // 0F

    writecommand(0XC0);     // Power Control 1
    writedata(0x17);        // 11
    writedata(0x15);        // 09

    writecommand(0xC1);     // Power Control 2
    writedata(0x41);

    writecommand(0xC5);     // VCOM Control
    writedata(0x00);        // 00
    writedata(0x12);        // 28
    writedata(0x80);        // 80

    writecommand(TFT_MADCTL); // Memory Access Control
    //writedata(0x48);          // MX, BGR
    writedata(0x08);


    writecommand(0x3A);     // set Pixel Format Set
#if (TFT_COLORS == TFT_COLOR_1BITS)  
    writedata(0x01);        // 3 bits / pixel
#elif (TFT_COLORS == TFT_COLOR_4BITS)   
#error 4 bits Colors depth not supported 
#elif (TFT_COLORS == TFT_COLOR_12BITS)    
#error 12 bits Colors depth not supported 
    writedata(0x03);        // 12 bits / pixel
#elif (TFT_COLORS == TFT_COLOR_16BITS)   
    writedata(0x05);        // 16 bits / pixel 
#elif (TFT_COLORS == TFT_COLOR_18BITS)
    writedata(0x66);        // 18 bits / pixel
#elif (TFT_COLORS == TFT_COLOR_24BITS)
    writedata(0x77);        // 24 bits / pixel
#else
#error Colors depth should be defined!
#endif    

/*
    writecommand(0x3A); // Pixel Interface Format
#if defined (TFT_PARALLEL_8_BIT) || defined (TFT_PARALLEL_16_BIT) || defined (RPI_DISPLAY_TYPE)
    //writedata(0x55);  // 16-bit colour for parallel
#else
    //writedata(0x66);  // 18-bit colour for SPI
#endif
writedata(0x66); 
*/

    writecommand(0xB0); // Interface Mode Control
    writedata(0x00);

    writecommand(0xB1); // Frame Rate Control
    //writedata(0xA0);
    writedata(0xB0);        // *
    writedata(0x11);        // *

    writecommand(0xB4);     // Display Inversion Control
    writedata(0x02);

    writecommand(0xB6);     // Display Function Control
    writedata(0x02);
    writedata(0x02);
    writedata(0x3B);

    writecommand(0xB7);     // Entry Mode Set
    writedata(0xC6);

	writecommand(0xE9);     // Set Image Function
	writedata(0x00);

    writecommand(0xF7);     // Adjust Control 3
    writedata(0xA9);
    writedata(0x51);
    writedata(0x2C);
    writedata(0x82);    

    writecommand(TFT_SLPOUT);  //Exit Sleep
    delay(120);

    writecommand(TFT_DISPON);  //Display on
    delay(25);

    // End of ILI9488 display configuration
}

/***************************************************************************************
** Function name:           width
** Description:             Return the pixel width of display (per current rotation)
***************************************************************************************/
// Return the size of the display (per current rotation)
int16_t width(void) {
	if (_vpDatum)
		return _xWidth;
	return _width;
}


/***************************************************************************************
** Function name:           height
** Description:             Return the pixel height of display (per current rotation)
***************************************************************************************/
int16_t height(void) {
	if (_vpDatum)
		return _yHeight;
	return _height;
}



/***************************************************************************************
** Function name:           resetViewport
** Description:             Reset viewport to whole TFT screen, datum at 0,0
***************************************************************************************/
void resetViewport(void) {
  // Reset viewport to the whole screen (or sprite) area
  _vpDatum = false;
  _vpOoB   = false;
  _xDatum = 0;
  _yDatum = 0;
  _vpX = 0;
  _vpY = 0;
  _vpW = width();
  _vpH = height();
  _xWidth  = width();
  _yHeight = height();
}

/***************************************************************************************
** Function name:           setRotation
** Description:             rotate the screen orientation m = 0-3 or 4-7 for BMP drawing
***************************************************************************************/
void setRotation(uint8_t m) {

    begin_tft_write();
    writecommand(TFT_MADCTL);
    rotation = m % 4;
    switch (rotation) {
    case 0: // Portrait
        writedata(TFT_MAD_MX | TFT_MAD_BGR);
        _width  = TFT_WIDTH;
        _height = TFT_HEIGHT;
        break;
    case 1: // Landscape (Portrait + 90)
        writedata(TFT_MAD_MV | TFT_MAD_BGR);
        _width  = TFT_HEIGHT;
        _height = TFT_WIDTH;
        break;
    case 2: // Inverter portrait
        writedata(TFT_MAD_MY | TFT_MAD_BGR);
        _width  = TFT_WIDTH;
        _height = TFT_HEIGHT;
        break;
    case 3: // Inverted landscape
        writedata(TFT_MAD_MX | TFT_MAD_MY | TFT_MAD_MV | TFT_MAD_BGR);
        _width  = TFT_HEIGHT;
        _height = TFT_WIDTH;
        break;
  }

  //delayMicroseconds(10);
	sleep_us(10);
	end_tft_write();

	addr_row = 0xFFFF;
	addr_col = 0xFFFF;

	// Reset the viewport to the whole screen
	resetViewport();
}



#ifdef _OLD
void setWindow(int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
#ifdef USE_SPI
    SPI_BUSY_CHECK;
#endif
    DC_C; 
    tft_Write_8(TFT_CASET);
    DC_D;
    tft_Write_32C(x0, x1);

    DC_C; 
    tft_Write_8(TFT_PASET);
    DC_D; 
    tft_Write_32C(y0, y1);

    DC_C; 
    tft_Write_8(TFT_RAMWR);
    DC_D;
}
#endif



void ILI9488_Init(void) {
    ILI9488_Hardware_Init();
    ILI9488_HardReset();
    ILI9488_Init_Registers();

    setRotation(3);
}





