//#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#define PIO_16_to_24
#define DMA_16_to_24

#include "ILI9488_Regs.h"
#include "pio_spi_test.pio.h"
//#include "build\pio_spi_test.pio.h"

#include "tft_pinout.h"
#include "color_RGB565.h"


// SM jump instructions to change SM behaviour
uint32_t pio_instr_jmp8  = 0;
uint32_t pio_instr_fill  = 0;
uint32_t pio_instr_addr  = 0;

// SM "set" instructions to control DC control signal
uint32_t pio_instr_set_dc = 0;
uint32_t pio_instr_clr_dc = 0;

static bool _swapBytes;


#define LOW         0
#define HIGH        1

#define CS_L        gpio_put(TFT_CS, LOW)
#define CS_H        gpio_put(TFT_CS, HIGH)

#define DC_C        PIO_NB->sm[pio_sm].instr = pio_instr_clr_dc;
#define DC_D        PIO_NB->sm[pio_sm].instr = pio_instr_set_dc;

#ifdef PIO_16_to_24

#define PIO_NB      PIO_sm[pio_number].pio 
#define PIO_SM      PIO_sm[pio_number].sm 
#define PIO_PSP     PIO_sm[pio_number].pull_stall_mask

typedef struct {
    // PIO number    
    PIO     pio;
    // SM number
    int8_t  sm;
    // SM stalled mask
    uint32_t pull_stall_mask;
} PIO_sm_def;

static PIO_sm_def PIO_sm[] = {
    {NULL, -1, 0},
    {NULL, -1, 0},
};

static int8_t pio_number = -1;

#else

PIO tft_pio;
int8_t tft_pio_sm;
uint32_t tft_pio_pull_stall_mask;

#define PIO_NB      tft_pio 
#define PIO_SM      tft_pio_sm 
#define PIO_PSP     tft_pio_pull_stall_mask

#endif


// Wait for the PIO to stall (SM pull request finds no data in TX FIFO)
// This is used to detect when the SM is idle and hence ready for a jump instruction
#define WAIT_FOR_STALL  PIO_NB->fdebug = PIO_PSP; while (!(PIO_NB->fdebug & PIO_PSP))

// Wait until at least "S" locations free
#define WAIT_FOR_FIFO_FREE(S) while (((PIO_NB->flevel >> (PIO_SM * 8)) & 0x000F) > (8-S)){}

// The write register of the TX FIFO.
#define TX_FIFO  PIO_NB->txf[PIO_SM]

// Note: the following macros do not wait for the end of transmission
#ifdef PIO_16_to_24

#define RGB_565_to_666(C)  (C)
// convertion is made by the second pio, and write in 3 bytes
#define tft_Write_16(C)     WAIT_FOR_FIFO_FREE(1); TX_FIFO = (C)
#else
#define RGB_565_to_666(C)   ((((uint32_t)(C) & 0xF800)<<8) | (((C) & 0x07E0)<<5) | (((C) & 0x001F)<<3))
// Convert 16-bit colour to 18-bit and write in 3 bytes
#define tft_Write_16(C)     WAIT_FOR_FIFO_FREE(1); TX_FIFO = RGB_565_to_666(C)
#endif

// reconfigure GPIO for PIO0  
#define USE_PIO0 {\
    io_bank0_hw->io[TFT_DC].ctrl = GPIO_FUNC_PIO0 << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;\
    io_bank0_hw->io[TFT_SCLK].ctrl = GPIO_FUNC_PIO0 << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;\
    io_bank0_hw->io[TFT_MOSI].ctrl = GPIO_FUNC_PIO0 << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;\
    }

// reconfigure GPIO for PIO1  
#define USE_PIO1 {\
    io_bank0_hw->io[TFT_DC].ctrl = GPIO_FUNC_PIO1 << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;\
    io_bank0_hw->io[TFT_SCLK].ctrl = GPIO_FUNC_PIO1 << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;\
    io_bank0_hw->io[TFT_MOSI].ctrl = GPIO_FUNC_PIO1 << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;\
    }

/*
#define tft_Write_16N(C)    WAIT_FOR_FIFO_FREE(1); TX_FIFO = (C)
#define tft_Write_16S(C)    WAIT_FOR_FIFO_FREE(1); TX_FIFO = ((C)<<8) | ((C)>>8)
#define tft_Write_32(C)     WAIT_FOR_FIFO_FREE(2); TX_FIFO = ((C)>>16); TX_FIFO = (C)
#define tft_Write_32C(C,D)  WAIT_FOR_FIFO_FREE(2); TX_FIFO = (C); TX_FIFO = (D)
#define tft_Write_32D(C)    WAIT_FOR_FIFO_FREE(2); TX_FIFO = (C); TX_FIFO = (C)
*/



void begin_tft_write(void) {
    CS_L;
}

void end_tft_write(void) {
    // wait TXFIFO is empty
    WAIT_FOR_STALL;
    CS_H;
}


void tft_Write_8(uint8_t data) {
#ifdef PIO_16_to_24    
    pio_number = 0;
    USE_PIO0;
#endif
    uint8_t pio_sm = PIO_SM;

    PIO_NB->sm[pio_sm].instr = pio_instr_jmp8;
    TX_FIFO = data;
    WAIT_FOR_STALL;
}

void writecommand(uint16_t c) {    
    begin_tft_write();
    //pio_number = 0;
    uint8_t pio_sm = PIO_SM;
    DC_C;
    tft_Write_8(c);
    DC_D;
    end_tft_write();
}


void writedata(uint8_t d) {
    begin_tft_write();
    //pio_number = 0;
    uint8_t pio_sm = PIO_SM;
    DC_D;        // Play safe, but should already be in data mode
    tft_Write_8(d);
    //CS_L;        // Allow more hold time for low VDI rail
    end_tft_write();
}


// Code will try both pio's to find a free SM
uint8_t pio_check_available_pio(uint8_t pio_number, const pio_program_t *program) {

    // Find enough free space on one of the PIO's
    PIO_NB = pio0;
    if (!pio_can_add_program(PIO_NB, program)) {
        PIO_NB = pio1;
        if (!pio_can_add_program(PIO_NB, program)) {
            // Serial.println("No room for PIO program!");
            return false;
        }
    }
    return true;
}



void pio_init(uint8_t _pio_number, const pio_program_t *program, uint16_t clock_div) {
#ifdef PIO_16_to_24  
    pio_number = _pio_number;
#else
    _pio_number = _pio_number; // suppress warning
#endif
    // pioinit will claim a free one
    PIO_SM = pio_claim_unused_sm(PIO_NB, false);

    // Load the PIO program
    int _program_offset = pio_add_program(PIO_NB, program);

    // Associate pins with the PIO
    pio_gpio_init(PIO_NB, TFT_DC);
    pio_gpio_init(PIO_NB, TFT_SCLK);
    pio_gpio_init(PIO_NB, TFT_MOSI);

    // Configure the pins to be outputs
    pio_sm_set_consecutive_pindirs(PIO_NB, PIO_SM, TFT_DC, 1, true);
    pio_sm_set_consecutive_pindirs(PIO_NB, PIO_SM, TFT_SCLK, 1, true);
    pio_sm_set_consecutive_pindirs(PIO_NB, PIO_SM, TFT_MOSI, 1, true);

    pio_sm_config c;
    // Configure the state machine
#ifdef PIO_16_to_24    
    if (pio_number == 0) { 
#endif        
        c = pio_test1_program_get_default_config(_program_offset);
#ifdef PIO_16_to_24           
    } else {
        c = pio_send_RGB565_to_24bits_program_get_default_config(_program_offset);
    }
#endif
    sm_config_set_set_pins(&c, TFT_DC, 1);
    // Define the single side-set pin
    sm_config_set_sideset_pins(&c, TFT_SCLK);
    // Define the pin used for data output
    sm_config_set_out_pins(&c, TFT_MOSI, 1);

    // Set clock division
    sm_config_set_clkdiv(&c, clock_div);

    // Make a single 8 words FIFO from the 4 words TX and RX FIFOs
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    // The OSR register shifts to the left, sm designed to send MS byte of a colour first, autopull off
    sm_config_set_out_shift(&c, false, false, 0);

    // Create the pull stall bit mask
    PIO_PSP = 1u << (PIO_FDEBUG_TXSTALL_LSB + PIO_SM);

#ifdef PIO_16_to_24  
    if (pio_number == 0) {
        // pio_test1 program
#endif
        // Now load the configuration
        pio_sm_init(PIO_NB, PIO_SM, _program_offset + pio_test1_offset_start_tx, &c);

        // Create the assembler instruction for the jump to byte send routine
        pio_instr_jmp8  = pio_encode_jmp(_program_offset + pio_test1_offset_start_8);
        pio_instr_fill  = pio_encode_jmp(_program_offset + pio_test1_offset_block_fill);
        pio_instr_addr  = pio_encode_jmp(_program_offset + pio_test1_offset_set_addr_window);

        pio_instr_set_dc = pio_encode_set(pio_pins, 1);
        pio_instr_clr_dc = pio_encode_set(pio_pins, 0);

#ifdef PIO_16_to_24          
    } else {
        // pio_test2 program

        // Now load the configuration
        pio_sm_init(PIO_NB, PIO_SM, _program_offset + pio_send_RGB565_to_24bits_offset_start, &c);
    }
#endif
    // Start the state machine.
    pio_sm_set_enabled(PIO_NB, PIO_SM, true);
   
}

#ifdef DMA_16_to_24   

int32_t            dma_tx_channel;
dma_channel_config dma_tx_config;
static bool DMA_Enabled = false;

void dma_handler(void);


bool init_DMA(void) {

    uint8_t pio_number = 1;
    uint8_t pio_sm = PIO_SM;

    if (DMA_Enabled) 
        return false;

    dma_tx_channel = dma_claim_unused_channel(false);

    if (dma_tx_channel < 0) 
        return false;

    dma_tx_config = dma_channel_get_default_config(dma_tx_channel);

    channel_config_set_transfer_data_size(&dma_tx_config, DMA_SIZE_16);

    channel_config_set_dreq(&dma_tx_config, pio_get_dreq(PIO_NB, pio_sm, true));

    // Tell the DMA to raise IRQ line 0 when the channel finishes a block
    dma_channel_set_irq0_enabled(dma_tx_channel, true);

    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);

    irq_set_enabled(DMA_IRQ_0, true);

    DMA_Enabled = true;
    return true;
}

void dmaWait(void) {
    while (dma_channel_is_busy(dma_tx_channel));
}
#endif  /* DMA_16_to_24 */

void pioinit2(uint32_t clock_freq) {

    // Set clock divider, frequency is set up to 2% faster than specified, or next division down
    uint16_t clock_div = 0.98 + clock_get_hz(clk_sys) / (clock_freq * 2.0); // 2 cycles per bit


    if (pio_check_available_pio(0, &pio_test1_program) == false) {
        return;
    }
    pio_init(0, &pio_test1_program, clock_div);

#ifdef PIO_16_to_24  
    if (pio_check_available_pio(1, &pio_send_RGB565_to_24bits_program) == false) {
        return;
    }    
    pio_init(1, &pio_send_RGB565_to_24bits_program, clock_div);
#ifdef DMA_16_to_24  
    init_DMA();
#endif
#endif
}

void _setWindow(int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
#ifdef PIO_16_to_24    
    pio_number = 0;
    USE_PIO0;
#endif
    uint8_t pio_sm = PIO_SM;

    // GPIO reconfiguration is made in dma_handler() when dma transfer completed 

    PIO_NB->sm[pio_sm].instr = pio_instr_addr;
    register uint32_t x0x1 = (x0<<16) | x1; 
    register uint32_t y0y1 = (y0<<16) | y1;     

    WAIT_FOR_FIFO_FREE(5);
    TX_FIFO = TFT_CASET;    
    TX_FIFO = x0x1; 
    TX_FIFO = TFT_PASET; 
    TX_FIFO = y0y1;  
    TX_FIFO = TFT_RAMWR;     
    
    WAIT_FOR_STALL;
    DC_D;
}



    
#if defined(DMA_16_to_24) && defined(PIO_16_to_24)

static volatile bool _dma_completed = false;

void dma_handler(void) {
    dma_hw->ints0 = 1u << dma_tx_channel;

    _dma_completed = true;

    end_tft_write();

    // reconfigure GPIO for PIO0, ready to recieve new command
    USE_PIO0;
}

void pushPixelsDMA(uint16_t *_image, uint32_t _len) {

    if ((_len == 0) || (!DMA_Enabled)) 
        return;

    dmaWait();

    _dma_completed = false;
   
    pio_number = 1;
    uint8_t pio_sm = PIO_SM;    
    // reconfigure GPIO for PIO1     
    USE_PIO1;

    DC_D;
    channel_config_set_read_increment(&dma_tx_config, true);    
    dma_channel_configure(dma_tx_channel, &dma_tx_config, &PIO_NB->txf[pio_sm], _image, _len, true);
}

void pushBlockDMA(uint16_t _color, uint32_t _len) {
    static uint16_t color;

    if ((_len == 0) || (!DMA_Enabled)) 
        return;

    dmaWait();
    color = _color;
    _dma_completed = false;
    pio_number = 1;
    uint8_t pio_sm = PIO_SM;    
    // reconfigure GPIO for PIO1     
    USE_PIO1;
    DC_D;

    channel_config_set_read_increment(&dma_tx_config, false);
    dma_channel_configure(dma_tx_channel, &dma_tx_config, &PIO_NB->txf[pio_sm], &color, _len, true);
}

#else


/***************************************************************************************
** Function name:           pushPixels - for RP2040 and 3 byte RGB display
** Description:             Write a sequence of pixels
***************************************************************************************/
void pushPixels(const void* data_in, uint32_t len) {
#ifdef PIO_16_to_24  
    pio_number = 1;
    uint8_t pio_sm = PIO_SM;
    USE_PIO1;
    DC_D;
#endif
    uint16_t *data = (uint16_t*)data_in;
    //if (_swapBytes) {
        while ( len-- ) {
            uint32_t col = *data++;
            tft_Write_16(col);
        }
    /*} else {
        while ( len-- ) {
            uint32_t col = *data++;
            tft_Write_16S(col);
        }
    }*/
}

void pushBlock(uint16_t _color, uint32_t _len) {

#ifdef PIO_16_to_24  
    pio_number = 1;
    uint8_t pio_sm = PIO_SM;
    USE_PIO1;
    DC_D;
#endif        
    while (_len > 4) {
        // 5 seems to be the optimum for maximum transfer rate
        WAIT_FOR_FIFO_FREE(5);
        TX_FIFO = RGB_565_to_666(_color);
        TX_FIFO = RGB_565_to_666(_color);
        TX_FIFO = RGB_565_to_666(_color);
        TX_FIFO = RGB_565_to_666(_color);
        TX_FIFO = RGB_565_to_666(_color);

        _len -= 5;
    }

    if (_len) {
        // There could be a maximum of 4 words left  to send
        WAIT_FOR_FIFO_FREE(_len);
        while (_len--) {
            TX_FIFO = RGB_565_to_666(_color);
        }
    }
}



#endif /* DMA_16_to_24 && PIO_16_to_24 */


#include "img_demo1.h"
#include "img_demo2.h"

const uint16_t colors[] = {
    TFT_BLACK,
    TFT_NAVY,
    TFT_DARKGREEN,
    TFT_DARKCYAN,
    TFT_MAROON,
    TFT_PURPLE,
    TFT_OLIVE,
    TFT_LIGHTGREY,
    TFT_DARKGREY,
    TFT_BLUE,
    TFT_GREEN,
    TFT_CYAN,
    TFT_RED,
    TFT_MAGENTA,
    TFT_YELLOW,
    TFT_WHITE,
    TFT_ORANGE,
    TFT_GREENYELLOW,
    TFT_PINK, //Lighter pink, was 0xFC9F
    TFT_BROWN,
    TFT_GOLD,
    TFT_SILVER,
    TFT_SKYBLUE,
    TFT_VIOLET
};

#define COLOR_NUMBER sizeof(colors)/sizeof(uint16_t)


void drawRectThickness(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t t, uint16_t color) {
    typedef struct {
        uint16_t x0;
        uint16_t y0;
        uint16_t x1;
        uint16_t y1;
        uint16_t len;
    } Coord_def;

    Coord_def coor[4] = {
        {x,             y,              x+w-1,          y+t-1,          w*t},
        {x,             y+t,            x+t-1,          y+h-t-1,        (h-2*t)*t},
        {x,             y+h-t,          x+w-1,          y+h-1,          w*t},
        {x+w-t,         y+t,            x+w-1,          y+h-t-1,        (h-2*t)*t}
    };
    
    
    uint8_t i;
    for (i=0;i<4;i++) {
        /* pushBlock test */
        begin_tft_write();
        _setWindow(coor[i].x0, coor[i].y0, coor[i].x1, coor[i].y1);

#if defined(DMA_16_to_24) && defined(PIO_16_to_24) 
        pushBlockDMA(color, coor[i].len);
        while (!_dma_completed) {
            tight_loop_contents();
        }
#else
        pushBlock(color, coor[i].len);
        end_tft_write();
#endif /* DMA_16_to_24 */   

    }
}


/**
 * Demo using PIO[0/1] 16->24 bits if PIO_16_to_24 is defined
 * or using DMA -> PIO[0/1] 16->24 bits if DMA_16_to_24 is defined
 * otherwise using PIO[0] as orginal version with software 16->24 bits conversion
 */
void test_pio_16_24(void) {
    uint16_t x = 0;
    uint16_t y = 0;  
    uint16_t w = 480;
    uint16_t h = 320;  
    uint16_t *image;
    uint32_t len;
    static uint16_t ix_color = 0;      
    static bool flip = false;

    if (ix_color == 0) {
        if (flip) {
            image = (uint16_t *)image2;
            len = sizeof(image2) / sizeof(uint16_t);
            flip = false;
        } else {
            image = (uint16_t *)image1;
            len = sizeof(image1) / sizeof(uint16_t);
            flip = true;
        }


        /* pushPixels test */    
        begin_tft_write();  
        _setWindow(x, y, x + w - 1, y + h - 1);

#if defined(DMA_16_to_24) && defined(PIO_16_to_24)
        pushPixelsDMA((uint16_t *)image, len);
        while (!_dma_completed) {
            // you can do something while the DMA is running
            tight_loop_contents();
        }
#else
        pushPixels((uint16_t *)image, len);
        end_tft_write();
#endif /* DMA_16_to_24 */    
    }

    x = 20;
    y = 20;  
    w = 480-40;
    h = 320-40; 
  
    uint16_t color = colors[ix_color];
    if (++ix_color>COLOR_NUMBER) {
        ix_color = 0;
    }

    drawRectThickness(x, y, w, h, 10, color);

}


