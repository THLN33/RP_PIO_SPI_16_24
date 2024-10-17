#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"

#include "def.h"




int main() {
    stdio_init_all();
/*
    if (!set_sys_clock_khz(250000, false))
        printf("system clock 250MHz failed\n");
*/
    ILI9488_Init();


    while(true) {
        
        test_pio_16_24();

        printf("Ok\n");
        sleep_ms(100);
    }

}