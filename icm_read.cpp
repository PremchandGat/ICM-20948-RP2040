/*
    Written By Premchand Gat
    Github: PremchandGat
    Email: Premchandg278@gmail.com
*/

#include "pico/stdlib.h"
#include "hardware/structs/spi.h"
#include "hardware/regs/dreq.h"
#include "hardware/irq.h"
#include "pico/binary_info.h"
#include "pico.h"
#include "hardware/spi.h"
#include <stdio.h>
#include <string.h>
#include "./src/icm20948.cpp"

// define Pins
#define SCLK 2
#define SDA 3
#define SDI 4
#define CS 5

ICM20948 icm20948(CS,spi0);

void initiaLizeSPI()
{
    printf("Initializing SPI");
    spi_init(spi0, 1000000); // 1MHz is the max speed of the ICM-20948
    spi_set_slave(spi0, false);
    gpio_set_function(SDI, GPIO_FUNC_SPI);
    gpio_set_function(SCLK, GPIO_FUNC_SPI);
    gpio_set_function(SDA, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(SDI, SCLK, SDA, GPIO_FUNC_SPI));
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(CS);
    gpio_set_dir(CS, GPIO_OUT);
    gpio_put(CS, true);
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(CS, "SPI CS"));
    // spi_set_format(spi0, 8,SSPCLKOUT,SSPCLKOUT,SPI_MSB_FIRST);
}

int main()
{
    stdio_init_all();
    initiaLizeSPI();
    icm20948.powerOnICM();
    // Switch to user bank 0
    icm20948.whoAmI();
    while (true)
    {
        icm20948.readAccelAndGyro();
        icm20948.printValues();
        sleep_ms(1000);
    }
    return 0;
}