#include "pico/stdlib.h"
#include "hardware/structs/spi.h"
#include "hardware/regs/dreq.h"
#include "hardware/irq.h"
#include "pico/binary_info.h"
#include "pico.h"
#include "hardware/spi.h"
#include <stdio.h>
#include <string.h>
#include "icm-reg.h"

// define Pins
#define SCLK 18
#define SDA 19
#define SDI 16
#define CS 17
/*  0x80 in Binary is 10000000
    we can use OR operation with 0x80 and Address value to get SPI Address Format
    Last 7 bit indicates address of register and first bit shows Read or Write Address
*/

#define READ_REG 0x80
/* Gyro Configuration */
int gyro_range = GYRO_RANGE_250_DPS;
uint8_t gyro_filter = GYRO_250;
/*  1125/(1+GYRO_SMPLRT_DIV)Hz where
    GYRO_SMPLRT_DIV is 0, 1, 2,…255
    For 100HZ sampling rate we 0x10
*/
uint8_t gyro_sampling_rate_div = 0x10;

/* Accel Configuration */
int accel_range = ACCEL_RANGE_2_G;
uint8_t accel_filter = ACCEL_DLPFCFG_68;
/*  1125/(1+ACCEL_SMPLRT_DIV)Hz where
    ACCEL_SMPLRT_DIV is 0, 1, 2,…4095
        ACCEL_SMPLRT_DIV_1
        7:4  Reserved.
        3:0  ACCEL_SMPLRT_DIV[11:8] MSB for ACCEL sample rate div.
        ACCEL_SMPLRT_DIV_2
        7:0 ACCEL_SMPLRT_DIV[7:0]   LSB for ACCEL sample rate div.
                                    ODR is computed as follows:
                                    1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0])
    There are 2, 8 Bit registers to store ACCEL_SMPLRT_DIV  ACCEL_SMPLRT_DIV_1 and ACCEL_SMPLRT_DIV_2
    ACCEL_SMPLRT_DIV_1 stores 0 to 3 MSB bits  (4 Bits)
    ACCEL_SMPLRT_DIV_2 stores 0 to 7 LSB bits  (8 Bits)
    Total 12 bits are stored in both registers

*/
uint8_t accel_sampling_rate_div_1 = 0x00;
uint8_t accel_sampling_rate_div_2 = 0x10;

/*
PIN NUMBER      PIN NAME        PIN DESCRIPTION
9               AD0 / SDO       I2C Slave Address LSB (AD0); SPI serial data output (SDO)
22              nCS             Chip select (SPI mode only)
23              SCL / SCLK      I2C serial clock (SCL); SPI serial clock (SCLK)
24              SDA / SDI       I2C serial data (SDA); SPI serial data input (SDI)

    1. Data is delivered MSB first and LSB last
    2. Data is latched on the rising edge of SCLK
    3. Data should be transitioned on the falling edge of SCLK
    4. The maximum frequency of SCLK is 7MHz
    5. SPI read and write operations are completed in 16 or more clock cycles (two or more bytes). The first byte
    contains the SPI Address, and the following byte(s) contain(s) the SPI data. The first bit of the first byte
    contains the Read/Write bit and indicates the Read (1) or Write (0) operation. The following 7 bits contain the
    Register Address. In cases of multiple-byte Read/Writes, data is two or more bytes

    SPI Address format
    MSB                         LSB
    R/W A6  A5  A4  A3  A2  A1  A0

    SPI Data format
    MSB                         LSB
    D7  D6  D5  D4  D3  D2  D1  D0
*/

/* Vars to store data */
float accX, ///< Last reading's accelerometer X axis m/s^2
    accY,   ///< Last reading's accelerometer Y axis m/s^2
    accZ,   ///< Last reading's accelerometer Z axis m/s^2
    gyroX,  ///< Last reading's gyro X axis in rad/s
    gyroY,  ///< Last reading's gyro Y axis in rad/s
    gyroZ;  ///< Last reading's gyro Z axis in rad/s

int16_t rawAccX, ///< temp variables
    rawAccY,     ///< temp variables
    rawAccZ,     ///< temp variables
    rawTemp,     ///< temp variables
    rawGyroX,    ///< temp variables
    rawGyroY,    ///< temp variables
    rawGyroZ;    ///< temp variables

void cs_deselect()
{
    gpio_put(CS, true);
}

void cs_select()
{
    gpio_put(CS, false);
}
// sudo openocd -f interface/cmsis-dap.cfg -c "adapter speed 5000" -f target/rp2040.cfg -s tcl

static void read_registers(uint8_t reg, uint8_t *buf, uint16_t len)
{
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.
    uint8_t read = reg | READ_REG;
    cs_select();
    spi_write_blocking(spi0, &read, 1);
    sleep_ms(10);
    spi_read_blocking(spi0, 0, buf, len);
    cs_deselect();
}

static void write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buffer[2] = {reg, data};
    cs_select();
    spi_write_blocking(spi0, &reg, 1);
    sleep_ms(10);
    spi_write_blocking(spi0, &data, 1);
    cs_deselect();
    sleep_ms(10);
    // readAndPrint(reg);
}

void whoAmI()
{
    uint8_t buf[1];
    read_registers(0x00, buf, 1);
    printf("\n");
    printf("Who am i: %x", buf[0]);
}

void scaleValues()
{
    float accel_scale = 1.0;
    float gyro_scale = 1.0;

    /*  Gyro Sensitivity Factor
        GYRO_FS_SEL=0		131  LSB/(dps)
        GYRO_FS_SEL=1		65.5 LSB/(dps)
        GYRO_FS_SEL=2		32.8 LSB/(dps)
        GYRO_FS_SEL=3		16.4 LSB/(dps)
    */
    if (gyro_range == GYRO_RANGE_250_DPS)
        gyro_scale = 131.0;
    if (gyro_range == GYRO_RANGE_500_DPS)
        gyro_scale = 65.5;
    if (gyro_range == GYRO_RANGE_1000_DPS)
        gyro_scale = 32.8;
    if (gyro_range == GYRO_RANGE_2000_DPS)
        gyro_scale = 16.4;

    /*  ACCELEROMETER Sensitivity Scale Factor
        ACCEL_FS=0      16384 LSB/g
        ACCEL_FS=1      8192  LSB/g
        ACCEL_FS=2      4096  LSB/g
        ACCEL_FS=3      2048  LSB/g
    */
    if (accel_range == ACCEL_RANGE_2_G)
        accel_scale = 16384.0;
    if (accel_range == ACCEL_RANGE_4_G)
        accel_scale = 8192.0;
    if (accel_range == ACCEL_RANGE_8_G)
        accel_scale = 4096.0;
    if (accel_range == ACCEL_RANGE_16_G)
        accel_scale = 2048.0;

    gyroX = (rawGyroX / gyro_scale) * SENSORS_DPS_TO_RADS;
    gyroY = (rawGyroY / gyro_scale) * SENSORS_DPS_TO_RADS;
    gyroZ = (rawGyroZ / gyro_scale) * SENSORS_DPS_TO_RADS;

    accX = (rawAccX / accel_scale) * SENSORS_GRAVITY_EARTH;
    accY = (rawAccY / accel_scale) * SENSORS_GRAVITY_EARTH;
    accZ = (rawAccZ / accel_scale) * SENSORS_GRAVITY_EARTH;
}

void readAccelAndGyro()
{
    printf("Getting Data: \n\n");
    uint8_t buffer[12];
    read_registers(ACCEL_START_REG, buffer, 12);
    rawAccX = buffer[0] << 8 | buffer[1];
    rawAccY = buffer[2] << 8 | buffer[3];
    rawAccZ = buffer[4] << 8 | buffer[5];

    rawGyroX = buffer[6] << 8 | buffer[7];
    rawGyroY = buffer[8] << 8 | buffer[9];
    rawGyroZ = buffer[10] << 8 | buffer[11];
    scaleValues();
}

void printValues()
{
    printf("Acceleration    X: %f Y: %f Z: %f\n", accX, accY, accZ);
    printf("Gyro            X: %f Y: %f Z: %f\n", gyroX, gyroY, gyroZ);
}

void configureGyro()
{
    write_reg(REG_BANK_SEL, 0x20);
    sleep_ms(20);
    uint8_t gyro_fs_sel = 0x0;
    if (gyro_range == GYRO_RANGE_250_DPS)
    {
        gyro_fs_sel = GYRO_250;
    }
    else if (gyro_range == GYRO_RANGE_500_DPS)
    {
        gyro_fs_sel = GYRO_500;
    }
    else if (gyro_range == GYRO_RANGE_1000_DPS)
    {
        gyro_fs_sel = GYRO_1000;
    }
    else if (gyro_range == GYRO_RANGE_2000_DPS)
    {
        gyro_fs_sel = GYRO_2000;
    }
    else
    {
        gyro_fs_sel = GYRO_250;
    }
    write_reg(GYRO_CONFIG_1, (gyro_fs_sel | gyro_filter | GYRO_FCHOICE)); // Configure Gyro
    sleep_ms(10);
    // Set gyroscope sample rate to 100hz (0x0A) in GYRO_SMPLRT_DIV register (0x00)
    write_reg(GYRO_SMPLRT_DIV, gyro_sampling_rate_div);
    sleep_ms(10);
}

void configureAccel()
{
    write_reg(REG_BANK_SEL, 0x20);
    sleep_ms(20);
    uint8_t accel_fs_sel = 0x0;
    if (accel_range == ACCEL_RANGE_2_G)
    {
        accel_fs_sel = ACCEL_2G;
    }
    else if (accel_range == ACCEL_RANGE_4_G)
    {
        accel_fs_sel = ACCEL_4G;
    }
    else if (accel_range == ACCEL_RANGE_8_G)
    {
        accel_fs_sel = ACCEL_8G;
    }
    else if (accel_range == ACCEL_RANGE_16_G)
    {
        accel_fs_sel = ACCEL_16G;
    }
    else
    {
        accel_fs_sel = ACCEL_2G;
    }
    // Configure Accelerometer
    write_reg(ACCEL_CONFIG, (accel_fs_sel | accel_filter | ACCEL_FCHOICE)); // Configure Gyro
    sleep_ms(10);
    // Set accelerometer sample rate
    write_reg(ACCEL_SMPLRT_DIV_1, accel_sampling_rate_div_1);
    sleep_ms(10);
    write_reg(ACCEL_SMPLRT_DIV_2, accel_sampling_rate_div_2);
    sleep_ms(10);
}

void powerOnICM()
{
    sleep_ms(10);
    write_reg(REG_BANK_SEL, USER_BANK_0); // Switch to user bank 0
    sleep_ms(10);
    write_reg(USER_CTRL, 0x78); // Disable I2C
    sleep_ms(10);
    write_reg(PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready else
    sleep_ms(10);
    write_reg(PWR_MGMT_2, (0x38 | 0x07)); // Turn Off GYRO and Accelerometer
    sleep_ms(20);
    write_reg(PWR_MGMT_2, (0x00 | 0x00)); // Turn On Acce Gyro
    sleep_ms(10);
    configureGyro();
    configureAccel();
}

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
    cs_deselect();
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(CS, "SPI CS"));
    // spi_set_format(spi0, 8,SSPCLKOUT,SSPCLKOUT,SPI_MSB_FIRST);
}

int main()
{
    stdio_init_all();
    initiaLizeSPI();
    powerOnICM();
    // Switch to user bank 0
    write_reg(REG_BANK_SEL, USER_BANK_0);
    whoAmI();
    while (true)
    {
        readAccelAndGyro();
        printValues();
        sleep_ms(1000);
    }
    return 0;
}