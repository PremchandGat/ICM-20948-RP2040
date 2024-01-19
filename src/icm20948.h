#pragma once
#include <stdio.h>
#include "hardware/spi.h"

/*--------------------------------------ICM20948 Register Map-----------------------------------------*/
/*  Hex: 00  Dec: 0   Register Name: WHO_AM_I Type: R     WHO_AM_I[7:0]
    Register to indicate to user which device is being accessed.
    The value for ICM-20948 is 0xEA.
*/
#define WHO_AM_I 0X00

/*  Acceleration
    Values are in 16 bit (2 Bytes)
    Add 1 to get next 8 bit value
    Values can be readed in sequence of X-High, X-Low, Y-High, Y-Low, Z-HIGH, Z-LOW
*/
#define ACCEL_START_REG 0x2d

/*  Gyro
    Values are in 16 bit (2 Bytes)
    Add 1 to get next 8 bit value
    Values can be readed in sequence of X-High, X-Low, Y-High, Y-Low, Z-HIGH, Z-LOW
*/
/*  0x80 in Binary is 10000000
    we can use OR operation with 0x80 and Address value to get SPI Address Format
    Last 7 bit indicates address of register and first bit shows Read or Write Address
*/

#define GYRO_START_REG 0x33

#define GYRO_RATE_250 0x00
#define GYRO_LPF_17HZ 0x29

/**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_EARTH (9.80665F)
/**< Degrees/s to rad/s multiplier */
#define SENSORS_DPS_TO_RADS (0.017453293F)

// USER BANK 0 Register MAP
#define PWR_MGMT_1 0x06 // Device defaults to the SLEEP mode
#define PWR_MGMT_2 0x07
#define INT_PIN_CFG 0x0F
#define INT_ENABLE 0x10
#define INT_ENABLE_1 0x11
#define REG_BANK_SEL 0x7F
#define USER_CTRL 0x03

// USER BANK 2 REGISTER MAP
#define GYRO_SMPLRT_DIV 0x00
#define GYRO_CONFIG_1 0x01
#define TEMP_CONFIG 0x53
#define ACCEL_SMPLRT_DIV_1 0x10
#define ACCEL_SMPLRT_DIV_2 0x11
#define ACCEL_CONFIG 0x14

/* User Banks */
#define USER_BANK_0 0x00
#define USER_BANK_1 0x10
#define USER_BANK_2 0x20
#define USER_BANK_3 0x30

/** The gyro data range */
typedef enum
{
    GYRO_RANGE_250_DPS,
    GYRO_RANGE_500_DPS,
    GYRO_RANGE_1000_DPS,
    GYRO_RANGE_2000_DPS,
} gyro_range_t;

#define GYRO_250 0x00
#define GYRO_500 0x02
#define GYRO_1000 0x04
#define GYRO_2000 0x06
/* Turn GYRO_FCHOICE make 1 */
#define GYRO_FCHOICE 0x01
/* Filter Configurations */
#define GYRO_DLPFCFG_229 0x0
#define GYRO_DLPFCFG_187 0x08
#define GYRO_DLPFCFG_154 0x10
#define GYRO_DLPFCFG_73 0x18
#define GYRO_DLPFCFG_35 0x20
#define GYRO_DLPFCFG_17 0x28
#define GYRO_DLPFCFG_8 0x30
#define GYRO_DLPFCFG_376 0x38

/** The accelerometer data range */
typedef enum
{
    ACCEL_RANGE_2_G,
    ACCEL_RANGE_4_G,
    ACCEL_RANGE_8_G,
    ACCEL_RANGE_16_G,
} accel_range_t;

#define ACCEL_2G 0x00
#define ACCEL_4G 0x02
#define ACCEL_8G 0x04
#define ACCEL_16G 0x06
/* Turn ACCEL_FCHOICE make 1 */
#define ACCEL_FCHOICE 0x01
/* Filter Configurations */
#define ACCEL_DLPFCFG_265 0x0
#define ACCEL_DLPFCFG_265_1 0x08
#define ACCEL_DLPFCFG_136 0x10
#define ACCEL_DLPFCFG_68 0x18
#define ACCEL_DLPFCFG_34 0x20
#define ACCEL_DLPFCFG_17 0x28
#define ACCEL_DLPFCFG_8 0x30
#define ACCEL_DLPFCFG_499 0x38

// enum Ascale
//     {
//       AFS_2G = 0,
//       AFS_4G,
//       AFS_8G,
//       AFS_16G
//     };

#define READ_REG 0x80

/*--------------------------------------------END-------------------------------------*/

class ICM20948
{
public:
    ICM20948(int cs,spi_inst_t * spiId);
    void whoAmI();
    void readAccelAndGyro();
    void powerOnICM();
    void printValues();
    /* Vars to store data */
    float accX, ///< Last reading's accelerometer X axis m/s^2
        accY,   ///< Last reading's accelerometer Y axis m/s^2
        accZ,   ///< Last reading's accelerometer Z axis m/s^2
        gyroX,  ///< Last reading's gyro X axis in rad/s
        gyroY,  ///< Last reading's gyro Y axis in rad/s
        gyroZ;  ///< Last reading's gyro Z axis in rad/s
private:
    void chipDeselect();
    void chipSelect();
    void scaleValues();
    void configureGyro();
    void configureAccel();
    void read_registers(uint8_t reg, uint8_t *buf, uint16_t len);
     void write_reg(uint8_t reg, uint8_t data);
    int _cs;
    spi_inst_t * _spiId;
    uint8_t _Ascale = 0x0;
    int16_t _rawAccX, ///< temp variables
        _rawAccY,     ///< temp variables
        _rawAccZ,     ///< temp variables
        _rawTemp,     ///< temp variables
        _rawGyroX,    ///< temp variables
        _rawGyroY,    ///< temp variables
        _rawGyroZ;    ///< temp variables
                      /* Gyro Configuration */
    int _gyro_range = GYRO_RANGE_250_DPS;
    uint8_t _gyro_filter = GYRO_250;
    /*  1125/(1+GYRO_SMPLRT_DIV)Hz where
        GYRO_SMPLRT_DIV is 0, 1, 2,…255
        For 100HZ sampling rate we 0x10
    */
    uint8_t _gyro_sampling_rate_div = 0x10;

    /* Accel Configuration */
    int _accel_range = ACCEL_RANGE_2_G;
    uint8_t _accel_filter = ACCEL_DLPFCFG_68;
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
    uint8_t _accel_sampling_rate_div_1 = 0x00;
    uint8_t _accel_sampling_rate_div_2 = 0x10;
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
};