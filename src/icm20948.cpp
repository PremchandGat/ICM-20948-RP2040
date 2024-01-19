#include "icm20948.h"

/*  0x80 in Binary is 10000000
    we can use OR operation with 0x80 and Address value to get SPI Address Format
    Last 7 bit indicates address of register and first bit shows Read or Write Address
*/

ICM20948::ICM20948(int cs,spi_inst_t * spiId){
    _cs = cs;
    _spiId = spiId;
}
void ICM20948::chipDeselect()
{
    gpio_put(_cs, true);
}

void ICM20948::chipSelect()
{
    gpio_put(_cs, false);
}
// sudo openocd -f interface/cmsis-dap.cfg -c "adapter speed 5000" -f target/rp2040.cfg -s tcl

void ICM20948::read_registers(uint8_t reg, uint8_t *buf, uint16_t len)
{
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.
    uint8_t read = reg | READ_REG;
    chipSelect();
    spi_write_blocking(spi0, &read, 1);
    sleep_ms(10);
    spi_read_blocking(spi0, 0, buf, len);
    chipDeselect();
}

 void ICM20948::write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buffer[2] = {reg, data};
    chipSelect();
    spi_write_blocking(spi0, &reg, 1);
    sleep_ms(10);
    spi_write_blocking(spi0, &data, 1);
    chipDeselect();
    sleep_ms(10);
    // readAndPrint(reg);
}

void ICM20948::whoAmI()
{
    uint8_t buf[1];
    read_registers(0x00, buf, 1);
    printf("\n");
    printf("Who am i: %x", buf[0]);
}

void ICM20948::scaleValues()
{
    float accel_scale = 1.0;
    float gyro_scale = 1.0;

    /*  Gyro Sensitivity Factor
        GYRO_FS_SEL=0		131  LSB/(dps)
        GYRO_FS_SEL=1		65.5 LSB/(dps)
        GYRO_FS_SEL=2		32.8 LSB/(dps)
        GYRO_FS_SEL=3		16.4 LSB/(dps)
    */
    if (_gyro_range == GYRO_RANGE_250_DPS)
        gyro_scale = 131.0;
    if (_gyro_range == GYRO_RANGE_500_DPS)
        gyro_scale = 65.5;
    if (_gyro_range == GYRO_RANGE_1000_DPS)
        gyro_scale = 32.8;
    if (_gyro_range == GYRO_RANGE_2000_DPS)
        gyro_scale = 16.4;

    /*  ACCELEROMETER Sensitivity Scale Factor
        ACCEL_FS=0      16384 LSB/g
        ACCEL_FS=1      8192  LSB/g
        ACCEL_FS=2      4096  LSB/g
        ACCEL_FS=3      2048  LSB/g
    */
    if (_accel_range == ACCEL_RANGE_2_G)
        accel_scale = 16384.0;
    if (_accel_range == ACCEL_RANGE_4_G)
        accel_scale = 8192.0;
    if (_accel_range == ACCEL_RANGE_8_G)
        accel_scale = 4096.0;
    if (_accel_range == ACCEL_RANGE_16_G)
        accel_scale = 2048.0;

    gyroX = (_rawGyroX / gyro_scale) * SENSORS_DPS_TO_RADS;
    gyroY = (_rawGyroY / gyro_scale) * SENSORS_DPS_TO_RADS;
    gyroZ = (_rawGyroZ / gyro_scale) * SENSORS_DPS_TO_RADS;

    accX = (_rawAccX / accel_scale) * SENSORS_GRAVITY_EARTH;
    accY = (_rawAccY / accel_scale) * SENSORS_GRAVITY_EARTH;
    accZ = (_rawAccZ / accel_scale) * SENSORS_GRAVITY_EARTH;
}

void ICM20948::readAccelAndGyro()
{
    printf("Getting Data: \n\n");
    uint8_t buffer[12];
    read_registers(ACCEL_START_REG, buffer, 12);
    _rawAccX = buffer[0] << 8 | buffer[1];
    _rawAccY = buffer[2] << 8 | buffer[3];
    _rawAccZ = buffer[4] << 8 | buffer[5];

    _rawGyroX = buffer[6] << 8 | buffer[7];
    _rawGyroY = buffer[8] << 8 | buffer[9];
    _rawGyroZ = buffer[10] << 8 | buffer[11];
    scaleValues();
}

void ICM20948::printValues()
{
    printf("Acceleration    X: %f Y: %f Z: %f\n", accX, accY, accZ);
    printf("Gyro            X: %f Y: %f Z: %f\n", gyroX, gyroY, gyroZ);
}

void ICM20948::configureGyro()
{
    write_reg(REG_BANK_SEL, 0x20);
    sleep_ms(20);
    uint8_t gyro_fs_sel = 0x0;
    if (_gyro_range == GYRO_RANGE_250_DPS)
    {
        gyro_fs_sel = GYRO_250;
    }
    else if (_gyro_range == GYRO_RANGE_500_DPS)
    {
        gyro_fs_sel = GYRO_500;
    }
    else if (_gyro_range == GYRO_RANGE_1000_DPS)
    {
        gyro_fs_sel = GYRO_1000;
    }
    else if (_gyro_range == GYRO_RANGE_2000_DPS)
    {
        gyro_fs_sel = GYRO_2000;
    }
    else
    {
        gyro_fs_sel = GYRO_250;
    }
    write_reg(GYRO_CONFIG_1, (gyro_fs_sel | _gyro_filter | GYRO_FCHOICE)); // Configure Gyro
    sleep_ms(10);
    // Set gyroscope sample rate to 100hz (0x0A) in GYRO_SMPLRT_DIV register (0x00)
    write_reg(GYRO_SMPLRT_DIV, _gyro_sampling_rate_div);
    sleep_ms(10);
}

void ICM20948::configureAccel()
{
    write_reg(REG_BANK_SEL, 0x20);
    sleep_ms(20);
    uint8_t accel_fs_sel = 0x0;
    if (_accel_range == ACCEL_RANGE_2_G)
    {
        accel_fs_sel = ACCEL_2G;
    }
    else if (_accel_range == ACCEL_RANGE_4_G)
    {
        accel_fs_sel = ACCEL_4G;
    }
    else if (_accel_range == ACCEL_RANGE_8_G)
    {
        accel_fs_sel = ACCEL_8G;
    }
    else if (_accel_range == ACCEL_RANGE_16_G)
    {
        accel_fs_sel = ACCEL_16G;
    }
    else
    {
        accel_fs_sel = ACCEL_2G;
    }
    // Configure Accelerometer
    write_reg(ACCEL_CONFIG, (accel_fs_sel | _accel_filter | ACCEL_FCHOICE)); // Configure Gyro
    sleep_ms(10);
    // Set accelerometer sample rate
    write_reg(ACCEL_SMPLRT_DIV_1, _accel_sampling_rate_div_1);
    sleep_ms(10);
    write_reg(ACCEL_SMPLRT_DIV_2, _accel_sampling_rate_div_2);
    sleep_ms(10);
}

void ICM20948::powerOnICM()
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
    write_reg(REG_BANK_SEL, USER_BANK_0);
}