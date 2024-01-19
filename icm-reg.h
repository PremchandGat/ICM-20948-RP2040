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

uint8_t Ascale = 0x0;
