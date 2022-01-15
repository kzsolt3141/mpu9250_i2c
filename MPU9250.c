#include "MPU9250.h"

#include <stddef.h>
#include <util/delay.h>

static TWI_write_reg_cb_t    write_reg      = NULL;
static TWI_read_reg_burst_cb read_reg_burst = NULL;
static uint8_t mag_bias[3]                  = {0};

void register_MPU_cb(TWI_write_reg_cb_t write, TWI_read_reg_burst_cb read) {
    if (!write || !read) return;

    write_reg = write;
    read_reg_burst = read;
}

uint8_t MPU9250_init() {
    uint8_t buf = 0;
    if (!write_reg || ! read_reg_burst) return 1;

    read_reg_burst(&buf, MPU9250_I2C_ADDRESS, MPU9250_WHO_AM_I, 1);  // should return 0x73
    if (0x73 != buf) return buf;

    write_reg(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_1,    0x00);  // Clear sleep mode bit (6)
    _delay_ms(100);
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_1,    0x01);  // disable reset condition, and select internal clock
    _delay_ms(100);

    write_reg(MPU9250_I2C_ADDRESS, MPU9250_CONFIG,        0x03);  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
                                                                                             // minimum delay time for this setting is 5.9 ms,
                                                                                             // which means sensor fusion update rates cannot
                                                                                             // be higher than 1 / 0.0059 = 170 Hz; sample rate is 1kHz
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_SMPLRT_DIV,    0x04);  // set sample rate to 1/(1+4) = 200 Hz

    write_reg(MPU9250_I2C_ADDRESS, MPU9250_GYRO_CONFIG,   0x00);  // set gyro range to 250dps

    write_reg(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG,  0x00);  // set accelero range 2g
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG2, 0x03);  // set accelero low pass filter DLPCFG 44.8 Hz, 4.88 ms, sample rate 1kHz

    write_reg(MPU9250_I2C_ADDRESS, MPU9250_USER_CTRL,     0x00);  // i2c master disable
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_INT_PIN_CFG,   0x02);  // bypass I2C to magnetometer

    read_reg_burst(&buf, AK8963_DEFAULT_ADDRESS, AK8963_RA_WIA, 1);  // should return 0x48
    if (AK8963_WIA_MASK != buf) return buf;

    write_reg(AK8963_DEFAULT_ADDRESS, AK8963_RA_CNTL1,    0x0F);  // rom fuse access

    read_reg_burst(mag_bias, AK8963_DEFAULT_ADDRESS, AK8963_RA_ASAX, 3);

    write_reg(AK8963_DEFAULT_ADDRESS, AK8963_RA_CNTL1,    0x16);  // 16 bit values with continiuos measurement 100Hz

    write_reg(MPU9250_I2C_ADDRESS, MPU9250_USER_CTRL,     0x20);  // i2c master enable

    write_reg(MPU9250_I2C_ADDRESS, MPU9250_INT_PIN_CFG,   0x00);  // bypass I2C disabled
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_I2C_MST_CTRL,  0x0D);  // i2c master clock select

    write_reg(MPU9250_I2C_ADDRESS, MPU9250_I2C_SLV0_CTRL, 0x00);  // disable slave
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_I2C_SLV0_ADDR, AK8963_DEFAULT_ADDRESS | 0x80);  // set first slave to magnetometer
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_I2C_SLV0_REG,  AK8963_RA_HXL);  // start reading magneto from this addr
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_I2C_SLV0_CTRL, 0x87);  // read transfer, 7 registers

    return 0;
}

void MPU9250_get_data(MPU9250_data* data) {
    uint8_t buf[21];

    read_reg_burst(buf, MPU9250_I2C_ADDRESS, MPU9250_ACCEL_XOUT_H, 21);

    data->acc[0] = (uint16_t)(buf[0] << 8) + buf[1];
    data->acc[1] = (uint16_t)(buf[2] << 8) + buf[3];
    data->acc[2] = (uint16_t)(buf[4] << 8) + buf[5];

    data->tmp = ((uint16_t)(buf[6] << 8) + buf[7] + 5014) / 334;  // tmp / 334 -5 + 21

    data->gyro[0] = (uint16_t)(buf[8] << 8) + buf[9];
    data->gyro[1] = (uint16_t)(buf[10] << 8) + buf[11];
    data->gyro[2] = (uint16_t)(buf[12] << 8) + buf[13];

    data->mag[0] = (uint16_t)(buf[15] << 8) + buf[14];
    data->mag[1] = (uint16_t)(buf[17] << 8) + buf[16];
    data->mag[2] = (uint16_t)(buf[19] << 8) + buf[18];

    data->mag[0] = data->mag[0] +  data->mag[0] * (mag_bias[0] - 128) / 256;
    data->mag[1] = data->mag[1] +  data->mag[1] * (mag_bias[1] - 128) / 256;
    data->mag[2] = data->mag[2] +  data->mag[2] * (mag_bias[2] - 128) / 256;

    ////processing...
    ////angles accelero
    //rollAngle = atan2(Ay,Az)*R2D;
    //pitchAngle = atan2((double)-Ax, sqrt((double)Ay*(double)Ay + (double)Az*(double)Az))*R2D;
    ////temperature mpu9250
    //realTemp = (T - 21)/333.87 + 21; // ??
    ////magnetom processing
    //p = ((float)aux.mag[0] - 202.0) / (-4.9);
    //r = ((float)aux.mag[1] - 310.0) / (-4.74444);
    //y = ((float)aux.mag[2] - 74) / (-5.53333);
}

void MPU9250_calib() {
    int32_t gyro_bias[3] = {0};
    MPU9250_data mpu_data = {};

    write_reg(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_1,    0x80);  // Write a one to bit 7 reset bit; toggle reset device
    _delay_ms(100);

    write_reg(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_2,    0x00);  // enable all sensors
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE,    0x00);  // Disable all interrupts
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_FIFO_EN,       0x00);  // Disable FIFO
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_1,    0x00);  // Turn on internal clock source
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_I2C_MST_CTRL,  0x00);  // Disable I2C master
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_USER_CTRL,     0x00);  // Disable FIFO and I2C master modes
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_USER_CTRL,     0x0C);  // Reset FIFO and DMP
    _delay_ms(15);

    write_reg(MPU9250_I2C_ADDRESS, MPU9250_CONFIG,       0x01);  // Set low-pass filter to 188 Hz
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_SMPLRT_DIV,   0x00);  // Set sample rate to 1 kHz
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_GYRO_CONFIG,  0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00);  // Set accelerometer full-scale to 2 g, maximum sensitivity

    write_reg(MPU9250_I2C_ADDRESS, MPU9250_USER_CTRL,    0x40);  // Enable FIFO
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_FIFO_EN,      0x78);  // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9250)
    _delay_ms(40);  // accumulate 40 samples in 40 milliseconds = 480 bytes

    write_reg(MPU9250_I2C_ADDRESS, MPU9250_FIFO_EN, 0x00);       // Disable gyro and accelerometer sensors for FIFO

    for (uint8_t i = 0; i < 32; i++) {
        MPU9250_get_data(&mpu_data);
        
        gyro_bias[0]  += (int32_t) mpu_data.gyro[0];
        gyro_bias[1]  += (int32_t) mpu_data.gyro[1];
        gyro_bias[2]  += (int32_t) mpu_data.gyro[2];
    }

    gyro_bias[0]  >>= 5;
    gyro_bias[1]  >>= 5;
    gyro_bias[2]  >>= 5;

    // Push gyro biases to hardware registers
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_XG_OFFSET_H, -gyro_bias[0] >> (8 + 2));
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_XG_OFFSET_L, -gyro_bias[0] >> 2);
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_YG_OFFSET_H, -gyro_bias[1] >> (8 + 2));
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_YG_OFFSET_L, -gyro_bias[1] >> 2);
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_ZG_OFFSET_H, -gyro_bias[2] >> (8 + 2));
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_ZG_OFFSET_L, -gyro_bias[2] >> 2);

    //TODO(Zsolt) calibrate accelero with a level
    uint8_t *buf = (uint8_t*)gyro_bias;
    read_reg_burst(buf, MPU9250_I2C_ADDRESS, MPU9250_XA_OFFSET_H, 2);
    uint16_t acc_bias_reg = ((uint16_t)(buf[0] << 8) | buf[1]) - (0 >> 3);
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_XA_OFFSET_H, acc_bias_reg >> 8);
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_XA_OFFSET_L, acc_bias_reg);

    read_reg_burst(buf, MPU9250_I2C_ADDRESS, MPU9250_YA_OFFSET_H, 2);
    acc_bias_reg = ((uint16_t)(buf[0] << 8) | buf[1]) - (0 >> 3);
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_YA_OFFSET_H, acc_bias_reg >> 8);
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_YA_OFFSET_L, acc_bias_reg);

    read_reg_burst(buf, MPU9250_I2C_ADDRESS, MPU9250_ZA_OFFSET_H, 2);
    acc_bias_reg = ((uint16_t)(buf[0] << 8) | buf[1]) - (1600 >> 3);
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_ZA_OFFSET_H, acc_bias_reg >> 8);
    write_reg(MPU9250_I2C_ADDRESS, MPU9250_ZA_OFFSET_L, acc_bias_reg);
}
