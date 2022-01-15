#include "uart.h"
#include "twi.h"
#include "MPU9250.h"

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

typedef struct USART_RXC_cb_ctx_t {
    uint8_t rx;
    int cnt;
} USART_RXC_cb_ctx;

/**
 * USART RX interrupt callback handle
 * @param[inout] ctx user data for interrupt callback
 * When ISR occurs USART_RXC_cb will be called with ctx as parameter
 * UART RX data (UDR) should be saved in this function
 */
static void USART_RXC_cb_handle(void* ctx) {
    USART_RXC_cb_ctx* t_ctx = (USART_RXC_cb_ctx*)ctx;

    t_ctx->rx = UDR;
    t_ctx->cnt++;
}

typedef struct TWI_cb_ctx_t {
    int cnt;
}TWI_cb_ctx;

/**
 * TWI interrupt callback handle
 * @param[inout] ctx user data for interrupt callback
 */
static void TWI_cb_handle(void* ctx) {
    TWI_cb_ctx* t_ctx = (TWI_cb_ctx*)ctx;

    t_ctx->cnt++;
}

int main(void) {

    // UART INIT
    //-------------------------------
    const uint16_t baud_rate = 38400;

    uint8_t sts = 0;
    struct USART_RXC_cb_ctx_t USART_RXC_ctx = {};

    regiter_USART_RXC_cb(USART_RXC_cb_handle, &USART_RXC_ctx);

    USART_init(baud_rate);

    printf("Init Done UART baud: %u\n", (uint16_t)baud_rate);

    // TWI init
    //-------------------------------
    uint8_t slave_addr;
    TWI_cb_ctx twi_ctx = {};
    regiter_TWI_isr_cb(TWI_cb_handle, &twi_ctx);

    TWI_init(TWI_PS_1, 2);

    printf("TWI init done\n");

    sts = TWI_sniff(0x00, 0x7F, &slave_addr);
    if (sts == 0xFF) printf("TWI error\n");

    printf("TWI sniff: slaves:%d fist slave addr:0x%02x\n", sts, slave_addr);
    printf("total number of TWI interrupts: %d\n", twi_ctx.cnt);

    // MPU9250 init
    //-------------------------------
    register_MPU_cb(TWI_write_reg, TWI_read_reg_burst);

    MPU9250_calib();

    sts = MPU9250_init();
    if (sts) return sts;

    printf("MPU9250 init done\n");

    MPU9250_data mpu_data = {};

    int16_t min_mag[3] = {0};
    int16_t max_mag[3] = {0};

    printf("MPU9250 getting magnetometer offsets...\n");

    while (!USART_RXC_ctx.cnt) {
        MPU9250_get_data(&mpu_data);

        if (mpu_data.mag[0] < min_mag[0]) min_mag[0] = mpu_data.mag[0];
        if (mpu_data.mag[0] > max_mag[0]) max_mag[0] = mpu_data.mag[0];
        if (mpu_data.mag[1] < min_mag[1]) min_mag[1] = mpu_data.mag[1];
        if (mpu_data.mag[1] > max_mag[1]) max_mag[1] = mpu_data.mag[1];
        if (mpu_data.mag[2] < min_mag[2]) min_mag[2] = mpu_data.mag[2];
        if (mpu_data.mag[2] > max_mag[2]) max_mag[2] = mpu_data.mag[2];
    }

    printf("calib: %d %d %d %d %d %d\n", min_mag[0], max_mag[0], min_mag[1], max_mag[1], min_mag[2], max_mag[2]);

    while(1) {
        MPU9250_get_data(&mpu_data);
        // printf("%05d\n", mpu_data.tmp);
        printf("%05d, %05d, %05d\n", mpu_data.acc[0], mpu_data.acc[1], mpu_data.acc[2]);
        // printf("%05d, %05d, %05d\n", mpu_data.gyro[0], mpu_data.gyro[1], mpu_data.gyro[2]);
        // printf("%05d, %05d, %05d\n", mpu_data.mag[0] - (min_mag[0] + max_mag[0]) / 2, mpu_data.mag[1] -  (min_mag[1] + max_mag[1]) / 2, mpu_data.mag[2] - (min_mag[2] + max_mag[2]) / 2);

        _delay_ms(100);
    }

    return sts;
}
