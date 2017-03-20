/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example
 * @brief Blinky Example Application main file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "app.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "ble_advertising.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"

const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;


/* Indicates if reading operation from accelerometer has ended. */
static volatile bool m_xfer_done = true;
/* Indicates if setting mode operation has ended. */
static volatile bool m_set_mode_done = false;
/* TWI instance. */
static const nrf_drv_twi_t m_twi_mpu9250 = NRF_DRV_TWI_INSTANCE(0);

static uint32_t _MS_ = 0;

// I2C pins
#define SCL_PIN 31
#define SDA_PIN 30

/*UART buffer size. */
#define UART_TX_BUF_SIZE 1024
#define UART_RX_BUF_SIZE 1


// IMU device addresses
#define MPU9250         (0x68)
#define AK8963          (0x0C)


struct IMU
{
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
    }accel, gyro, magno;

    int16_t temperature;
};

static void uart_events_handler(app_uart_evt_t * p_event);


// uart init
static void uart_config(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_events_handler,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);
}


// uart non blocking
static void uart_events_handler(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


// i2f init
void twi_init (void)
{
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t twi_mma_7660_config = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };

    /*err_code = nrf_drv_twi_init(&m_twi_mpu9250, &twi_mma_7660_config, twi_handler, NULL);*/
    err_code = nrf_drv_twi_init(&m_twi_mpu9250, &twi_mma_7660_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(&m_twi_mpu9250);
}

#define b2int16(x) ((*(uint8_t*)(x+1)) |  ((*(uint8_t*)(x))<<8))
#define b2int16n(x) ((*(uint8_t*)(x)) |  ((*(uint8_t*)(x+1))<<8))

void parse_data(struct IMU * imu, uint8_t * data)
{
    imu->accel.x = b2int16(data + 0);
    imu->accel.y = b2int16(data + 2);
    imu->accel.z = b2int16(data + 4);
    
    imu->temperature = b2int16(data + 6);

    imu->gyro.x = b2int16(data + 8);
    imu->gyro.y = b2int16(data + 10);
    imu->gyro.z = b2int16(data + 12);

    imu->magno.x = b2int16n(data + 14);
    imu->magno.y = b2int16n(data + 16);
    imu->magno.z = b2int16n(data + 18);

}

uint8_t read_reg(uint8_t device, uint8_t addr)
{
    ret_code_t err_code;
    uint8_t data;

    // put magnometer in continuous mode
    err_code = nrf_drv_twi_tx(&m_twi_mpu9250, device, &addr, 1, true);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_rx(&m_twi_mpu9250, device, &data, 1);
    APP_ERROR_CHECK(err_code);

    return data;
}


void read_data(uint8_t device, uint8_t addr, uint8_t * data, int num)
{
    ret_code_t err_code;

    // put magnometer in continuous mode
    err_code = nrf_drv_twi_tx(&m_twi_mpu9250, device, &addr, 1, true);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_rx(&m_twi_mpu9250, device, data, num);
    APP_ERROR_CHECK(err_code);
}

void init_imu()
{
    ret_code_t err_code;
    // enable I2C pass through to access magnetometer
    uint8_t mode_data[] = {0x38, 0x01};
    err_code = nrf_drv_twi_tx(&m_twi_mpu9250, MPU9250, mode_data, 2, false);
    APP_ERROR_CHECK(err_code);

    mode_data[0] = 0x37;
    mode_data[1] = 0x22;
    err_code = nrf_drv_twi_tx(&m_twi_mpu9250, MPU9250, mode_data, 2, false);
    APP_ERROR_CHECK(err_code);


    // reset magnetometer
    mode_data[0] = 0x0b;
    mode_data[1] = 0x01;
    err_code = nrf_drv_twi_tx(&m_twi_mpu9250, AK8963, mode_data, 2, false);
    APP_ERROR_CHECK(err_code);


    // put magnometer in continuous mode
    mode_data[0] = 0x0a;
    mode_data[1] = 0x12;
    err_code = nrf_drv_twi_tx(&m_twi_mpu9250, AK8963, mode_data, 2, false);
    APP_ERROR_CHECK(err_code);
}

void init_ble()
{
    uint32_t err_code;
    bool erase_bonds;

    // Initialize.
    timers_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    device_manager_init(erase_bonds);
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();

    // Start execution.
    application_timers_start();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

static void timer_timeout_handler(void * p_context)
{
    _MS_ = _MS_ + 1;
    /*printf("A");*/
}

void init_timer()
{
#define PRESCALER 0
    ret_code_t err_code;
    APP_TIMER_INIT(PRESCALER, 3, false);
    APP_TIMER_DEF(m_our_char_timer_id);
    err_code = app_timer_create(&m_our_char_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_our_char_timer_id, APP_TIMER_TICKS(1, PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
}


int main(void)
{

    ret_code_t err_code;
    uint8_t reg; // ACCEL_XOUT_L;
    uint8_t data[SENSOR_DATA_LEN];
    struct IMU imu;
    uint8_t sens_id = 0x55;

    /*nrf_drv_clock_init();*/
    /*[>APP_ERROR_CHECK(err_code);<]*/
    /*nrf_drv_clock_lfclk_request(NULL);*/

    uart_config();
    init_timer();
    // twi == i2c
    twi_init();
    // Configure LED-pins as outputs.
    LEDS_CONFIGURE(LEDS_MASK);
    init_imu();

    // ble start up
    init_ble();



    printf("smart shirt\r\n");

    while (true)
    {
        // Toggle LEDs.
        for (int i = 0; i < LEDS_NUMBER; i++)
        {
            LEDS_INVERT(1 << leds_list[i]);
            nrf_delay_ms(1);
        }
        power_manage();
        
        // burst read accel, gyro, temperature
        reg = 0x3B;          
        err_code = nrf_drv_twi_tx(&m_twi_mpu9250, MPU9250, &reg, sizeof(reg), true);
        APP_ERROR_CHECK(err_code);
        err_code = nrf_drv_twi_rx(&m_twi_mpu9250, MPU9250, data, 14);
        APP_ERROR_CHECK(err_code);

        // read magno
        if (read_reg(AK8963, 0x02) & 1)
        {
            reg = 0x03;  // magno start
            err_code = nrf_drv_twi_tx(&m_twi_mpu9250, AK8963, &reg, sizeof(reg), true);
            APP_ERROR_CHECK(err_code);
            err_code = nrf_drv_twi_rx(&m_twi_mpu9250, AK8963, data+14, 7);
            APP_ERROR_CHECK(err_code);
        }

        // add timestamp
        memmove(data+14+7, &_MS_, 4);

        // add sensor ID
        memmove(data+14+7+4, &sens_id, 1);

        memmove(sensor_output_buf, data, SENSOR_DATA_LEN);

        sensor_update();

        // put data neatly in IMU struct
        /*parse_data(&imu,data);*/

        // output
        /*printf("accel:\r\n");*/
        /*printf("    x:%d y:%d z:%d\r\n", imu.accel.x, imu.accel.y, imu.accel.z);*/
        /*printf("gyro:\r\n");*/
        /*printf("    x:%d y:%d z:%d\r\n", imu.gyro.x, imu.gyro.y, imu.gyro.z);*/
        /*printf("magno:\r\n");*/
        /*printf("    x:%d y:%d z:%d\r\n", imu.magno.x, imu.magno.y, imu.magno.z);*/
        /*printf("temperature:\r\n");*/
        /*printf("    t:%d\r\n", imu.temperature);*/
        /*printf("time:\r\n");*/
        /*printf("    t:%ld\r\n", _MS_);*/
        /*printf("\r\n");*/

    }
}


