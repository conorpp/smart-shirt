#ifndef _APP_H__
#define _APP_H__

#include <ble_gatts.h>
#include <stdint.h>

//conor
typedef struct
{
    // custom service reference
    uint16_t    service_handle;

    // ble connection reference
    uint16_t    conn_handle;

    // custom characteristics reference
    ble_gatts_char_handles_t char_handle;

} ble_sens_t;

// conor
#define BLE_UUID_OUR_BASE_UUID {0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00} // 128-bit base UUID
#define BLE_UUID_OUR_SERVICE    0xCAFE // Just a random, but recognizable value
#define BLE_UUID_OUR_CHAR       0xBABE // Just a random, but recognizable value

#define SENSOR_DATA_LEN     11

// conor
extern uint8_t sensor_output_buf[SENSOR_DATA_LEN];

void timers_init(void);
void buttons_leds_init(bool * p_erase_bonds);
void ble_stack_init(void);
void device_manager_init(bool erase_bonds);
void gap_params_init(void);
void advertising_init(void);
void services_init(void);
void conn_params_init(void);
void application_timers_start(void);
void power_manage(void);

void sensor_update();

#endif

