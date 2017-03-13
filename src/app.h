#ifndef _APP_H__
#define _APP_H__

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

#endif

