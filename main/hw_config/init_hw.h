/**
 * @file hw_init.h
 * @author Yasir K. Qureshi (embenix.com)
 * @brief Hardware initialization functions
 * @version 0.1
 * @date 2025-09-19
 * 
 * @copyright Copyright (c) 2025
 * 
 * This file contains the function declarations for initializing
 * the hardware components of the ESP32-P4 Function EV Board.
 */

#pragma once

#include "sdkconfig.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

#if CONFIG_ESPRESSIF_ESP32P4_FUNCTION_EV_BOARD
#include "espressif_esp32p4_function_ev_board.h"
#elif CONFIG_OLIMEX_ESP32P4_DEVKIT
#include "olimex_esp32p4_devkit.h"
#elif CONFIG_WAVESHARE_ESP32P4_MODULE_DEVKIT
#include "waveshare_esp32p4_module_devkit.h"
#else
#error "No board configuration selected. Please select a board in the project configuration."
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the hardware components
 * 
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t hw_init(void);

// Other hardware initialization functions
esp_err_t esp_i2c_init(void);
i2c_master_bus_handle_t esp_i2c_get_handle(void);
void i2c_scan(void);
esp_err_t esp_enable_dsi_phy_power(void);

#ifdef __cplusplus
}
#endif
