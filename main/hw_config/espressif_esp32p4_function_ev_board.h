/**
 * @file espressif_esp32p4_function_ev_board.h
 * @author Yasir K. Qureshi (embenix.com)
 * @brief Board Support Package for ESP32-P4 Function EV Board
 * @version 0.1
 * @date 2025-09-19
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include "sdkconfig.h"

#if CONFIG_ESPRESSIF_ESP32P4_FUNCTION_EV_BOARD

#ifndef ESPRESSIF_ESP32P4_FUNCTION_EV_BOARD_H
#define ESPRESSIF_ESP32P4_FUNCTION_EV_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include "esp_err.h"

#define HW_BOARD            "ESPRESSIF_ESP32P4_FUNCTION_EV_BOARD"

// I2C configuration for the board
#define I2C_DSI_PORT        (I2C_NUM_1)
#define I2C_SCL_GPIO        (8)
#define I2C_SDA_GPIO        (7)
#define I2C_FREQ_HZ         (100000)

#define ESP_MIPI_DSI_PHY_PWR_LDO_CHAN       (3)  // LDO_VO3 is connected to VDD_MIPI_DPHY
#define ESP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV (2500)

/**
 * @brief Initialize the board
 * 
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t esp_espressif_esp32p4_function_ev_board_init(void);

#ifdef __cplusplus
}
#endif

#endif // ESPRESSIF_ESP32P4_FUNCTION_EV_BOARD_H
#endif // CONFIG_ESPRESSIF_ESP32P4_FUNCTION_EV_BOARD