/**
 * @file waveshare_esp32p4_module_devkit.h
 * @author Yasir K. Qureshi (embenix.com)
 * @brief Hardware initialization functions for Waveshare ESP32-P4 Module Dev Kit
 * @version 0.1
 * @date 2025-09-19
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include "sdkconfig.h"

#if CONFIG_WAVESHARE_ESP32P4_MODULE_DEVKIT

#ifndef WAVESHARE_ESP32P4_MODULE_DEVKIT_H
#define WAVESHARE_ESP32P4_MODULE_DEVKIT_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HW_BOARD            "WAVESHARE_ESP32P4_MODULE_DEVKIT"

// I2C configuration for the board
#define I2C_DSI_PORT        (I2C_NUM_1)
#define I2C_SCL_GPIO        (8)
#define I2C_SDA_GPIO        (7)
#define I2C_FREQ_HZ         (100000)

#define ESP_MIPI_DSI_PHY_PWR_LDO_CHAN       (3)  // LDO_VO3 is connected to VDD_MIPI_DPHY
#define ESP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV (2500)

// Todo: Define other board-specific configurations here

// Work in progress

/**
 * @brief Initialize the hardware components
 * 
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t hw_init(void);

#ifdef __cplusplus
}
#endif
#endif // CONFIG_WAVESHARE_ESP32P4_MODULE_DEVKIT
#endif // WAVESHARE_ESP32P4_MODULE_DEVKIT_H