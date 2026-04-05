/**
 * @file dfrobot_firebeetle2_esp32p4.h
 * @author Yasir K. Qureshi (embenix.com)
 * @brief Board Support Package for DFRobot FireBeetle 2 ESP32-P4
 * @version 0.1
 * @date 2026-04-05
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once
 
#include "sdkconfig.h"

#if CONFIG_DFROBOT_FIREBEETLE2_ESP32P4

#ifndef DFROBOT_FIREBEETLE2_ESP32P4_H
#define DFROBOT_FIREBEETLE2_ESP32P4_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HW_BOARD            "DFROBOT_FIREBEETLE2_ESP32P4"

// I2C configuration for the board
#define I2C_DSI_PORT        (I2C_NUM_1)
#define I2C_SCL_GPIO        (8)
#define I2C_SDA_GPIO        (7)
#define I2C_FREQ_HZ         (100000)

#define ESP_MIPI_DSI_PHY_PWR_LDO_CHAN       (3)  // LDO_VO3 is connected to VDD_MIPI_DPHY
#define ESP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV (2500)

// Todo: Define other board-specific configurations here

/**
 * @brief Initialize the DFRobot FireBeetle 2 ESP32-P4 board
 * 
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t dfrobot_firebeetle2_esp32p4_init(void);

#ifdef __cplusplus
}
#endif

#endif // DFROBOT_FIREBEETLE2_ESP32P4_H

#endif // CONFIG_DFROBOT_FIREBEETLE2_ESP32P4
