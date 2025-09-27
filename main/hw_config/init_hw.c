/**
 * @file init_hw.c
 * @author Yasir K. Qureshi (embenix.com)
 * @brief Hardware initialization for ESP32-P4 DSI 
 * @version 0.1
 * @date 2025-09-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "sdkconfig.h"
#include "init_hw.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "driver/i2c_master.h"
#include "esp_ldo_regulator.h"
#include "esp_vfs_fat.h"
#include "usb/usb_host.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "esp_err_check.h"


static bool i2c_initialized = false;
static i2c_master_bus_handle_t i2c_handle = NULL;


 /**
  * @brief Initialize I2C bus
  * 
  * @return esp_err_t ESP_OK on success, or an error code on failure
  *
  */
esp_err_t esp_i2c_init(void)
{
    /* I2C was initialized before */
    if (i2c_initialized) {
        ESP_LOGW(__func__, "I2C already initialized.");
        return ESP_OK;
    }

    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .i2c_port = I2C_DSI_PORT,
    };
    ESP_ERROR_CHECK_RETURN_ERR(i2c_new_master_bus(&i2c_bus_conf, &i2c_handle));

    #if CONFIG_ESP_I2C_USE_PULLUPS
        /* Enable internal pull-ups on I2C lines (helps when external pull-ups are absent/weak) */
        gpio_pullup_en(I2C_SDA_GPIO);
        gpio_pullup_en(I2C_SCL_GPIO);
    #endif // CONFIG_ESP_I2C_USE_PULLUPS

    i2c_initialized = true;
    ESP_LOGI(__func__, "I2C initialized successfully on port %d (SCL GPIO: %d, SDA GPIO: %d)", 
             I2C_DSI_PORT, I2C_SCL_GPIO, I2C_SDA_GPIO);

    return ESP_OK;
}

/**
 * @brief Get the I2C bus handle
 * 
 * @return i2c_master_bus_handle_t I2C bus handle
 */
i2c_master_bus_handle_t esp_i2c_get_handle(void)
{
    return i2c_handle;
}

/**
 * @brief Scan I2C bus for devices
 * 
 */
void i2c_scan(void)
{
    // Ensure bus is initialized
    if (!i2c_initialized) {
        if (esp_i2c_init() != ESP_OK) {
            ESP_LOGE(__func__, "I2C init failed; aborting scan");
            return;
        }
    }

    i2c_master_bus_handle_t bus = esp_i2c_get_handle();
    if (bus == NULL) {
        ESP_LOGE(__func__, "I2C bus handle is NULL; aborting scan");
        return;
    }

    ESP_LOGI(__func__, "Scanning I2C bus...");
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        esp_err_t err = i2c_master_probe(bus, addr, 50 /* ms */);
        if (err == ESP_OK) {
            ESP_LOGI(__func__, "Found device at 0x%02X", addr);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    ESP_LOGI(__func__, "Scan done.");
}

/**
 * @brief Initialize hardware components
 * 
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t init_hw(void)
{
    // Initialize hardware components for ESP32-P4 DSI
    ESP_LOGI(__func__, "Initializing hardware components for %s", HW_BOARD);
    return ESP_OK;
}

 /**
 * @brief Enable power for MIPI DSI PHY
 * 
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t esp_enable_dsi_phy_power(void)
{
#if ESP_MIPI_DSI_PHY_PWR_LDO_CHAN > 0
    // Turn on the power for MIPI DSI PHY, so it can go from "No Power" state to "Shutdown" state
    static esp_ldo_channel_handle_t phy_pwr_chan = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = ESP_MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = ESP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo_cfg, &phy_pwr_chan), __func__, "Acquire LDO channel for DPHY failed");
    ESP_LOGI(__func__, "MIPI DSI PHY Powered on");
#endif // ESP_MIPI_DSI_PHY_PWR_LDO_CHAN > 0

    return ESP_OK;
}

