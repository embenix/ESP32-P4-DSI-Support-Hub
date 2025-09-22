/**
 * @file init_lcd.h
 * @author Yasir K. Qureshi (embenix.com)
 * @brief LCD initialization header file
 * @version 0.1
 * @date 2025-09-19
 * 
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#ifndef INIT_LCD_H
#define INIT_LCD_H

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_err_check.h"
#include "init_hw.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_panel_io.h"
#include "esp_lvgl_port.h"        // lvgl_port_cfg_t, lv_display_t, lv_indev_t
#include "esp_lcd_dsi.h"
#include "soc/soc_caps.h"
#if __has_include("esp_lcd_mipi_dsi.h")
#include "esp_lcd_mipi_dsi.h"     // esp_lcd_dsi_bus_handle_t
#endif

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
    int dummy;
} lcd_display_config_t;

typedef struct {
    void *dummy;    /* Placeholder for touch configuration parameters */
} lcd_touch_config_t;

// /**
//  * @brief LCD panel vendor configuration.
//  *
//  * @note  This structure needs to be passed to the `vendor_config` field in `esp_lcd_panel_dev_config_t`.
//  *
//  */
// typedef struct {
//     const hx8394_lcd_init_cmd_t *init_cmds;         /*!< Pointer to initialization commands array. Set to NULL if using default commands.
//                                                      *   The array should be declared as `static const` and positioned outside the function.
//                                                      *   Please refer to `vendor_specific_init_default` in source file.
//                                                      */
//     uint16_t init_cmds_size;                        /*<! Number of commands in above array */
//     struct {
//         esp_lcd_dsi_bus_handle_t dsi_bus;               /*!< MIPI-DSI bus configuration */
//         const esp_lcd_dpi_panel_config_t *dpi_config;   /*!< MIPI-DPI panel configuration */
//         uint8_t  lane_num;
//     } mipi_config;
// } ili9881c_vendor_config_t;

/**
 * @brief ESP display return handles
 *
 */
typedef struct {
#if defined(SOC_MIPI_DSI_SUPPORTED) && SOC_MIPI_DSI_SUPPORTED && __has_include("esp_lcd_mipi_dsi.h")
    esp_lcd_dsi_bus_handle_t    mipi_dsi_bus;  /*!< MIPI DSI bus handle */
#else
    void *                      mipi_dsi_bus;  /*!< MIPI DSI bus handle (placeholder when DSI not available) */
#endif
    esp_lcd_panel_io_handle_t   io;            /*!< ESP LCD IO handle */
    esp_lcd_panel_handle_t      panel;         /*!< ESP LCD panel (color) handle */
    esp_lcd_panel_handle_t      control;       /*!< ESP LCD panel (control) handle */
} esp_lcd_handles_t;


esp_err_t init_lcd(void);
lv_indev_t *display_get_input_dev(void);
lv_display_t *display_get_handle(void);
esp_err_t lcd_brightness_set(int brightness);
esp_err_t lcd_backlight_off(void);
esp_err_t lcd_backlight_on(void);


#ifdef __cplusplus
}
#endif

#endif // INIT_LCD_H