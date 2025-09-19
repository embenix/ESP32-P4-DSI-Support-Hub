/**
 * @file Luckfox-5inch-DSI-Touchscreen.h
 * @author Yasir K. Qureshi (embenix.com)
 * @brief Luckfox 5-inch DSI Touchscreen driver
 * @version 0.1
 * @date 2025-09-19
 * 
 * @copyright Copyright (c) 2025
 * 
 * @link https://wiki.luckfox.com/Display/5inch-DSI-Touchscreen/ @endlink
 */

// This file is included only if the Luckfox 5-inch DSI Touchscreen
// is enabled in the project configuration.
#include "sdkconfig.h"
#if CONFIG_LUCKFOX_5INCH_DSI_TOUCHSCREEN

#include <stdio.h>
#include "esp_log.h"
#include "init_lcd.h"

const char *TAG = "Luckfox_5inch_DSI_Touchscreen";

/* LCD color formats */
#define ESP_LCD_COLOR_FORMAT_RGB565    (1)
#define ESP_LCD_COLOR_FORMAT_RGB888    (2)

#define ESP_LCD_H_RES              (800)
#define ESP_LCD_V_RES              (480)
#define ESP_LCD_MIPI_DSI_LANE_BITRATE_MBPS (800)

#define DSI_PANEL_DPI_CONFIG(px_format)                  \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 30,                        \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 800,                               \
            .v_size = 480,                               \
            .hsync_back_porch = 2,                       \
            .hsync_pulse_width = 45,                     \
            .hsync_front_porch = 131,                    \
            .vsync_back_porch = 2,                       \
            .vsync_pulse_width = 22,                     \
            .vsync_front_porch = 7,                      \
        },                                               \
        .flags.use_dma2d = true,                         \
    }


/**
 * @brief Initialize the LCD
 * 
 */
void init_lcd(void)
{
    // Initialize the LCD here
    ESP_LOGI(TAG, "LCD initialized successfully.");
}

#endif // CONFIG_LUCKFOX_5INCH_DSI_TOUCHSCREEN
// End of file Luckfox-5inch-DSI-Touchscreen.h