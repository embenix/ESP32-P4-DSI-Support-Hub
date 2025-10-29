/**
 * @file buydisplay-5inch-ER-TFT050-10.c
 * @author Yasir K. Qureshi (embenix.com)
 * @brief BuyDisplay 5inch ER-TFT050-10 driver
 * @version 0.1
 * @date 2025-09-26
 * 
 * @copyright Copyright (c) 2025
 * 
 * @link https://www.buydisplay.com/5-inch-720x1280-ips-tft-lcd-display-mipi-interface-ili9881-controller @endlink
 */

// This file is included only if the BuyDisplay 5inch ER-TFT050-10
// is enabled in the project configuration.
#include "sdkconfig.h"

#if CONFIG_BUYDISPLAY_5INCH_ER_TFT050_10 == 1

#include <stdio.h>
#include "driver/ledc.h"
#include "init_lcd.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_lcd_ili9881c.h"

const char *LCD_NAME = "BuyDisplay 5 inch 720x1280 IPS TFT LCD Display";
const char *TAG      = "BuyDisplay 5 inch ER-TFT050-10";

/* LCD color formats and other configurations */
#define LCD_COLOR_FORMAT_RGB565         (1)
#define LCD_COLOR_FORMAT_RGB888         (2)  // Works but very slow | better to use RGB565
#define USE_LCD_COLOR_FORMAT_RGB888     (0)  // Set to 1 to use RGB888, 0 for RGB565
#define USE_LVGL_AVOID_TEAR             (0)  // Set to 1 to enable avoid tearing feature in LVGL
#define USE_LVGL_FULL_REFRESH           (0)  // Set to 1 to enable full refresh feature in LVGL
#define USE_LVGL_DIRECT_MODE            (0)  // Set to 1 to enable direct mode feature in LVGL

#define LCD_COLOR_SPACE                 (ESP_LCD_COLOR_SPACE_RGB)

#define LCD_H_RES                       (720)
#define LCD_V_RES                       (1280)
#define LCD_MIPI_DSI_LANE_BITRATE_MBPS  (1000)
#define LCD_MIPI_DSI_LANE_NUM           (2)  // 2 data lanes

#if USE_LCD_COLOR_FORMAT_RGB888
/* RGB888 needs DMA2D path; keep buffers in internal RAM and moderate size */
#define LCD_DRAW_BUFF_SIZE              (LCD_H_RES * 30)
#define LCD_DRAW_BUFF_DOUBLE            (0)
#define EN_LCD_BUFF_DMA                 (0) // Set to 0 to allocate frame buffer in internal RAM
#define EN_LCD_BUFF_SPIRAM              (0) // Set to 0 to allocate frame buffer in internal RAM
#else
#define LCD_DRAW_BUFF_SIZE              (LCD_H_RES * 50) // Frame buffer size in pixels
#define LCD_DRAW_BUFF_DOUBLE            (0)
#define EN_LCD_BUFF_DMA                 (1)  // Set to 1 to allocate frame buffer in DMA-capable memory
#define EN_LCD_BUFF_SPIRAM              (1)  // Set to 1 to allocate frame buffer in SPIRAM (if available)
#endif

#define EN_LCD_SW_ROTATE                (1)  // Set to 1 to enable software rotation (90° or 270°)
#define LCD_DPI_BUFFER_NUMS             (2)  // Number of frame buffers for DPI mode (1 or 2)

#define BUYDISPLAY_5INCH_ER_TFT050_10_CONF1(px_format)  \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 80,                        \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 720,                               \
            .v_size = 1280,                              \
            .hsync_back_porch = 239,                     \
            .hsync_pulse_width = 50,                     \
            .hsync_front_porch = 33,                     \
            .vsync_back_porch = 20,                      \
            .vsync_pulse_width = 30,                     \
            .vsync_front_porch = 2,                      \
        },                                               \
        .flags.use_dma2d = true,                         \
    }

#define BUYDISPLAY_5INCH_ER_TFT050_10_CONF2(px_format)         \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 80,                        \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 720,                               \
            .v_size = 1280,                              \
            .hsync_back_porch = 100,                     \
            .hsync_pulse_width = 100,                    \
            .hsync_front_porch = 80,                     \
            .vsync_back_porch = 20,                      \
            .vsync_pulse_width = 20,                     \
            .vsync_front_porch = 20,                     \
        },                                               \
        .flags.use_dma2d = true,                         \
    }

#define LCD_RST_GPIO                    (33) // GPIO for LCD reset
#define LCD_TOUCH_RST_GPIO              (23) // Shared with LCD reset
#define LCD_TOUCH_INT_GPIO              (22) // GPIO for LCD touch interrupt
#define LCD_DATA_BIGENDIAN              (0)

static esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
static lv_indev_t *disp_indev = NULL;
static lv_display_t *disp = NULL;
static esp_lcd_touch_handle_t s_touch_handle = NULL;

/* Optional: provide a supplier-specific ILI9881C init sequence.
 * Set to 1 and fill buydisplay_ili9881c_init[] with the vendor's commands
 * (type ili9881c_lcd_init_cmd_t) to override the driver's default sequence.
 * NOTE: Leaving this enabled with an empty array will likely result in a blank display.
 */
#define USE_SUPPLIER_ILI9881C_INIT 1
#if USE_SUPPLIER_ILI9881C_INIT
static const ili9881c_lcd_init_cmd_t buydisplay_ili9881c_init[] = {
    // { cmd, data_ptr, data_bytes, delay_ms }
    // ER-TFT050-10
    { 0xFF, (uint8_t[]){ 0x98, 0x81, 0x03 }, 3, 0 },

    // GIP_1
    { 0x01, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x02, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x03, (uint8_t[]){ 0x73 }, 1, 0 },
    { 0x04, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x05, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x06, (uint8_t[]){ 0x08 }, 1, 0 },
    { 0x07, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x08, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x09, (uint8_t[]){ 0x1B }, 1, 0 },
    { 0x0A, (uint8_t[]){ 0x01 }, 1, 0 },
    { 0x0B, (uint8_t[]){ 0x01 }, 1, 0 },
    { 0x0C, (uint8_t[]){ 0x0D }, 1, 0 },
    { 0x0D, (uint8_t[]){ 0x01 }, 1, 0 },
    { 0x0E, (uint8_t[]){ 0x01 }, 1, 0 },
    { 0x0F, (uint8_t[]){ 0x26 }, 1, 0 },
    { 0x10, (uint8_t[]){ 0x26 }, 1, 0 },
    { 0x11, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x12, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x13, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x14, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x15, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x16, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x17, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x18, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x19, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x1A, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x1B, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x1C, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x1D, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x1E, (uint8_t[]){ 0x40 }, 1, 0 },
    { 0x1F, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x20, (uint8_t[]){ 0x06 }, 1, 0 },
    { 0x21, (uint8_t[]){ 0x01 }, 1, 0 },
    { 0x22, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x23, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x24, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x25, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x26, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x27, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x28, (uint8_t[]){ 0x33 }, 1, 0 },
    { 0x29, (uint8_t[]){ 0x03 }, 1, 0 },
    { 0x2A, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x2B, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x2C, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x2D, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x2E, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x2F, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x30, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x31, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x32, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x33, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x34, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x35, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x36, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x37, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x38, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x39, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x3A, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x3B, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x3C, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x3D, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x3E, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x3F, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x40, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x41, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x42, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x43, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x44, (uint8_t[]){ 0x00 }, 1, 0 },

    // GIP_2
    { 0x50, (uint8_t[]){ 0x01 }, 1, 0 },
    { 0x51, (uint8_t[]){ 0x23 }, 1, 0 },
    { 0x52, (uint8_t[]){ 0x45 }, 1, 0 },
    { 0x53, (uint8_t[]){ 0x67 }, 1, 0 },
    { 0x54, (uint8_t[]){ 0x89 }, 1, 0 },
    { 0x55, (uint8_t[]){ 0xAB }, 1, 0 },
    { 0x56, (uint8_t[]){ 0x01 }, 1, 0 },
    { 0x57, (uint8_t[]){ 0x23 }, 1, 0 },
    { 0x58, (uint8_t[]){ 0x45 }, 1, 0 },
    { 0x59, (uint8_t[]){ 0x67 }, 1, 0 },
    { 0x5A, (uint8_t[]){ 0x89 }, 1, 0 },
    { 0x5B, (uint8_t[]){ 0xAB }, 1, 0 },
    { 0x5C, (uint8_t[]){ 0xCD }, 1, 0 },
    { 0x5D, (uint8_t[]){ 0xEF }, 1, 0 },

    // GIP_3
    { 0x5E, (uint8_t[]){ 0x11 }, 1, 0 },
    { 0x5F, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x60, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x61, (uint8_t[]){ 0x07 }, 1, 0 },
    { 0x62, (uint8_t[]){ 0x06 }, 1, 0 },
    { 0x63, (uint8_t[]){ 0x0E }, 1, 0 },
    { 0x64, (uint8_t[]){ 0x0F }, 1, 0 },
    { 0x65, (uint8_t[]){ 0x0C }, 1, 0 },
    { 0x66, (uint8_t[]){ 0x0D }, 1, 0 },
    { 0x67, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x68, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x69, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x6A, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x6B, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x6C, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x6D, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x6E, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x6F, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x70, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x71, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x72, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x73, (uint8_t[]){ 0x05 }, 1, 0 },
    { 0x74, (uint8_t[]){ 0x01 }, 1, 0 },
    { 0x75, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x76, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x77, (uint8_t[]){ 0x07 }, 1, 0 },
    { 0x78, (uint8_t[]){ 0x06 }, 1, 0 },
    { 0x79, (uint8_t[]){ 0x0E }, 1, 0 },
    { 0x7A, (uint8_t[]){ 0x0F }, 1, 0 },
    { 0x7B, (uint8_t[]){ 0x0C }, 1, 0 },
    { 0x7C, (uint8_t[]){ 0x0D }, 1, 0 },
    { 0x7D, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x7E, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x7F, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x80, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x81, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x82, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x83, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x84, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x85, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x86, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x87, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x88, (uint8_t[]){ 0x02 }, 1, 0 },
    { 0x89, (uint8_t[]){ 0x05 }, 1, 0 },
    { 0x8A, (uint8_t[]){ 0x01 }, 1, 0 },

    // CMD_Page 4
    { 0xFF, (uint8_t[]){ 0x98, 0x81, 0x04 }, 3, 0 },
    // { 0x00, (uint8_t[]){ 0x80 }, 1, 0 },
    { 0x38, (uint8_t[]){ 0x01 }, 1, 0 },
    { 0x39, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x6C, (uint8_t[]){ 0x15 }, 1, 0 },
    { 0x6E, (uint8_t[]){ 0x1A }, 1, 0 }, // di_pwr_reg=0 VGH clamp 15V
    { 0x6F, (uint8_t[]){ 0x25 }, 1, 0 }, // reg vcl + VGH pumping ratio 3x VGL=-2x
    { 0x3A, (uint8_t[]){ 0xA4 }, 1, 0 },
    { 0x8D, (uint8_t[]){ 0x20 }, 1, 0 }, // VGL clamp -10V
    { 0x87, (uint8_t[]){ 0xBA }, 1, 0 },
    { 0x3B, (uint8_t[]){ 0x98 }, 1, 0 },

    // CMD_Page 1
    { 0xFF, (uint8_t[]){ 0x98, 0x81, 0x01 }, 3, 0 },
    { 0x22, (uint8_t[]){ 0x0A }, 1, 0 }, // BGR, SS
    { 0x31, (uint8_t[]){ 0x00 }, 1, 0 }, // column inversion
    { 0x50, (uint8_t[]){ 0x6B }, 1, 0 }, // VREG1OUT
    { 0x51, (uint8_t[]){ 0x66 }, 1, 0 }, // VREG2OUT
    { 0x53, (uint8_t[]){ 0x73 }, 1, 0 }, // VCOM1
    { 0x55, (uint8_t[]){ 0x8B }, 1, 0 }, // VCOM2
    { 0x60, (uint8_t[]){ 0x1B }, 1, 0 }, // SDT
    { 0x61, (uint8_t[]){ 0x01 }, 1, 0 },
    { 0x62, (uint8_t[]){ 0x0C }, 1, 0 },
    { 0x63, (uint8_t[]){ 0x00 }, 1, 0 },

    // VP255 Gamma P
    { 0xA0, (uint8_t[]){ 0x00 }, 1, 0 },
    // VP251
    { 0xA1, (uint8_t[]){ 0x15 }, 1, 0 },
    // VP247
    { 0xA2, (uint8_t[]){ 0x1F }, 1, 0 },
    // VP243
    { 0xA3, (uint8_t[]){ 0x13 }, 1, 0 },
    // VP239
    { 0xA4, (uint8_t[]){ 0x11 }, 1, 0 },
    // VP231
    { 0xA5, (uint8_t[]){ 0x21 }, 1, 0 },
    // VP219
    { 0xA6, (uint8_t[]){ 0x17 }, 1, 0 },
    // VP203
    { 0xA7, (uint8_t[]){ 0x1B }, 1, 0 },
    // VP175
    { 0xA8, (uint8_t[]){ 0x6B }, 1, 0 },
    // VP144
    { 0xA9, (uint8_t[]){ 0x1E }, 1, 0 },
    // VP111
    { 0xAA, (uint8_t[]){ 0x2B }, 1, 0 },
    // VP80
    { 0xAB, (uint8_t[]){ 0x5D }, 1, 0 },
    // VP52
    { 0xAC, (uint8_t[]){ 0x19 }, 1, 0 },
    // VP36
    { 0xAD, (uint8_t[]){ 0x14 }, 1, 0 },
    // VP24
    { 0xAE, (uint8_t[]){ 0x4B }, 1, 0 },
    // VP16
    { 0xAF, (uint8_t[]){ 0x1D }, 1, 0 },
    // VP12
    { 0xB0, (uint8_t[]){ 0x27 }, 1, 0 },
    // VP8
    { 0xB1, (uint8_t[]){ 0x49 }, 1, 0 },
    // VP4
    { 0xB2, (uint8_t[]){ 0x5D }, 1, 0 },
    // VP0
    { 0xB3, (uint8_t[]){ 0x39 }, 1, 0 },

    // VN255 Gamma N
    { 0xC0, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0xC1, (uint8_t[]){ 0x01 }, 1, 0 },
    { 0xC2, (uint8_t[]){ 0x0C }, 1, 0 },
    { 0xC3, (uint8_t[]){ 0x11 }, 1, 0 },
    { 0xC4, (uint8_t[]){ 0x15 }, 1, 0 },
    { 0xC5, (uint8_t[]){ 0x28 }, 1, 0 },
    { 0xC6, (uint8_t[]){ 0x1B }, 1, 0 },
    { 0xC7, (uint8_t[]){ 0x1C }, 1, 0 },
    { 0xC8, (uint8_t[]){ 0x62 }, 1, 0 },
    { 0xC9, (uint8_t[]){ 0x1C }, 1, 0 },
    { 0xCA, (uint8_t[]){ 0x29 }, 1, 0 },
    { 0xCB, (uint8_t[]){ 0x60 }, 1, 0 },
    { 0xCC, (uint8_t[]){ 0x16 }, 1, 0 },
    { 0xCD, (uint8_t[]){ 0x17 }, 1, 0 },
    { 0xCE, (uint8_t[]){ 0x4A }, 1, 0 },
    { 0xCF, (uint8_t[]){ 0x23 }, 1, 0 },
    { 0xD0, (uint8_t[]){ 0x24 }, 1, 0 },
    { 0xD1, (uint8_t[]){ 0x4F }, 1, 0 },
    { 0xD2, (uint8_t[]){ 0x5F }, 1, 0 },
    { 0xD3, (uint8_t[]){ 0x39 }, 1, 0 },

    // CMD_Page 0
    { 0xFF, (uint8_t[]){ 0x98, 0x81, 0x00 }, 3, 0 },
    { 0x35, NULL, 0, 0 },
    { 0x11, NULL, 0, 120 }, // delay 120ms after SLPOUT
    { 0x29, NULL, 0, 0 },
};
#endif

// Function prototypes
static lv_display_t *display_lcd_init(void);
static lv_indev_t *esp_display_indev_init(lv_display_t *disp);
static esp_err_t esp_display_new_with_handles(const lcd_display_config_t *config, esp_lcd_handles_t *ret_handles);
static esp_err_t lcd_touch_new(const lcd_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch);
static esp_err_t lcd_brightness_init(void);
static esp_err_t panel_gpio_init_default(void);

/* Backlight PWM (LEDC) configuration */
#define BL_PIN                 (28)
#define BL_LEDC_TIMER          LEDC_TIMER_0
#define BL_LEDC_MODE           LEDC_LOW_SPEED_MODE
#define BL_LEDC_CHANNEL        LEDC_CHANNEL_0
#define BL_LEDC_FREQ_HZ        (180)            /* 180 Hz to avoid audible noise */
#define BL_LEDC_DUTY_RES       LEDC_TIMER_9_BIT /* 9-bit -> 511 max */

/* Touch and LCD GPIOs */
#define TOUCH_INT_PIN          (22)
#define TOUCH_RST_PIN          (23)
#define LCD_RST_PIN            (33)

static bool panel_gpio_inited = false;

#if CONFIG_PRINT_TOUCH_EVENTS
/**
 * @brief Touch event logger task
 */
static void touch_logger_task(void *arg) {
    esp_lcd_touch_handle_t tp = display_touch_get_handle();
    const uint8_t max = 5; // supports up to 5 points
    uint16_t xs[5], ys[5];
    uint8_t count = 0;
    for (;;) {
        if (tp) {
            esp_lcd_touch_read_data(tp);
            if (esp_lcd_touch_get_coordinates(tp, xs, ys, NULL, &count, max)) {
                chalk_printf(CHALK_WHITE, "Touch count: %u,", count);
                for (uint8_t i = 0; i < count; i++) {
                    chalk_printf(CHALK_WHITE, " T%u:", i);
                    chalk_printf(CHALK_GREEN, " x=%u", xs[i]);
                    chalk_printf(CHALK_BLUE, " y=%u", ys[i]);
                    if (i < count - 1) {
                        chalk_printf(CHALK_WHITE, ",");
                    } else {
                        chalk_printf(CHALK_WHITE, "\n");
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

#endif // CONFIG_PRINT_TOUCH_EVENTS

/**
 * @brief Initialize the LCD
 * 
 */
esp_err_t init_lcd(void)
{
    // Initialize the LCD here
    esp_err_t ret = ESP_OK;

    disp = display_lcd_init();
    ESP_NULL_CHECK(disp, ESP_FAIL);
    
    disp_indev = esp_display_indev_init(disp);
    ESP_NULL_CHECK(disp_indev, ESP_FAIL);

    /* Initialize backlight to 50 | Range: 0-100 */
    ESP_ERROR_CHECK_RETURN_ERR(lcd_brightness_set(50));

    ESP_LOGI(__func__, "%s initialized successfully.", LCD_NAME);

#if CONFIG_PRINT_TOUCH_EVENTS
    xTaskCreate(touch_logger_task, "touch_logger", 4 * 1024, NULL, 2, NULL);
#endif

    return ret;
}

/**
 * @brief Deinitialize the LCD
 * 
 */
esp_err_t deinit_lcd(void)
{
    // Deinitialize the LCD here
    return ESP_OK;
}

/**
 * @brief Get the LVGL input device for the display
 * 
 * @return lv_indev_t* Pointer to the LVGL input device, or NULL if not initialized
 */
lv_indev_t *display_get_input_dev(void)
{
    return disp_indev;
}

/**
 * @brief Get the LVGL display handle
 * 
 * @return lv_display_t* Pointer to the LVGL display, or NULL if not initialized
 */
lv_display_t *display_get_handle(void)
{
    return disp;
}

/**
 * @brief Initialize panel GPIOs (reset, touch INT)
 * 
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t panel_gpio_init_default(void)
{
    if (panel_gpio_inited) {
        return ESP_OK;
    }

    /* Configure LCD RST (output, default high released) */
    gpio_config_t io_lcd_conf = {
        .pin_bit_mask = (1ULL << LCD_RST_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_lcd_conf));
    gpio_set_level(LCD_RST_PIN, 1);

    /* Setup LEDC PWM for backlight on BL_PIN */
    ledc_timer_config_t timer_cfg = {
        .speed_mode = BL_LEDC_MODE,
        .timer_num = BL_LEDC_TIMER,
        .freq_hz = BL_LEDC_FREQ_HZ,
        .duty_resolution = BL_LEDC_DUTY_RES,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t ch_cfg = {
        .gpio_num = BL_PIN,
        .speed_mode = BL_LEDC_MODE,
        .channel = BL_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BL_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

    panel_gpio_inited = true;

    ESP_LOGI(__func__, "Panel GPIO initialized successfully");

    return ESP_OK;
}

/**
 * @brief Set the backlight brightness percentage (0-100%)
 * 
 * @param percent Brightness percentage (0-100)
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t lcd_brightness_set(int brightness)
{
    if (!panel_gpio_inited) {
        ESP_ERROR_CHECK_RETURN_ERR(panel_gpio_init_default());
    }
    
    if (brightness < 0) brightness = 0;
    if (brightness > 100) brightness = 100;
    /* Map 0..100% to 0..2^res-1 */
    const uint32_t max_duty = (1U << BL_LEDC_DUTY_RES) - 1U;
    uint32_t duty = (max_duty * brightness) / 100U;
    ESP_RETURN_ON_ERROR(ledc_set_duty(BL_LEDC_MODE, BL_LEDC_CHANNEL, duty), __func__, "set duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(BL_LEDC_MODE, BL_LEDC_CHANNEL), __func__, "update duty failed");
    return ESP_OK;
}

/**
 * @brief Initialize LCD brightness control
 * 
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t lcd_brightness_init(void)
{
    esp_i2c_init();
    return ESP_OK;
}

/**
 * @brief Turn off the LCD backlight
 * 
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t lcd_backlight_off(void)
{
    return lcd_brightness_set(0);
}

/**
 * @brief Turn on the LCD backlight
 * 
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t lcd_backlight_on(void)
{
    return lcd_brightness_set(100);
}

/**
 * @brief Create a new display with initialized LCD and return the LVGL display handle
 * 
 * @param cfg Display configuration
 * @return lv_display_t* Pointer to the created LVGL display, or NULL on failure
 */
static lv_display_t *display_lcd_init(void)
{
    esp_lcd_handles_t lcd_panels;
    ESP_ERROR_CHECK_RETURN_NULL(esp_display_new_with_handles(NULL, &lcd_panels));

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_panels.io,
        .panel_handle = lcd_panels.panel,
        .control_handle = lcd_panels.control,
        .buffer_size = LCD_DRAW_BUFF_SIZE,
        .double_buffer = LCD_DRAW_BUFF_DOUBLE,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = true,
        },
#if LVGL_VERSION_MAJOR >= 9
#if USE_LCD_COLOR_FORMAT_RGB888
        .color_format = LV_COLOR_FORMAT_RGB888,
#else
        .color_format = LV_COLOR_FORMAT_RGB565,
#endif
#endif
        .flags = {
            .buff_dma = EN_LCD_BUFF_DMA,
            .buff_spiram = EN_LCD_BUFF_SPIRAM,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = (LCD_DATA_BIGENDIAN ? true : false),
#endif
#if USE_LVGL_AVOID_TEAR
            .sw_rotate = false,                /* Avoid tearing is not supported for SW rotation */
#else
            .sw_rotate = EN_LCD_SW_ROTATE, /* Only SW rotation is supported for 90° and 270° */
#endif
#if USE_LVGL_FULL_REFRESH
            .full_refresh = true,
#elif USE_LVGL_DIRECT_MODE
            .direct_mode = true,
#endif
        }
    };

    const lvgl_port_display_dsi_cfg_t dpi_cfg = {
        .flags = {
#if USE_LVGL_AVOID_TEAR
            .avoid_tearing = true,
#else
            .avoid_tearing = false,
#endif
        }
    };

    return lvgl_port_add_disp_dsi(&disp_cfg, &dpi_cfg);
}

/**
 * @brief Create a new display with initialized LCD and return the handles
 * 
 * @param config Display configuration (currently not used, can be NULL)
 * @param ret_handles Pointer to store the returned LCD handles
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t esp_display_new_with_handles(const lcd_display_config_t *config, esp_lcd_handles_t *ret_handles)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_ERROR(lcd_brightness_init(), __func__, "Brightness init failed");
    ESP_RETURN_ON_ERROR(panel_gpio_init_default(), __func__, "Panel GPIO init failed");
    ESP_RETURN_ON_ERROR(esp_enable_dsi_phy_power(), __func__, "DSI PHY power failed");

    /* create MIPI DSI bus first, it will initialize the DSI PHY as well */
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = LCD_MIPI_DSI_LANE_NUM,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = LCD_MIPI_DSI_LANE_BITRATE_MBPS,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus), __func__, "New DSI bus init failed");

    ESP_LOGI(__func__, "Install (BuyDisplay 5inch) MIPI DSI LCD control panel");
    // we use DBI interface to send LCD commands and parameters
    esp_lcd_panel_io_handle_t io;
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,   // according to the LCD ILI9881C spec
        .lcd_param_bits = 8, // according to the LCD ILI9881C spec
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &io), err, __func__, "New panel IO failed");

    esp_lcd_panel_handle_t disp_panel = NULL;

#if USE_LCD_COLOR_FORMAT_RGB888
    esp_lcd_dpi_panel_config_t dpi_config = BUYDISPLAY_5INCH_ER_TFT050_10_CONF2(LCD_COLOR_PIXEL_FORMAT_RGB888);
#else
    esp_lcd_dpi_panel_config_t dpi_config = BUYDISPLAY_5INCH_ER_TFT050_10_CONF2(LCD_COLOR_PIXEL_FORMAT_RGB565);
#endif

    dpi_config.num_fbs = LCD_DPI_BUFFER_NUMS;

    ili9881c_vendor_config_t vendor_config = {
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
            .lane_num = LCD_MIPI_DSI_LANE_NUM,
        },
    };
#if USE_SUPPLIER_ILI9881C_INIT
    // Attach the supplier-provided init array so the driver uses it instead of its default
    vendor_config.init_cmds = buydisplay_ili9881c_init;
    vendor_config.init_cmds_size = sizeof(buydisplay_ili9881c_init) / sizeof(buydisplay_ili9881c_init[0]);
#endif
    esp_lcd_panel_dev_config_t lcd_dev_config = {

#if USE_LCD_COLOR_FORMAT_RGB888
        .bits_per_pixel = 24,
#else
        .bits_per_pixel = 16,
#endif
        .rgb_ele_order = LCD_COLOR_SPACE,
        .reset_gpio_num = LCD_RST_GPIO,
        .vendor_config = &vendor_config,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_ili9881c(io, &lcd_dev_config, &disp_panel), err, __func__, "New LCD panel Luckfox 5\" failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(disp_panel), err, __func__, "LCD panel reset failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(disp_panel), err, __func__, "LCD panel init failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_disp_on_off(disp_panel, true), err, __func__, "LCD panel ON failed");

    /* Return all handles */
    ret_handles->io = io;
    ret_handles->mipi_dsi_bus = mipi_dsi_bus;
    ret_handles->panel = disp_panel;
    ret_handles->control = NULL;

    ESP_LOGI(__func__, "%s initialized", LCD_NAME);

    return ret;

err:
    ESP_LOGE(__func__, "%s initialization failed", LCD_NAME);
    if (disp_panel) {
        esp_lcd_panel_del(disp_panel);
        disp_panel = NULL;
    }
    if (io) {
        esp_lcd_panel_io_del(io);
        io = NULL;
    }
    if (mipi_dsi_bus) {
        esp_lcd_del_dsi_bus(mipi_dsi_bus);
        mipi_dsi_bus = NULL;
    }
    return ret;
}

/**
 * @brief Create a new touch input device
 * 
 * @param config Touch configuration (currently not used, can be NULL)
 * @param ret_touch Pointer to store the returned touch handle
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t lcd_touch_new(const lcd_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    /* Initilize I2C */
    ESP_ERROR_CHECK_RETURN_ERR(esp_i2c_init());

    /* Initialize touch */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,

        .rst_gpio_num = LCD_TOUCH_RST_GPIO,
        .int_gpio_num = LCD_TOUCH_INT_GPIO,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = {
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS, // primary GT911 addr (0x5D)
        .control_phase_bytes = 1,
        .dc_bit_offset = 0,
        .lcd_cmd_bits = 16,
        .flags = {
            .disable_control_phase = 1,
        },
    };

    tp_io_config.scl_speed_hz = 100000;

    i2c_master_bus_handle_t i2c_bus = esp_i2c_get_handle();
    ESP_LOGI(__func__, "I2C bus handle: %p", i2c_bus);
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_bus, &tp_io_config, &tp_io_handle), __func__, "");

    esp_err_t ret = esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, ret_touch);
    if (ret == ESP_OK) {
        s_touch_handle = *ret_touch;
    }
    return ret;
}


/**
 * @brief Initialize touch input device
 * 
 * @param disp Pointer to the display
 * @return lv_indev_t* Pointer to the input device, or NULL on failure
 */
static lv_indev_t *esp_display_indev_init(lv_display_t *disp)
{
    esp_lcd_touch_handle_t tp;
    ESP_ERROR_CHECK_RETURN_NULL(lcd_touch_new(NULL, &tp));
    assert(tp);

    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp,
        .handle = tp,
    };

    return lvgl_port_add_touch(&touch_cfg);
}

/**
 * @brief Get the touch handle
 * 
 * @return esp_lcd_touch_handle_t Touch handle, or NULL if not initialized
 */
esp_lcd_touch_handle_t display_touch_get_handle(void)
{
    return s_touch_handle;
}

#endif // CONFIG_BUYDISPLAY_5INCH_ER_TFT050_10
// End of file buydisplay-5inch-ER-TFT050-10.c