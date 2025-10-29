/**
 * @file rpi-5inch-touch-display-v2.c
 * @author Yasir K. Qureshi (embenix.com)
 * @brief Raspberry Pi 5inch Touch Display V2 driver
 * @version 0.1
 * @date 2025-10-27
 * 
 * @copyright Copyright (c) 2025
 * 
 * @link https://www.raspberrypi.com/documentation/accessories/touch-display-2.html#specifications @endlink
 */

// This file is included only if the Raspberry Pi 5inch Touch Display V2
// is enabled in the project configuration.
#include "sdkconfig.h"

#if CONFIG_RPI_5INCH_TOUCH_DISPLAY_V2 == 1

#include <stdio.h>
#include "init_lcd.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_lcd_ili9881c.h"

const char *LCD_NAME = "Raspberry Pi 5inch Touch Display V2";
const char *TAG      = "RPi 5\" Touch Display V2";

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

#define RPI_5INCH_TOUCH_DISPLAY_V2_CONFIG(px_format) \
    {                                                      \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,       \
        .dpi_clock_freq_mhz = 80,                          \
        .virtual_channel = 0,                              \
        .pixel_format = px_format,                         \
        .num_fbs = 1,                                      \
        .video_timing = {                                  \
            .h_size = 720,                                 \
            .v_size = 1280,                                \
            .hsync_back_porch = 105,                       \
            .hsync_pulse_width = 2,                        \
            .hsync_front_porch = 110,                      \
            .vsync_back_porch = 100,                       \
            .vsync_pulse_width = 2,                        \
            .vsync_front_porch = 100,                      \
        },                                                 \
        .flags.use_dma2d = true,                           \
    }

#define LCD_RST_GPIO                    (-1) // GPIO for LCD reset
#define LCD_TOUCH_RST_GPIO              (-1) // Shared with LCD reset
#define LCD_TOUCH_INT_GPIO              (-1) // GPIO for LCD touch interrupt
#define LCD_DATA_BIGENDIAN              (0)

// TC358762 bridge configuration
#define TC358762_ADDR                   (0x45) // 7-bit I2C address of the TC358762 bridge
#define RPI_TOUCH_V2_REG_ID             (0x01)
#define RPI_TOUCH_V2_REG_POWERON        (0x02)
#define RPI_TOUCH_V2_REG_PWM            (0x03)

// REG_POWERON bits
#define RPI_TOUCH_V2_LCD_RESET_BIT      (1u << 0) // 1 = deassert LCD reset
#define RPI_TOUCH_V2_CTP_RESET_BIT      (1u << 1) // 1 = deassert Touch reset

// REG_PWM bits
#define RPI_TOUCH_V2_PWM_ENABLE         (1u << 7)
#define RPI_TOUCH_V2_PWM_VALUE_MASK     (0x1Fu)      // 5-bit brightness (0..31)

// Goodix GT911 typical 7-bit I2C address
#define GT911_ADDR                      (0x5D)

static esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
static lv_indev_t *disp_indev = NULL;
static lv_display_t *disp = NULL;
static esp_lcd_touch_handle_t s_touch_handle = NULL;

/**
 * @brief Custom ILI9881C initialization sequence for Raspberry Pi 5" Touch Display V2
 * 
 * This sequence is based on the vendor's specifications and is used to initialize the display
 * with the correct settings for optimal performance.
 * 
 */
#define USE_SUPPLIER_ILI9881C_INIT 1
#if USE_SUPPLIER_ILI9881C_INIT
static const ili9881c_lcd_init_cmd_t rpi_custom_ili9881c_init[] = {
    // Raspberry Pi 5" Touch Display V2 initialization sequence
    // Derived from rpi_5inch_init[] in panel-ilitek-ili9881c.c
    
    // Page 3
    {0xFF, (const uint8_t[]){0x98, 0x81, 0x03}, 3, 0},
    {0x01, (const uint8_t[]){0x00}, 1, 0},
    {0x02, (const uint8_t[]){0x00}, 1, 0},
    {0x03, (const uint8_t[]){0x73}, 1, 0},
    {0x04, (const uint8_t[]){0x73}, 1, 0},
    {0x05, (const uint8_t[]){0x00}, 1, 0},
    {0x06, (const uint8_t[]){0x06}, 1, 0},
    {0x07, (const uint8_t[]){0x02}, 1, 0},
    {0x08, (const uint8_t[]){0x00}, 1, 0},
    {0x09, (const uint8_t[]){0x01}, 1, 0},
    {0x0a, (const uint8_t[]){0x01}, 1, 0},
    {0x0b, (const uint8_t[]){0x01}, 1, 0},
    {0x0c, (const uint8_t[]){0x01}, 1, 0},
    {0x0d, (const uint8_t[]){0x01}, 1, 0},
    {0x0e, (const uint8_t[]){0x01}, 1, 0},
    {0x0f, (const uint8_t[]){0x01}, 1, 0},
    {0x10, (const uint8_t[]){0x01}, 1, 0},
    {0x11, (const uint8_t[]){0x00}, 1, 0},
    {0x12, (const uint8_t[]){0x00}, 1, 0},
    {0x13, (const uint8_t[]){0x01}, 1, 0},
    {0x14, (const uint8_t[]){0x00}, 1, 0},
    {0x15, (const uint8_t[]){0x00}, 1, 0},
    {0x16, (const uint8_t[]){0x00}, 1, 0},
    {0x17, (const uint8_t[]){0x00}, 1, 0},
    {0x18, (const uint8_t[]){0x00}, 1, 0},
    {0x19, (const uint8_t[]){0x00}, 1, 0},
    {0x1a, (const uint8_t[]){0x00}, 1, 0},
    {0x1b, (const uint8_t[]){0x00}, 1, 0},
    {0x1c, (const uint8_t[]){0x00}, 1, 0},
    {0x1d, (const uint8_t[]){0x00}, 1, 0},
    {0x1e, (const uint8_t[]){0xc0}, 1, 0},
    {0x1f, (const uint8_t[]){0x80}, 1, 0},
    {0x20, (const uint8_t[]){0x04}, 1, 0},
    {0x21, (const uint8_t[]){0x03}, 1, 0},
    {0x22, (const uint8_t[]){0x00}, 1, 0},
    {0x23, (const uint8_t[]){0x00}, 1, 0},
    {0x24, (const uint8_t[]){0x00}, 1, 0},
    {0x25, (const uint8_t[]){0x00}, 1, 0},
    {0x26, (const uint8_t[]){0x00}, 1, 0},
    {0x27, (const uint8_t[]){0x00}, 1, 0},
    {0x28, (const uint8_t[]){0x33}, 1, 0},
    {0x29, (const uint8_t[]){0x03}, 1, 0},
    {0x2a, (const uint8_t[]){0x00}, 1, 0},
    {0x2b, (const uint8_t[]){0x00}, 1, 0},
    {0x2c, (const uint8_t[]){0x00}, 1, 0},
    {0x2d, (const uint8_t[]){0x00}, 1, 0},
    {0x2e, (const uint8_t[]){0x00}, 1, 0},
    {0x2f, (const uint8_t[]){0x00}, 1, 0},
    {0x30, (const uint8_t[]){0x00}, 1, 0},
    {0x31, (const uint8_t[]){0x00}, 1, 0},
    {0x32, (const uint8_t[]){0x00}, 1, 0},
    {0x33, (const uint8_t[]){0x00}, 1, 0},
    {0x34, (const uint8_t[]){0x03}, 1, 0},
    {0x35, (const uint8_t[]){0x00}, 1, 0},
    {0x36, (const uint8_t[]){0x03}, 1, 0},
    {0x37, (const uint8_t[]){0x00}, 1, 0},
    {0x38, (const uint8_t[]){0x00}, 1, 0},
    {0x39, (const uint8_t[]){0x00}, 1, 0},
    {0x3a, (const uint8_t[]){0x00}, 1, 0},
    {0x3b, (const uint8_t[]){0x00}, 1, 0},
    {0x3c, (const uint8_t[]){0x00}, 1, 0},
    {0x3d, (const uint8_t[]){0x00}, 1, 0},
    {0x3e, (const uint8_t[]){0x00}, 1, 0},
    {0x3f, (const uint8_t[]){0x00}, 1, 0},
    {0x40, (const uint8_t[]){0x00}, 1, 0},
    {0x41, (const uint8_t[]){0x00}, 1, 0},
    {0x42, (const uint8_t[]){0x00}, 1, 0},
    {0x43, (const uint8_t[]){0x00}, 1, 0},
    {0x44, (const uint8_t[]){0x00}, 1, 0},
    {0x50, (const uint8_t[]){0x01}, 1, 0},
    {0x51, (const uint8_t[]){0x23}, 1, 0},
    {0x52, (const uint8_t[]){0x45}, 1, 0},
    {0x53, (const uint8_t[]){0x67}, 1, 0},
    {0x54, (const uint8_t[]){0x89}, 1, 0},
    {0x55, (const uint8_t[]){0xab}, 1, 0},
    {0x56, (const uint8_t[]){0x01}, 1, 0},
    {0x57, (const uint8_t[]){0x23}, 1, 0},
    {0x58, (const uint8_t[]){0x45}, 1, 0},
    {0x59, (const uint8_t[]){0x67}, 1, 0},
    {0x5a, (const uint8_t[]){0x89}, 1, 0},
    {0x5b, (const uint8_t[]){0xab}, 1, 0},
    {0x5c, (const uint8_t[]){0xcd}, 1, 0},
    {0x5d, (const uint8_t[]){0xef}, 1, 0},
    {0x5e, (const uint8_t[]){0x10}, 1, 0},
    {0x5f, (const uint8_t[]){0x09}, 1, 0},
    {0x60, (const uint8_t[]){0x08}, 1, 0},
    {0x61, (const uint8_t[]){0x0f}, 1, 0},
    {0x62, (const uint8_t[]){0x0e}, 1, 0},
    {0x63, (const uint8_t[]){0x0d}, 1, 0},
    {0x64, (const uint8_t[]){0x0c}, 1, 0},
    {0x65, (const uint8_t[]){0x02}, 1, 0},
    {0x66, (const uint8_t[]){0x02}, 1, 0},
    {0x67, (const uint8_t[]){0x02}, 1, 0},
    {0x68, (const uint8_t[]){0x02}, 1, 0},
    {0x69, (const uint8_t[]){0x02}, 1, 0},
    {0x6a, (const uint8_t[]){0x02}, 1, 0},
    {0x6b, (const uint8_t[]){0x02}, 1, 0},
    {0x6c, (const uint8_t[]){0x02}, 1, 0},
    {0x6d, (const uint8_t[]){0x02}, 1, 0},
    {0x6e, (const uint8_t[]){0x02}, 1, 0},
    {0x6f, (const uint8_t[]){0x02}, 1, 0},
    {0x70, (const uint8_t[]){0x02}, 1, 0},
    {0x71, (const uint8_t[]){0x06}, 1, 0},
    {0x72, (const uint8_t[]){0x07}, 1, 0},
    {0x73, (const uint8_t[]){0x02}, 1, 0},
    {0x74, (const uint8_t[]){0x02}, 1, 0},
    {0x75, (const uint8_t[]){0x06}, 1, 0},
    {0x76, (const uint8_t[]){0x07}, 1, 0},
    {0x77, (const uint8_t[]){0x0e}, 1, 0},
    {0x78, (const uint8_t[]){0x0f}, 1, 0},
    {0x79, (const uint8_t[]){0x0c}, 1, 0},
    {0x7a, (const uint8_t[]){0x0d}, 1, 0},
    {0x7b, (const uint8_t[]){0x02}, 1, 0},
    {0x7c, (const uint8_t[]){0x02}, 1, 0},
    {0x7d, (const uint8_t[]){0x02}, 1, 0},
    {0x7e, (const uint8_t[]){0x02}, 1, 0},
    {0x7f, (const uint8_t[]){0x02}, 1, 0},
    {0x80, (const uint8_t[]){0x02}, 1, 0},
    {0x81, (const uint8_t[]){0x02}, 1, 0},
    {0x82, (const uint8_t[]){0x02}, 1, 0},
    {0x83, (const uint8_t[]){0x02}, 1, 0},
    {0x84, (const uint8_t[]){0x02}, 1, 0},
    {0x85, (const uint8_t[]){0x02}, 1, 0},
    {0x86, (const uint8_t[]){0x02}, 1, 0},
    {0x87, (const uint8_t[]){0x09}, 1, 0},
    {0x88, (const uint8_t[]){0x08}, 1, 0},
    {0x89, (const uint8_t[]){0x02}, 1, 0},
    {0x8A, (const uint8_t[]){0x02}, 1, 0},
    
    // Page 4
    {0xFF, (const uint8_t[]){0x98, 0x81, 0x04}, 3, 0},
    {0x6C, (const uint8_t[]){0x15}, 1, 0},  // Set VCORE voltage = 1.5V
    {0x6E, (const uint8_t[]){0x2a}, 1, 0},  // di_pwr_reg=0 for power mode 2A
    {0x6F, (const uint8_t[]){0x33}, 1, 0},  // pumping ratio VGH=3x VGL=-2x (reduced from 0x57 to reduce flickering)
    {0x3A, (const uint8_t[]){0x24}, 1, 0},  // POWER SAVING (changed from 0xa4 to 0x24 - less aggressive power saving)
    {0x8D, (const uint8_t[]){0x1a}, 1, 0},  // VGL clamp
    {0x87, (const uint8_t[]){0xba}, 1, 0},  // ESD
    {0x26, (const uint8_t[]){0x76}, 1, 0},
    {0xB2, (const uint8_t[]){0xd1}, 1, 0},
    
    // Page 1
    {0xFF, (const uint8_t[]){0x98, 0x81, 0x01}, 3, 0},
    {0x22, (const uint8_t[]){0x0A}, 1, 0},  // BGR, SS
    {0x31, (const uint8_t[]){0x02}, 1, 0},  // Column inversion (changed from 0x00 to 0x02 to reduce flicker)
    {0x53, (const uint8_t[]){0x35}, 1, 0},  // VCOM
    {0x55, (const uint8_t[]){0x50}, 1, 0},  // VCOM
    {0x50, (const uint8_t[]){0xaf}, 1, 0},  // VREG1OUT
    {0x51, (const uint8_t[]){0xaf}, 1, 0},  // VREG2OUT
    {0x60, (const uint8_t[]){0x20}, 1, 0},  // SDT (changed from 0x14 to 0x20 for higher frame rate)
    
    // Positive Gamma
    {0xA0, (const uint8_t[]){0x08}, 1, 0},
    {0xA1, (const uint8_t[]){0x1d}, 1, 0},
    {0xA2, (const uint8_t[]){0x2c}, 1, 0},
    {0xA3, (const uint8_t[]){0x14}, 1, 0},
    {0xA4, (const uint8_t[]){0x19}, 1, 0},
    {0xA5, (const uint8_t[]){0x2e}, 1, 0},
    {0xA6, (const uint8_t[]){0x22}, 1, 0},
    {0xA7, (const uint8_t[]){0x23}, 1, 0},
    {0xA8, (const uint8_t[]){0x97}, 1, 0},
    {0xA9, (const uint8_t[]){0x1e}, 1, 0},
    {0xAA, (const uint8_t[]){0x29}, 1, 0},
    {0xAB, (const uint8_t[]){0x7b}, 1, 0},
    {0xAC, (const uint8_t[]){0x18}, 1, 0},
    {0xAD, (const uint8_t[]){0x17}, 1, 0},
    {0xAE, (const uint8_t[]){0x4b}, 1, 0},
    {0xAF, (const uint8_t[]){0x1f}, 1, 0},
    {0xB0, (const uint8_t[]){0x27}, 1, 0},
    {0xB1, (const uint8_t[]){0x52}, 1, 0},
    {0xB2, (const uint8_t[]){0x63}, 1, 0},
    {0xB3, (const uint8_t[]){0x39}, 1, 0},
    
    // Negative Gamma
    {0xC0, (const uint8_t[]){0x08}, 1, 0},
    {0xC1, (const uint8_t[]){0x1d}, 1, 0},
    {0xC2, (const uint8_t[]){0x2c}, 1, 0},
    {0xC3, (const uint8_t[]){0x14}, 1, 0},
    {0xC4, (const uint8_t[]){0x19}, 1, 0},
    {0xC5, (const uint8_t[]){0x2e}, 1, 0},
    {0xC6, (const uint8_t[]){0x22}, 1, 0},
    {0xC7, (const uint8_t[]){0x23}, 1, 0},
    {0xC8, (const uint8_t[]){0x97}, 1, 0},
    {0xC9, (const uint8_t[]){0x1e}, 1, 0},
    {0xCA, (const uint8_t[]){0x29}, 1, 0},
    {0xCB, (const uint8_t[]){0x7b}, 1, 0},
    {0xCC, (const uint8_t[]){0x18}, 1, 0},
    {0xCD, (const uint8_t[]){0x17}, 1, 0},
    {0xCE, (const uint8_t[]){0x4b}, 1, 0},
    {0xCF, (const uint8_t[]){0x1f}, 1, 0},
    {0xD0, (const uint8_t[]){0x27}, 1, 0},
    {0xD1, (const uint8_t[]){0x52}, 1, 0},
    {0xD2, (const uint8_t[]){0x63}, 1, 0},
    {0xD3, (const uint8_t[]){0x39}, 1, 0},
    
    // Return to Page 0 (standard command set)
    {0xFF, (const uint8_t[]){0x98, 0x81, 0x00}, 3, 0},
    
    // Note: Driver handles SLPOUT (0x11) and DISPON (0x29) automatically
    {0x00, NULL, 0, 0x96}, // Delay 150ms
    {0x00, NULL, 0, 0x32}, // Delay 50ms
    {0x00, NULL, 0, 0},    // End of commands
};
#endif

// Forward declarations of static functions
static lv_display_t *display_lcd_init(void);
static lv_indev_t *esp_display_indev_init(lv_display_t *disp);
static esp_err_t esp_display_new_with_handles(const lcd_display_config_t *config, esp_lcd_handles_t *ret_handles);
static esp_err_t lcd_touch_new(const lcd_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch);
static esp_err_t lcd_brightness_init(void);

static bool IS_LCD_ENABLED = false; // Track if LCD is enabled

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

    /* Initialize backlight to 20 | Range: 0-31 */
    ESP_ERROR_CHECK_RETURN_ERR(lcd_brightness_set(20));

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

// TC358762 bridge configuration
static esp_err_t tc358762_send_command(uint8_t cmd, uint8_t data);
static esp_err_t lcd_enable_touch_panel(bool lcd_on, bool ctp_on);

/**
 * @brief Send a command to the TC358762 backlight controller
 * 
 * @return esp_err_t ESP_OK on success, or an error code on failure
 * 
 */
static esp_err_t tc358762_send_command(uint8_t cmd, uint8_t data)
{
    ESP_ERROR_CHECK_RETURN_ERR(esp_i2c_init());

    uint8_t data_to_send[2] = {cmd, data};

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = 100 * 1000,
        .device_address = TC358762_ADDR,
    };

    i2c_master_dev_handle_t dev_handle = NULL;
    i2c_master_bus_handle_t i2c_bus = esp_i2c_get_handle();
    if (i2c_bus == NULL) {
        ESP_LOGE(__func__, "I2C bus handle is NULL");
        return ESP_FAIL;
    }
    if (i2c_master_bus_add_device(i2c_bus, &i2c_dev_conf, &dev_handle) != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to add I2C device at address 0x%02X", TC358762_ADDR);
        return ESP_FAIL;
    }

    esp_err_t ret = i2c_master_transmit(dev_handle, data_to_send, sizeof(data_to_send), 50);
    if (ret != ESP_OK)
    {
        i2c_master_bus_rm_device(dev_handle);
        return ret;
    }

    i2c_master_bus_rm_device(dev_handle);

    return ESP_OK;
}

/**
 * @brief Enable or disable the touch panel via the TC358762 bridge
 * 
 * @param lcd_on true to enable LCD, false to disable
 * @param ctp_on true to enable touch panel, false to disable
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t lcd_enable_touch_panel(bool lcd_on, bool ctp_on)
{
    IS_LCD_ENABLED = lcd_on;
    uint8_t val = 0;
    if (lcd_on) val |= RPI_TOUCH_V2_LCD_RESET_BIT; // deassert LCD reset
    if (ctp_on) val |= RPI_TOUCH_V2_CTP_RESET_BIT; // deassert CTP reset
    return tc358762_send_command(RPI_TOUCH_V2_REG_POWERON, val);
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
 * @brief Set the LCD brightness
 * 
 * @param brightness_percent Brightness level (0-100)
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t lcd_brightness_set(int brightness)
{
    if (!IS_LCD_ENABLED) return ESP_FAIL;

    if (brightness > 31) {
        brightness = 31;
    }
    if (brightness < 0) {
        brightness = 0;
    }

    uint8_t val = RPI_TOUCH_V2_PWM_ENABLE | (brightness & RPI_TOUCH_V2_PWM_VALUE_MASK);
    return tc358762_send_command(RPI_TOUCH_V2_REG_PWM, val);
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
    return lcd_brightness_set(31);
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
    ESP_RETURN_ON_ERROR(esp_enable_dsi_phy_power(), __func__, "DSI PHY power failed");
    ESP_RETURN_ON_ERROR(lcd_enable_touch_panel(true, true), __func__, "Enable lcd power failed");
    
    /* create MIPI DSI bus first, it will initialize the DSI PHY as well */
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = LCD_MIPI_DSI_LANE_NUM,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = LCD_MIPI_DSI_LANE_BITRATE_MBPS,
        // .flags.non_continuous_clock = 1,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus), __func__, "New DSI bus init failed");

    ESP_LOGI(__func__, "Install (RPi 5inch) MIPI DSI LCD control panel");
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
    esp_lcd_dpi_panel_config_t dpi_config = RPI_5INCH_TOUCH_DISPLAY_V2_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#else
    esp_lcd_dpi_panel_config_t dpi_config = RPI_5INCH_TOUCH_DISPLAY_V2_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
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
    vendor_config.init_cmds = rpi_custom_ili9881c_init;
    vendor_config.init_cmds_size = sizeof(rpi_custom_ili9881c_init) / sizeof(rpi_custom_ili9881c_init[0]);
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

    // // After esp_lcd_panel_init(disp_panel);
    // vTaskDelay(pdMS_TO_TICKS(120));  // Post-init settle

    // // Minimal DSI/video cmds for Powertip PH720128T003 (from RPi kernel port)
    // const uint8_t dsi_cmds[][2] = {
    //     {0xBA, 0x03},  // DSI 2-lane mode
    //     {0xBB, 0x10},  // Video mode (non-burst)
    //     {0x35, 0x00},  // Tearing effect line off
    //     {0x36, 0x00},  // Memory access: portrait, no mirror (adjust to 0x08 for row inversion if blur)
    //     {0xB4, 0x02},  // Display inversion off
    //     {0x3A, 0x55},  // Pixel format RGB565
    //     {0x11, 0x00},  // Sleep out (redundant but safe)
    //     {0x29, 0x00},  // Display on
    // };
    // for (int i = 0; i < sizeof(dsi_cmds)/sizeof(dsi_cmds[0]); i++) {
    //     esp_lcd_panel_io_tx_param(io, dsi_cmds[i][0], &dsi_cmds[i][1], 1);
    //     vTaskDelay(pdMS_TO_TICKS(20));  // 20ms per cmd
    // }
    // lcd_brightness_set(20);  // Via expander, after on

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

    tp_io_config.scl_speed_hz = 400000;

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
 * @brief Get the touch input handle
 * 
 * @return esp_lcd_touch_handle_t Touch handle
 */
esp_lcd_touch_handle_t display_touch_get_handle(void)
{
    return s_touch_handle;
}


#endif // CONFIG_LCD_RPI_5INCH_TOUCH_DISPLAY_V2
// End of file rpi_5inch_touch_v2.c