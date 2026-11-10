/**
 * @file amelin-ek79007-7inch-touch-display.c
 * @author Yasir K. Qureshi (embenix.com)
 * @brief Amelin EK79007 7inch Touch Display driver
 * @version 0.2
 * @date 2025-09-22
 * 
 * @copyright Copyright (c) 2025
 * 
 * @link https://www.vip-lcd.com/7-Inch-LCD-Touch-Display-Screen-1024-600-LVDS-Interface-with-Touch-Panel-7-0-Inch-Lcd-Module-pd591986658.html @endlink
 * 
 */

// This file is included only if the Amelin EK79007 7inch Touch Display
// is enabled in the project configuration.
#include "sdkconfig.h"

#if CONFIG_AMELIN_EK79007_7INCH_TOUCH_DISPLAY == 1

#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_idf_version.h"
#include "init_lcd.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_lcd_ek79007.h"

const char *LCD_NAME = "Amelin EK79007 7inch Touch Display";
const char *TAG      = "Amelin EK79007 7\" DSI";

/* LCD color formats and other configurations */
#define LCD_COLOR_FORMAT_RGB565         (1)
#define LCD_COLOR_FORMAT_RGB888         (2)  // Works but very slow | better to use RGB565
#define USE_LCD_COLOR_FORMAT_RGB888     (0)  // Set to 1 to use RGB888, 0 for RGB565
#define USE_LVGL_AVOID_TEAR             (0)  // Set to 1 to enable avoid tearing feature in LVGL
#define USE_LVGL_FULL_REFRESH           (0)  // Set to 1 to enable full refresh feature in LVGL
#define USE_LVGL_DIRECT_MODE            (0)  // Set to 1 to enable direct mode feature in LVGL

#if CONFIG_ESPRESSIF_ESP32P4_FUNCTION_EV_BOARD
#define LCD_RGB_ELEMENT_ORDER           (LCD_RGB_ELEMENT_ORDER_RGB)
#else
#define LCD_RGB_ELEMENT_ORDER           (LCD_RGB_ELEMENT_ORDER_BGR)
#endif

#define LCD_H_RES                       (1024)
#define LCD_V_RES                       (600)
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

#define LCD_RST_GPIO                    CONFIG_AMELIN_LCD_RESET_GPIO
#define LCD_TOUCH_RST_GPIO              (-1)
#define LCD_TOUCH_INT_GPIO              (-1) // GPIO for LCD touch interrupt
#define LCD_DATA_BIGENDIAN              (0)

#if CONFIG_ESPRESSIF_ESP32P4_FUNCTION_EV_BOARD
#define BL_LEDC_TIMER                   LEDC_TIMER_1
#define BL_LEDC_CHANNEL                 LEDC_CHANNEL_1
#else
#define BL_LEDC_TIMER                   LEDC_TIMER_0
#define BL_LEDC_CHANNEL                 LEDC_CHANNEL_0
#endif
#define BL_LEDC_MODE                    LEDC_LOW_SPEED_MODE
#define BL_LEDC_FREQ_HZ                 (5000)
#define BL_LEDC_DUTY_RES                LEDC_TIMER_10_BIT

// Goodix GT911 typical 7-bit I2C address
#define GT911_ADDR                      (0x5D)

static esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
static lv_indev_t *disp_indev = NULL;
static lv_display_t *disp = NULL;
static esp_lcd_touch_handle_t s_touch_handle = NULL;

// Function prototypes
static lv_display_t *display_lcd_init(void);
static lv_indev_t *esp_display_indev_init(lv_display_t *disp);
static esp_err_t esp_display_new_with_handles(const lcd_display_config_t *config, esp_lcd_handles_t *ret_handles);
static esp_err_t lcd_touch_new(const lcd_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch);
static esp_err_t lcd_brightness_init(void);

static bool backlight_initialized = false;

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
            bool touched = false;
            lvgl_port_lock(0);
            esp_err_t err = esp_lcd_touch_read_data(tp);
            if (err == ESP_OK) {
                touched = esp_lcd_touch_get_coordinates(tp, xs, ys, NULL, &count, max);
            }
            lvgl_port_unlock();
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Touch logger read failed: %s", esp_err_to_name(err));
            } else if (touched) {
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
 * @brief Initialize LCD brightness control
 * 
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t lcd_brightness_init(void)
{
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(CONFIG_AMELIN_BACKLIGHT_PWM_GPIO),
                        ESP_ERR_INVALID_ARG, TAG, "Configure a valid backlight PWM GPIO in menuconfig");
    ESP_RETURN_ON_FALSE(CONFIG_AMELIN_BACKLIGHT_PWM_GPIO != I2C_SDA_GPIO &&
                        CONFIG_AMELIN_BACKLIGHT_PWM_GPIO != I2C_SCL_GPIO,
                        ESP_ERR_INVALID_ARG, TAG, "Backlight PWM GPIO conflicts with I2C");
    ESP_RETURN_ON_FALSE(LCD_RST_GPIO == -1 ||
                        (GPIO_IS_VALID_OUTPUT_GPIO(LCD_RST_GPIO) &&
                         LCD_RST_GPIO != I2C_SDA_GPIO && LCD_RST_GPIO != I2C_SCL_GPIO &&
                         LCD_RST_GPIO != CONFIG_AMELIN_BACKLIGHT_PWM_GPIO),
                        ESP_ERR_INVALID_ARG, TAG, "Invalid or conflicting LCD reset GPIO");

    const ledc_timer_config_t timer = {
        .speed_mode = BL_LEDC_MODE,
        .timer_num = BL_LEDC_TIMER,
        .freq_hz = BL_LEDC_FREQ_HZ,
        .duty_resolution = BL_LEDC_DUTY_RES,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "Backlight timer configuration failed");

    const ledc_channel_config_t channel = {
        .gpio_num = CONFIG_AMELIN_BACKLIGHT_PWM_GPIO,
        .speed_mode = BL_LEDC_MODE,
        .channel = BL_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BL_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), TAG, "Backlight channel configuration failed");
    backlight_initialized = true;
    return ESP_OK;
}

/**
 * @brief Set the LCD brightness
 * 
 * @param brightness Brightness level (0-100)
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t lcd_brightness_set(int brightness)
{
    ESP_RETURN_ON_FALSE(backlight_initialized, ESP_ERR_INVALID_STATE, TAG, "Backlight is not initialized");
    if (brightness > 100) brightness = 100;
    if (brightness < 0) {
        brightness = 0;
    }

    uint32_t duty = (((1U << BL_LEDC_DUTY_RES) - 1U) * brightness) / 100U;
    ESP_RETURN_ON_ERROR(ledc_set_duty(BL_LEDC_MODE, BL_LEDC_CHANNEL, duty), TAG, "Set backlight duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(BL_LEDC_MODE, BL_LEDC_CHANNEL), TAG, "Update backlight duty failed");
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
    ESP_RETURN_ON_ERROR(esp_enable_dsi_phy_power(), __func__, "DSI PHY power failed");

    /* create MIPI DSI bus first, it will initialize the DSI PHY as well */
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = LCD_MIPI_DSI_LANE_NUM,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = LCD_MIPI_DSI_LANE_BITRATE_MBPS,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus), __func__, "New DSI bus init failed");

    ESP_LOGI(__func__, "Install (Amelin EK79007 7inch) MIPI DSI LCD control panel");
    // we use DBI interface to send LCD commands and parameters
    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_handle_t disp_panel = NULL;
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &io), err, __func__, "New panel IO failed");

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#if USE_LCD_COLOR_FORMAT_RGB888
    esp_lcd_dpi_panel_config_t dpi_config = EK79007_1024_600_PANEL_60HZ_CONFIG_CF(LCD_COLOR_FMT_RGB888);
#else
    esp_lcd_dpi_panel_config_t dpi_config = EK79007_1024_600_PANEL_60HZ_CONFIG_CF(LCD_COLOR_FMT_RGB565);
#endif
#else
#if USE_LCD_COLOR_FORMAT_RGB888
    esp_lcd_dpi_panel_config_t dpi_config = EK79007_1024_600_PANEL_60HZ_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#else
    esp_lcd_dpi_panel_config_t dpi_config = EK79007_1024_600_PANEL_60HZ_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#endif
#endif

    dpi_config.num_fbs = LCD_DPI_BUFFER_NUMS;

    ek79007_vendor_config_t vendor_config = {
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
            .lane_num = LCD_MIPI_DSI_LANE_NUM,
        },
    };
    esp_lcd_panel_dev_config_t lcd_dev_config = {

#if USE_LCD_COLOR_FORMAT_RGB888
        .bits_per_pixel = 24,
#else
        .bits_per_pixel = 16,
#endif
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER,
        .reset_gpio_num = LCD_RST_GPIO,
        .vendor_config = &vendor_config,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_ek79007(io, &lcd_dev_config, &disp_panel), err, __func__, "New EK79007 panel failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(disp_panel), err, __func__, "LCD panel reset failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(disp_panel), err, __func__, "LCD panel init failed");

#if USE_LCD_COLOR_FORMAT_RGB888 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    // Enable DMA2D for RGB888 color format
    ESP_GOTO_ON_ERROR(esp_lcd_dpi_panel_enable_dma2d(disp_panel), err, __func__, "Enable DMA2D failed");
#endif
    // EK79007 initialization wakes the panel; its DPI handle has no disp_on_off callback.

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

static void lcd_touch_bound_coordinates(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y,
                                        uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    (void)strength;
    for (uint8_t i = 0; i < *point_num && i < max_point_num; i++) {
        if (x[i] > tp->config.x_max || y[i] > tp->config.y_max) {
            ESP_LOGW(TAG, "Touch coordinate out of range: x=%u y=%u", x[i], y[i]);
            if (x[i] > tp->config.x_max) x[i] = tp->config.x_max;
            if (y[i] > tp->config.y_max) y[i] = tp->config.y_max;
        }
    }
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
        .x_max = LCD_H_RES - 1,
        .y_max = LCD_V_RES - 1,

        .rst_gpio_num = LCD_TOUCH_RST_GPIO,
        .int_gpio_num = LCD_TOUCH_INT_GPIO,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 1,
            .mirror_y = 1,
        },
        .process_coordinates = lcd_touch_bound_coordinates,
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = {
        .dev_addr = GT911_ADDR,
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
 * @brief Get the touch handle
 * 
 * @return esp_lcd_touch_handle_t Touch handle, or NULL if not initialized
 */
esp_lcd_touch_handle_t display_touch_get_handle(void)
{
    return s_touch_handle;
}

#endif // CONFIG_AMELIN_EK79007_7INCH_TOUCH_DISPLAY
// End of file amelin-ek79007-7inch-touch-display.c