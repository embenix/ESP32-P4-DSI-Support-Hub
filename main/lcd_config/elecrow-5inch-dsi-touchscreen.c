/**
 * @file elecrow-5inch-dsi-touchscreen.c
 * @author Yasir K. Qureshi (embenix.com) / Contributed by Nobby
 * @brief Elecrow 5-inch DSI Touch Display driver (800x480)
 * @version 0.1
 *
 * This display utilizes a TC358762 DSI-to-DPI bridge and an ATTINY88
 * for power/reset sequencing over I2C, identical to the RPi 7" V1 architecture.
 */

#include "sdkconfig.h"

#if CONFIG_ELECROW_5INCH_DSI_TOUCHSCREEN == 1

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "init_lcd.h"
#include "esp_lcd_touch_ft5x06.h"
#include "esp_lcd_mipi_dsi.h"
#include "hal/mipi_dsi_hal.h"
#include "hal/mipi_dsi_types.h"
#include "hal/mipi_dsi_ll.h"

static const char *LCD_NAME = "Elecrow 5inch DSI Touch Display";
static const char *TAG      = "Elecrow 5\" DSI";

/* ----------------------------- Display configuration ----------------------------- */
#define LCD_COLOR_FORMAT_RGB565         (1)
#define LCD_COLOR_FORMAT_RGB888         (2)
#define USE_LCD_COLOR_FORMAT_RGB888     (1)
#define USE_LVGL_AVOID_TEAR             (0)
#define USE_LVGL_FULL_REFRESH           (0)
#define USE_LVGL_DIRECT_MODE            (0)

#define LCD_RGB_ELEMENT_ORDER           (LCD_RGB_ELEMENT_ORDER_RGB)

#define LCD_H_RES                       (800)
#define LCD_V_RES                       (480)
#define LCD_MIPI_DSI_LANE_BITRATE_MBPS  (600)
#define LCD_MIPI_DSI_LANE_NUM           (1)

// Full-screen LVGL draw buffers in SPIRAM + double buffer (partial render mode; faster than avoid_tearing path).
#define LCD_DRAW_BUFF_SIZE              (LCD_H_RES * LCD_V_RES)
#define LCD_DRAW_BUFF_DOUBLE            (1)
#define EN_LCD_BUFF_DMA                 (0)
#define EN_LCD_BUFF_SPIRAM              (1)
#define EN_LCD_SW_ROTATE                (1)
#define LCD_DPI_BUFFER_NUMS             (1)

// Elecrow 5-inch Active timing profile (Matches RPi Mode)
#define ELECROW_MODE_HFP                (210)
#define ELECROW_MODE_HSW                (2)
#define ELECROW_MODE_HBP                (46)
#define ELECROW_MODE_VFP                (22)
#define ELECROW_MODE_VSW                (20)
#define ELECROW_MODE_VBP                (4)

// FT5x06 orientation (same TC358762 + ATTINY88 class as RPi 7" V1 / Luckfox 5")
#define LCD_ROT_SWAP_XY                 (0)
#define LCD_ROT_MIRROR_X                (1)
#define LCD_ROT_MIRROR_Y                (1)

#define ELECROW_5INCH_DSI_CONFIG(px_format) \
    {                                                      \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,       \
        .dpi_clock_freq_mhz = 25.98,                       \
        .virtual_channel = 0,                              \
        .in_color_format  = (px_format),                   \
        .out_color_format = LCD_COLOR_FMT_RGB888,          \
        .num_fbs = 1,                                      \
        .video_timing = {                                  \
            .h_size = LCD_H_RES,                           \
            .v_size = LCD_V_RES,                           \
            .hsync_back_porch = ELECROW_MODE_HBP,          \
            .hsync_pulse_width = ELECROW_MODE_HSW,         \
            .hsync_front_porch = ELECROW_MODE_HFP,         \
            .vsync_back_porch = ELECROW_MODE_VBP,          \
            .vsync_pulse_width = ELECROW_MODE_VSW,         \
            .vsync_front_porch = ELECROW_MODE_VFP,         \
        },                                                 \
        .flags = {                                         \
            .disable_lp = 0,                               \
        },                                                 \
    }

#define LCD_RST_GPIO                    (-1)
#define LCD_TOUCH_RST_GPIO              (-1)
#define LCD_TOUCH_INT_GPIO              (-1)
#define LCD_DATA_BIGENDIAN              (0)

// Atmel ATTINY88-MUR configuration (I2C control)
#define ATTINY88_ADDR                   (0x45)
#define ATTINY88_I2C_SCL_HZ             (400000)
#define ATTINY88_I2C_TIMEOUT_MS         (50)
#define REG_ID                          (0x80)
#define REG_PORTA                       (0x81)
#define REG_PORTB                       (0x82)
#define REG_PORTC                       (0x83)
#define REG_PWM                         (0x86)
#define REG_ADDR_L                      (0x8c)
#define REG_ADDR_H                      (0x8d)
#define REG_WRITE_DATA_H                (0x90)
#define REG_WRITE_DATA_L                (0x91)

// REG_PORTA bits
#define PA_LCD_LR                       BIT(2)
// REG_PORTB bits
#define PB_LCD_MAIN                     BIT(7)
// REG_PORTC bits
#define PC_LED_EN                       BIT(0)
#define PC_RST_TP_N                     BIT(1)
#define PC_RST_LCD_N                    BIT(2)
#define PC_RST_BRIDGE_N                 BIT(3)

// FT5406 touch controller
#define FT5406_ADDR                     (0x38)

// TC358762XBG DSI-to-DPI bridge registers
#define TC_DSI_LANEENABLE               (0x0210)
#define TC_PPI_D0S_CLRSIPOCOUNT         (0x0164)
#define TC_PPI_D1S_CLRSIPOCOUNT         (0x0168)
#define TC_PPI_D0S_ATMR                 (0x0144)
#define TC_PPI_D1S_ATMR                 (0x0148)
#define TC_PPI_LPTXTIMECNT              (0x0114)
#define TC_SPICMR                       (0x0450)
#define TC_LCDCTRL                      (0x0420)
#define TC_SYSCTRL                      (0x0464)
#define TC_LCD_HS_HBP                   (0x0424)
#define TC_LCD_HDISP_HFP                (0x0428)
#define TC_LCD_VS_VBP                   (0x042c)
#define TC_LCD_VDISP_VFP                (0x0430)
#define TC_PPI_STARTPPI                 (0x0104)
#define TC_DSI_STARTDSI                 (0x0204)

static esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
static lv_indev_t *disp_indev = NULL;
static lv_display_t *disp = NULL;
static esp_lcd_touch_handle_t s_touch_handle = NULL;
static bool IS_LCD_ENABLED = false;

// Function prototypes
static lv_display_t *display_lcd_init(void);
static lv_indev_t *esp_display_indev_init(lv_display_t *disp);
static esp_err_t esp_display_new_with_handles(const lcd_display_config_t *config, esp_lcd_handles_t *ret_handles);
static esp_err_t lcd_touch_new(const lcd_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch);
static esp_err_t lcd_brightness_init(void);
static esp_err_t attiny88_read_register(uint8_t reg, uint8_t *data);
static esp_err_t attiny88_write_register(uint8_t reg, uint8_t val);
static esp_err_t tc358762_bridge_init(esp_lcd_dsi_bus_handle_t bus);
static esp_err_t lcd_power_on_sequence(void);
static esp_err_t lcd_release_bridge_reset_and_wake(void);
static esp_err_t lcd_release_touch_reset(void);

typedef struct {
    int bus_id;
    mipi_dsi_hal_context_t hal;
} rpi_dsi_bus_priv_t;

static void tc358762_reg_write(esp_lcd_dsi_bus_handle_t bus, uint16_t reg, uint32_t val)
{
    rpi_dsi_bus_priv_t *priv = (rpi_dsi_bus_priv_t *)bus;
    uint8_t payload[6] = {
        (reg >>  0) & 0xFF, (reg >>  8) & 0xFF,
        (val >>  0) & 0xFF, (val >>  8) & 0xFF,
        (val >> 16) & 0xFF, (val >> 24) & 0xFF,
    };
    mipi_dsi_hal_host_gen_write_long_packet(&priv->hal, 0, MIPI_DSI_DT_GENERIC_LONG_WRITE, payload, sizeof(payload));
}

static esp_err_t tc358762_bridge_init(esp_lcd_dsi_bus_handle_t bus)
{
    ESP_LOGI(__func__, "Starting TC358762 bridge configuration for Elecrow 5-inch");

    tc358762_reg_write(bus, TC_DSI_LANEENABLE, BIT(0) | BIT(1));
    tc358762_reg_write(bus, TC_PPI_D0S_CLRSIPOCOUNT, 0x05);
    tc358762_reg_write(bus, TC_PPI_D1S_CLRSIPOCOUNT, 0x05);
    tc358762_reg_write(bus, TC_PPI_D0S_ATMR, 0x00);
    tc358762_reg_write(bus, TC_PPI_D1S_ATMR, 0x00);
    tc358762_reg_write(bus, TC_PPI_LPTXTIMECNT, 0x03);

    tc358762_reg_write(bus, TC_SPICMR, 0x00);
    tc358762_reg_write(bus, TC_LCDCTRL, 0x00100150);
    tc358762_reg_write(bus, TC_SYSCTRL, 0x040f);

    tc358762_reg_write(bus, TC_LCD_HS_HBP,    (ELECROW_MODE_HBP << 16) | ELECROW_MODE_HSW);
    tc358762_reg_write(bus, TC_LCD_HDISP_HFP, (ELECROW_MODE_HFP << 16) | LCD_H_RES);
    tc358762_reg_write(bus, TC_LCD_VS_VBP,    (ELECROW_MODE_VBP << 16) | ELECROW_MODE_VSW);
    tc358762_reg_write(bus, TC_LCD_VDISP_VFP, (ELECROW_MODE_VFP << 16) | LCD_V_RES);

    vTaskDelay(pdMS_TO_TICKS(100));

    tc358762_reg_write(bus, TC_PPI_STARTPPI, 0x01);
    tc358762_reg_write(bus, TC_DSI_STARTDSI, 0x01);

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(__func__, "TC358762 bridge configured");
    return ESP_OK;
}

#if CONFIG_PRINT_TOUCH_EVENTS
static void touch_logger_task(void *arg)
{
    esp_lcd_touch_handle_t tp = display_touch_get_handle();
    const uint8_t max = 5;
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
                    chalk_printf(CHALK_WHITE, "%s", (i < count - 1) ? "," : "\n");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
#endif

esp_err_t init_lcd(void)
{
    esp_err_t ret = ESP_OK;

    disp = display_lcd_init();
    ESP_NULL_CHECK(disp, ESP_FAIL);

    disp_indev = esp_display_indev_init(disp);
    ESP_NULL_CHECK(disp_indev, ESP_FAIL);

    ESP_ERROR_CHECK_RETURN_ERR(lcd_brightness_set(255));
    ESP_LOGI(__func__, "%s initialized successfully.", LCD_NAME);

#if CONFIG_PRINT_TOUCH_EVENTS
    xTaskCreate(touch_logger_task, "touch_logger", 4 * 1024, NULL, 2, NULL);
#endif

    return ret;
}

esp_err_t deinit_lcd(void)
{
    return ESP_OK;
}

lv_indev_t *display_get_input_dev(void)
{
    return disp_indev;
}

lv_display_t *display_get_handle(void)
{
    return disp;
}

static esp_err_t attiny88_read_register(uint8_t reg, uint8_t *data)
{
    ESP_ERROR_CHECK_RETURN_ERR(esp_i2c_init());

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = ATTINY88_I2C_SCL_HZ,
        .device_address = ATTINY88_ADDR,
    };

    i2c_master_dev_handle_t dev_handle = NULL;
    i2c_master_bus_handle_t i2c_bus = esp_i2c_get_handle();
    if (i2c_bus == NULL) return ESP_FAIL;

    if (i2c_master_bus_add_device(i2c_bus, &i2c_dev_conf, &dev_handle) != ESP_OK) return ESP_FAIL;

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, data, 1, ATTINY88_I2C_TIMEOUT_MS);
    i2c_master_bus_rm_device(dev_handle);

    return ret;
}

static esp_err_t attiny88_write_register(uint8_t reg, uint8_t val)
{
    ESP_ERROR_CHECK_RETURN_ERR(esp_i2c_init());

    uint8_t data_to_send[2] = {reg, val};

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = ATTINY88_I2C_SCL_HZ,
        .device_address = ATTINY88_ADDR,
    };

    i2c_master_dev_handle_t dev_handle = NULL;
    i2c_master_bus_handle_t i2c_bus = esp_i2c_get_handle();
    if (i2c_bus == NULL) return ESP_FAIL;

    if (i2c_master_bus_add_device(i2c_bus, &i2c_dev_conf, &dev_handle) != ESP_OK) return ESP_FAIL;

    esp_err_t ret = i2c_master_transmit(dev_handle, data_to_send, sizeof(data_to_send), ATTINY88_I2C_TIMEOUT_MS);
    i2c_master_bus_rm_device(dev_handle);

    return ret;
}

static esp_err_t lcd_power_on_sequence(void)
{
    IS_LCD_ENABLED = false;

    ESP_RETURN_ON_ERROR(attiny88_write_register(REG_PORTC, 0x00), __func__, "REG_PORTC=0 failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_RETURN_ON_ERROR(attiny88_write_register(REG_PORTA, PA_LCD_LR), __func__, "REG_PORTA failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_RETURN_ON_ERROR(attiny88_write_register(REG_PORTB, PB_LCD_MAIN), __func__, "REG_PORTB failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_RETURN_ON_ERROR(attiny88_write_register(REG_PORTC, PC_LED_EN), __func__, "REG_PORTC=LED failed");
    vTaskDelay(pdMS_TO_TICKS(80));

    IS_LCD_ENABLED = true;
    ESP_LOGI(__func__, "ATTINY88 power-on sequence complete");
    return ESP_OK;
}

static esp_err_t lcd_release_bridge_reset_and_wake(void)
{
    ESP_RETURN_ON_ERROR(attiny88_write_register(REG_PORTC, PC_LED_EN | PC_RST_LCD_N | PC_RST_BRIDGE_N), __func__, "REG_PORTC=BRIDGE_RST_RELEASE failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_RETURN_ON_ERROR(attiny88_write_register(REG_ADDR_H,  0x04), __func__, "ADDR_H failed");
    vTaskDelay(pdMS_TO_TICKS(8));
    ESP_RETURN_ON_ERROR(attiny88_write_register(REG_ADDR_L,  0x7c), __func__, "ADDR_L failed");
    vTaskDelay(pdMS_TO_TICKS(8));
    ESP_RETURN_ON_ERROR(attiny88_write_register(REG_WRITE_DATA_H, 0x00), __func__, "WR_DATA_H failed");
    vTaskDelay(pdMS_TO_TICKS(8));
    ESP_RETURN_ON_ERROR(attiny88_write_register(REG_WRITE_DATA_L, 0x00), __func__, "WR_DATA_L failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(__func__, "TC358762 reset released");
    return ESP_OK;
}

static esp_err_t lcd_release_touch_reset(void)
{
    ESP_RETURN_ON_ERROR(attiny88_write_register(REG_PORTC, PC_LED_EN | PC_RST_TP_N | PC_RST_LCD_N | PC_RST_BRIDGE_N), __func__, "REG_PORTC=TOUCH_RST_RELEASE failed");
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

static esp_err_t lcd_brightness_init(void)
{
    esp_i2c_init();

    uint8_t fw_id = 0;
    if (attiny88_read_register(REG_ID, &fw_id) == ESP_OK) {
        ESP_LOGI(TAG, "ATTINY88 Firmware ID: 0x%02X", fw_id);
    }

    return ESP_OK;
}

esp_err_t lcd_brightness_set(int brightness)
{
    if (!IS_LCD_ENABLED) return ESP_FAIL;
    if (brightness > 255) brightness = 255;
    if (brightness < 0) brightness = 0;
    return attiny88_write_register(REG_PWM, (uint8_t)brightness);
}

esp_err_t lcd_backlight_off(void)
{
    return lcd_brightness_set(0);
}

esp_err_t lcd_backlight_on(void)
{
    return lcd_brightness_set(255);
}

static lv_display_t *display_lcd_init(void)
{
    esp_lcd_handles_t lcd_panels;
    ESP_ERROR_CHECK_RETURN_NULL(esp_display_new_with_handles(NULL, &lcd_panels));

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_panels.io,
        .panel_handle = lcd_panels.panel,
        .control_handle = lcd_panels.control,
        .buffer_size = LCD_DRAW_BUFF_SIZE,
        .double_buffer = LCD_DRAW_BUFF_DOUBLE,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
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
            .sw_rotate = EN_LCD_SW_ROTATE,
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

static esp_err_t esp_display_new_with_handles(const lcd_display_config_t *config, esp_lcd_handles_t *ret_handles)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_ERROR(lcd_brightness_init(), __func__, "Brightness init failed");
    ESP_RETURN_ON_ERROR(esp_enable_dsi_phy_power(), __func__, "DSI PHY power failed");
    ESP_RETURN_ON_ERROR(lcd_power_on_sequence(), __func__, "Power-on sequence failed");

    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = LCD_MIPI_DSI_LANE_NUM,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = LCD_MIPI_DSI_LANE_BITRATE_MBPS,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus), __func__, "New DSI bus init failed");

    esp_lcd_panel_io_handle_t dbi_io = NULL;
    esp_lcd_dbi_io_config_t lp_dbi_cfg = { .virtual_channel = 0, .lcd_cmd_bits = 8, .lcd_param_bits = 8 };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &lp_dbi_cfg, &dbi_io), __func__, "LP mode setup failed");
    esp_lcd_panel_io_del(dbi_io);

    esp_lcd_panel_handle_t disp_panel = NULL;

#if USE_LCD_COLOR_FORMAT_RGB888
    esp_lcd_dpi_panel_config_t dpi_config = ELECROW_5INCH_DSI_CONFIG(LCD_COLOR_FMT_RGB888);
#else
    esp_lcd_dpi_panel_config_t dpi_config = ELECROW_5INCH_DSI_CONFIG(LCD_COLOR_FMT_RGB565);
#endif

    dpi_config.num_fbs = LCD_DPI_BUFFER_NUMS;

    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_dpi(mipi_dsi_bus, &dpi_config, &disp_panel), err, __func__, "New DPI panel failed");

    {
        rpi_dsi_bus_priv_t *_priv = (rpi_dsi_bus_priv_t *)mipi_dsi_bus;
        mipi_dsi_host_ll_dpi_set_video_burst_type(_priv->hal.host, MIPI_DSI_LL_VIDEO_NON_BURST_WITH_SYNC_PULSES);
        mipi_dsi_host_ll_dpi_enable_frame_ack(_priv->hal.host, false);
    }

    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(disp_panel), err, __func__, "LCD panel init failed");

    {
        rpi_dsi_bus_priv_t *_priv = (rpi_dsi_bus_priv_t *)mipi_dsi_bus;
        mipi_dsi_host_ll_set_clock_lane_state(_priv->hal.host, MIPI_DSI_LL_CLOCK_LANE_STATE_HS);
        mipi_dsi_host_ll_enable_cmd_ack(_priv->hal.host, false);
    }

    ESP_GOTO_ON_ERROR(lcd_release_bridge_reset_and_wake(), err, __func__, "Bridge reset release failed");
    ESP_GOTO_ON_ERROR(tc358762_bridge_init(mipi_dsi_bus), err, __func__, "TC358762 bridge init failed");

    attiny88_write_register(REG_PWM, 255);

    ret_handles->io = NULL;
    ret_handles->mipi_dsi_bus = mipi_dsi_bus;
    ret_handles->panel = disp_panel;
    ret_handles->control = NULL;

    ESP_LOGI(__func__, "%s initialized", LCD_NAME);
    return ret;

err:
    if (disp_panel) esp_lcd_panel_del(disp_panel);
    if (mipi_dsi_bus) esp_lcd_del_dsi_bus(mipi_dsi_bus);
    return ret;
}

static esp_err_t lcd_touch_new(const lcd_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    ESP_ERROR_CHECK_RETURN_ERR(esp_i2c_init());

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = LCD_TOUCH_RST_GPIO,
        .int_gpio_num = LCD_TOUCH_INT_GPIO,
        .levels = { .reset = 0, .interrupt = 0, },
        .flags = {
            .swap_xy = LCD_ROT_SWAP_XY,
            .mirror_x = LCD_ROT_MIRROR_X,
            .mirror_y = LCD_ROT_MIRROR_Y,
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = {
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_FT5x06_ADDRESS,
        .control_phase_bytes = 1,
        .dc_bit_offset = 0,
        .lcd_cmd_bits = 8,
        .scl_speed_hz = 400000,
        .flags = {
            .disable_control_phase = 1,
        },
    };

    i2c_master_bus_handle_t i2c_bus = esp_i2c_get_handle();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_bus, &tp_io_config, &tp_io_handle), __func__, "");

    esp_err_t ret = esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, ret_touch);
    if (ret == ESP_OK) {
        s_touch_handle = *ret_touch;
    }
    return ret;
}

static lv_indev_t *esp_display_indev_init(lv_display_t *disp)
{
    esp_lcd_touch_handle_t tp;

    ESP_ERROR_CHECK_RETURN_NULL(lcd_release_touch_reset());
    ESP_ERROR_CHECK_RETURN_NULL(lcd_touch_new(NULL, &tp));

    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp,
        .handle = tp,
    };

    return lvgl_port_add_touch(&touch_cfg);
}

esp_lcd_touch_handle_t display_touch_get_handle(void)
{
    return s_touch_handle;
}

#endif // CONFIG_ELECROW_5INCH_DSI_TOUCHSCREEN
