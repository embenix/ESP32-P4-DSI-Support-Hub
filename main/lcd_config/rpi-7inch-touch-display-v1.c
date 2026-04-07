/**
 * @file rpi-7inch-touch-display-v1.c
 * @author Yasir K. Qureshi (embenix.com)
 * @brief Raspberry Pi 7inch Touch Display V1 driver
 * @version 0.1
 * @date 2026-04-06
 * 
 * @copyright Copyright (c) 2026
 * 
 * @link https://www.raspberrypi.com/products/raspberry-pi-touch-display/ @endlink
 */

// This file is included only if the Raspberry Pi 7inch Touch Display V1
// is enabled in the project configuration.
#include "sdkconfig.h"

#if CONFIG_RPI_7INCH_TOUCH_DISPLAY_V1 == 1

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "init_lcd.h"
#include "esp_lcd_touch_ft5x06.h"
#include "esp_lcd_mipi_dsi.h"  // Direct DSI/DPI API
#include "hal/mipi_dsi_hal.h"  // mipi_dsi_hal_host_gen_write_long_packet
#include "hal/mipi_dsi_types.h" // MIPI_DSI_DT_GENERIC_LONG_WRITE
#include "hal/mipi_dsi_ll.h"   // mipi_dsi_host_ll_dpi_set_video_burst_type (LL override for non-burst mode)

static const char *LCD_NAME = "Raspberry Pi 7inch Touch Display V1";
static const char *TAG      = "RPi 7\" Touch Display V1";

// Note: RPi 7" V1 uses FT5406 touch controller managed by Atmel ATTINY88
// over I2C. The ATTINY88 controls power, backlight, and panel initialization.
// The display is a simple DPI panel (no controller IC like ILI9881C).
// The TC358762 bridge converts 1-lane DSI to parallel DPI, configured by ATTINY88.

/* ----------------------------- Display configuration ----------------------------- */
#define LCD_COLOR_FORMAT_RGB565         (1)
#define LCD_COLOR_FORMAT_RGB888         (2)
#define USE_LCD_COLOR_FORMAT_RGB888     (1)  // Use RGB888 end-to-end: simpler, no DMA2D format conversion needed
#define USE_LVGL_AVOID_TEAR             (0)  // Set to 1 to enable avoid tearing feature in LVGL
#define USE_LVGL_FULL_REFRESH           (0)  // Set to 1 to enable full refresh feature in LVGL
#define USE_LVGL_DIRECT_MODE            (0)  // Set to 1 to enable direct mode feature in LVGL

#define LCD_RGB_ELEMENT_ORDER           (LCD_RGB_ELEMENT_ORDER_RGB)

#define LCD_H_RES                       (800)  // Native landscape resolution
#define LCD_V_RES                       (480)
#define LCD_MIPI_DSI_LANE_BITRATE_MBPS  (600)  // Observed stable profile on target hardware
#define LCD_MIPI_DSI_LANE_NUM           (1)    // TC358762 Linux driver uses 1 lane (has TODO for dual-lane)

// RGB888: 3 bytes/pixel → DPI framebuffer is 800×480×3 = 1.15 MB in SPIRAM (allocated by DPI driver).
// LVGL draw buffer is a smaller scratch area used for partial rendering.
#define LCD_DRAW_BUFF_SIZE              (LCD_H_RES * 30)  // 30 lines of draw buffer (pixels)
#define LCD_DRAW_BUFF_DOUBLE            (0)
#define EN_LCD_BUFF_DMA                 (0)  // Must be 0 for RGB888: LVGL port rejects DMA flag for non-RGB565 formats
#define EN_LCD_BUFF_SPIRAM              (1)  // Use SPIRAM for LVGL draw buffer (no DMA alignment needed)

#define EN_LCD_SW_ROTATE                (1)  // Use SW rotation path to avoid unsupported HW mirror/swap calls on raw DPI panel
#define LCD_DPI_BUFFER_NUMS             (1)  // Keep single framebuffer for stable RGB888 path on this panel

// Active timing profile (observed stable values from earlier testing)
#define RPI_MODE_HFP                    (210)
#define RPI_MODE_HSW                    (2)
#define RPI_MODE_HBP                    (46)
#define RPI_MODE_VFP                    (22)
#define RPI_MODE_VSW                    (20)
#define RPI_MODE_VBP                    (4)

// Orientation controls (set both display and touch to the same transform)
#define LCD_ROT_SWAP_XY                 (0)
#define LCD_ROT_MIRROR_X                (1)
#define LCD_ROT_MIRROR_Y                (1)

// Timing from official RPi Linux kernel driver (panel-raspberrypi-touchscreen.c)
#define RPI_7INCH_TOUCH_DISPLAY_V1_CONFIG(px_format) \
    {                                                      \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,       \
        .dpi_clock_freq_mhz = 25.98,                       \
        .virtual_channel = 0,                              \
        /* Timing fields are sourced from the RPI_MODE_* constants below. */ \
        .in_color_format  = (px_format),                   \
        .out_color_format = LCD_COLOR_FMT_RGB888,          \
        .num_fbs = 1,                                      \
        .video_timing = {                                  \
            .h_size = 800,                                 \
            .v_size = 480,                                 \
            .hsync_back_porch = RPI_MODE_HBP,              \
            .hsync_pulse_width = RPI_MODE_HSW,             \
            .hsync_front_porch = RPI_MODE_HFP,             \
            .vsync_back_porch = RPI_MODE_VBP,              \
            .vsync_pulse_width = RPI_MODE_VSW,             \
            .vsync_front_porch = RPI_MODE_VFP,             \
        },                                                 \
        .flags = {                                         \
            .disable_lp = 0,  /* allow LP data-lane blanking → LP command windows for TC358762 config */ \
        },                                                 \
    }

#define LCD_RST_GPIO                    (-1) // GPIO for LCD reset
#define LCD_TOUCH_RST_GPIO              (-1) // Shared with LCD reset
#define LCD_TOUCH_INT_GPIO              (-1) // GPIO for LCD touch interrupt
#define LCD_DATA_BIGENDIAN              (0)

// Atmel ATTINY88-MUR microcontroller configuration (I2C control)
// Registers from drivers/regulator/rpi-panel-attiny-regulator.c (rpi-6.6.y)
#define ATTINY88_ADDR                   (0x45)  // 7-bit I2C address
#define ATTINY88_I2C_SCL_HZ             (400000)
#define ATTINY88_I2C_TIMEOUT_MS         (50)
#define RPI_TOUCH_V1_REG_ID             (0x80)  // Firmware ID (0xC3=v2, 0xDE=v1)
#define RPI_TOUCH_V1_REG_PORTA          (0x81)  // Port A
#define RPI_TOUCH_V1_REG_PORTB          (0x82)  // Port B
#define RPI_TOUCH_V1_REG_PORTC          (0x83)  // Port C
#define RPI_TOUCH_V1_REG_POWERON        (0x85)  // Legacy power-on (old firmware only)
#define RPI_TOUCH_V1_REG_PWM            (0x86)  // Backlight PWM (0-255)
// Indirect TC358762 register-write registers (ATTINY88 proxies writes via SPI)
#define RPI_TOUCH_V1_REG_ADDR_L         (0x8c)  // TC358762 target address low byte
#define RPI_TOUCH_V1_REG_ADDR_H         (0x8d)  // TC358762 target address high byte
#define RPI_TOUCH_V1_REG_WRITE_DATA_H   (0x90)  // TC358762 write data high byte
#define RPI_TOUCH_V1_REG_WRITE_DATA_L   (0x91)  // TC358762 write data low byte

// REG_PORTA bits
#define PA_LCD_DITHB                    BIT(0)  // Dither enable
#define PA_LCD_MODE                     BIT(1)  // LCD mode
#define PA_LCD_LR                       BIT(2)  // Horizontal scan direction (left-to-right)
#define PA_LCD_UD                       BIT(3)  // Vertical scan direction

// REG_PORTB bits
#define PB_BRIDGE_PWRDNX_N              BIT(0)  // Bridge power-down (active low, 1=powered)
#define PB_LCD_VCC_N                    BIT(1)  // LCD VCC (active low, 0=on)
#define PB_LCD_MAIN                     BIT(7)  // Main regulator enable

// REG_PORTC bits
#define PC_LED_EN                       BIT(0)  // Backlight LED enable
#define PC_RST_TP_N                     BIT(1)  // Touch controller reset (active low)
#define PC_RST_LCD_N                    BIT(2)  // LCD reset (active low)
#define PC_RST_BRIDGE_N                 BIT(3)  // TC358762 bridge reset (active low)

// FT5406 touch controller (managed by ATTINY88)
#define FT5406_ADDR                     (0x38)  // Typical FT5x06 family address

// TC358762XBG DSI-to-DPI bridge registers (configured via DSI Generic Write by host)
// Reference: drivers/gpu/drm/bridge/tc358762.c (rpi-6.6.y)
#define TC_DSI_LANEENABLE           (0x0210)  // Lane enable: CLEN=BIT(0), L0EN=BIT(1), L1EN=BIT(2)
#define TC_PPI_D0S_CLRSIPOCOUNT     (0x0164)  // D0 clear SIPO count
#define TC_PPI_D1S_CLRSIPOCOUNT     (0x0168)  // D1 clear SIPO count
#define TC_PPI_D0S_ATMR             (0x0144)  // D0 ATMR
#define TC_PPI_D1S_ATMR             (0x0148)  // D1 ATMR
#define TC_PPI_LPTXTIMECNT          (0x0114)  // LP-TX timing count (LPX_PERIOD=3)
#define TC_SPICMR                   (0x0450)  // SPI command mode register
#define TC_LCDCTRL                  (0x0420)  // LCD control
#define TC_SYSCTRL                  (0x0464)  // System control
#define TC_LCD_HS_HBP               (0x0424)  // Hsync pulse | (HBP << 16)
#define TC_LCD_HDISP_HFP            (0x0428)  // H display | (HFP << 16)
#define TC_LCD_VS_VBP               (0x042c)  // Vsync pulse | (VBP << 16)
#define TC_LCD_VDISP_VFP            (0x0430)  // V display | (VFP << 16)
#define TC_PPI_STARTPPI             (0x0104)  // Start PPI (write 1)
#define TC_DSI_STARTDSI             (0x0204)  // Start DSI video (write 1)
// LCDCTRL bit fields
#define TC_LCDCTRL_VSDELAY(v)       ((v) << 20)
#define TC_LCDCTRL_RGB888           BIT(8)
#define TC_LCDCTRL_UNK6             BIT(6)    // Undocumented, set by Linux driver
#define TC_LCDCTRL_VTGEN            BIT(4)    // Video timing generator enable
#define TC_LCDCTRL_HSPOL            BIT(17)   // HSYNC polarity (1=positive, 0=negative)
#define TC_LCDCTRL_DEPOL            BIT(18)   // DE polarity (1=positive, 0=negative)
#define TC_LCDCTRL_VSPOL            BIT(19)   // VSYNC polarity (1=positive, 0=negative)

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
static esp_err_t attiny88_read_register(uint8_t reg, uint8_t *data);
static esp_err_t attiny88_write_register(uint8_t reg, uint8_t val);
static esp_err_t attiny88_send_command(uint8_t cmd, uint8_t data);
static esp_err_t tc358762_bridge_init(esp_lcd_dsi_bus_handle_t bus);
static esp_err_t lcd_power_on_sequence(void);
static esp_err_t lcd_release_bridge_reset_and_wake(void);
static esp_err_t lcd_release_touch_reset(void);

/*
 * Replicate the private bus struct layout so we can reach the HAL context.
 * This matches esp_lcd_dsi_bus_t in esp-idf/components/esp_lcd/dsi/mipi_dsi_priv.h.
 * If the IDF layout ever changes, update this accordingly.
 */
typedef struct {
    int bus_id;
    mipi_dsi_hal_context_t hal;
} rpi_dsi_bus_priv_t;

/**
 * @brief Write a TC358762 bridge register using a DSI Generic Long Write (0x29).
 *
 * TC358762 only processes Generic Write packets for its own register config;
 * it ignores DCS Write packets (0x39) and forwards them downstream.
 * Format: [reg_lo, reg_hi, val_b0, val_b1, val_b2, val_b3] — 6 bytes.
 */
static void tc358762_reg_write(esp_lcd_dsi_bus_handle_t bus, uint16_t reg, uint32_t val)
{
    rpi_dsi_bus_priv_t *priv = (rpi_dsi_bus_priv_t *)bus;
    uint8_t payload[6] = {
        (reg >>  0) & 0xFF,
        (reg >>  8) & 0xFF,
        (val >>  0) & 0xFF,  // Little-endian: LSB first (matches Linux)
        (val >>  8) & 0xFF,
        (val >> 16) & 0xFF,
        (val >> 24) & 0xFF,  // MSB last
    };
    mipi_dsi_hal_host_gen_write_long_packet(&priv->hal, 0,  // VC=0 default
        MIPI_DSI_DT_GENERIC_LONG_WRITE, payload, sizeof(payload));
}

/**
 * @brief Configure TC358762 DSI-to-DPI bridge registers.
 *
 * Mirrors linux/drivers/gpu/drm/panel/panel-raspberrypi-touchscreen.c
 * Must be called after DSI bus is created and before DPI panel is created.
 */
static esp_err_t tc358762_bridge_init(esp_lcd_dsi_bus_handle_t bus)
{
    ESP_LOGI(__func__, "Starting TC358762 bridge configuration");

    // Exact sequence from Linux drivers/gpu/drm/bridge/tc358762.c (rpi-6.6.y) tc358762_init()
    // Linux sets dsi->lanes = 1 with TODO comment: "Find out how to get dual-lane mode working"
    tc358762_reg_write(bus, TC_DSI_LANEENABLE, BIT(0) | BIT(1)); // CLEN + D0EN only — 1 data lane
    tc358762_reg_write(bus, TC_PPI_D0S_CLRSIPOCOUNT, 0x05);
    tc358762_reg_write(bus, TC_PPI_D1S_CLRSIPOCOUNT, 0x05);
    tc358762_reg_write(bus, TC_PPI_D0S_ATMR, 0x00);
    tc358762_reg_write(bus, TC_PPI_D1S_ATMR, 0x00);
    tc358762_reg_write(bus, TC_PPI_LPTXTIMECNT, 0x03);  // LPX_PERIOD = 3 (Linux literal)

    tc358762_reg_write(bus, TC_SPICMR, 0x00);
    // LCDCTRL literal from panel-raspberrypi-touchscreen.c — no HSPOL/VSPOL (mode has no NHSYNC/NVSYNC)
    tc358762_reg_write(bus, TC_LCDCTRL, 0x00100150);
    tc358762_reg_write(bus, TC_SYSCTRL, 0x040f);

    // With VTGEN=1 the bridge generates DPI output using its own clock and these
    // timing registers. They MUST be written. Source: Linux tc358762.c tc358762_init().
    // Formula: LCD_HS_HBP  = HSW | (HBP << 16)  where HSW=hsync_end-hsync_start, HBP=htotal-hsync_end
    //          LCD_HDISP_HFP = hdisplay | (HFP << 16)  where HFP=hsync_start-hdisplay
    //          LCD_VS_VBP  = VSW | (VBP << 16)  where VSW=vsync_end-vsync_start, VBP=vtotal-vsync_end
    //          LCD_VDISP_VFP = vdisplay | (VFP << 16)  where VFP=vsync_start-vdisplay
    // Mode: clock=25979400Hz, h=800+HFP+2+HBP=849, v=480+7+2+21=510
    tc358762_reg_write(bus, TC_LCD_HS_HBP,    (RPI_MODE_HBP << 16) | RPI_MODE_HSW);
    tc358762_reg_write(bus, TC_LCD_HDISP_HFP, (RPI_MODE_HFP << 16) | 800);
    tc358762_reg_write(bus, TC_LCD_VS_VBP,    (RPI_MODE_VBP << 16) | RPI_MODE_VSW);
    tc358762_reg_write(bus, TC_LCD_VDISP_VFP, (RPI_MODE_VFP << 16) | 480);

    vTaskDelay(pdMS_TO_TICKS(100));  // Linux: msleep(100) before starting streams

    // Linux order: PPI_STARTPPI first, then DSI_STARTDSI
    tc358762_reg_write(bus, TC_PPI_STARTPPI, 0x01);
    tc358762_reg_write(bus, TC_DSI_STARTDSI, 0x01);

    vTaskDelay(pdMS_TO_TICKS(100));  // Linux: msleep(100) after starting streams

    ESP_LOGI(__func__, "TC358762 bridge configured via Generic DSI writes");
    return ESP_OK;
}

static bool IS_LCD_ENABLED = false; // Set true after power-on sequence succeeds

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
 * @brief Initialize the LCD and touch input
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

    /* Initialize backlight to 10 | Range: 0-31 */
    ESP_ERROR_CHECK_RETURN_ERR(lcd_brightness_set(255));  // Set to full brightness initially

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
 * @brief Read a register from the ATTINY88 microcontroller
 * 
 * @param reg Register address to read from
 * @param data Pointer to store the read data
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t attiny88_read_register(uint8_t reg, uint8_t *data)
{
    ESP_ERROR_CHECK_RETURN_ERR(esp_i2c_init());

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = ATTINY88_I2C_SCL_HZ,
        .device_address = ATTINY88_ADDR,
    };

    i2c_master_dev_handle_t dev_handle = NULL;
    i2c_master_bus_handle_t i2c_bus = esp_i2c_get_handle();
    if (i2c_bus == NULL) {
        ESP_LOGE(__func__, "I2C bus handle is NULL");
        return ESP_FAIL;
    }
    if (i2c_master_bus_add_device(i2c_bus, &i2c_dev_conf, &dev_handle) != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to add I2C device at address 0x%02X", ATTINY88_ADDR);
        return ESP_FAIL;
    }

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, data, 1, ATTINY88_I2C_TIMEOUT_MS);
    i2c_master_bus_rm_device(dev_handle);

    return ret;
}

/**
 * @brief Send a command to the ATTINY88 control MCU
 * 
 * @return esp_err_t ESP_OK on success, or an error code on failure
 * 
 */
static esp_err_t attiny88_send_command(uint8_t cmd, uint8_t data)
{
    ESP_ERROR_CHECK_RETURN_ERR(esp_i2c_init());

    uint8_t data_to_send[2] = {cmd, data};

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = ATTINY88_I2C_SCL_HZ,
        .device_address = ATTINY88_ADDR,  // Atmel ATTINY88 microcontroller
    };

    i2c_master_dev_handle_t dev_handle = NULL;
    i2c_master_bus_handle_t i2c_bus = esp_i2c_get_handle();
    if (i2c_bus == NULL) {
        ESP_LOGE(__func__, "I2C bus handle is NULL");
        return ESP_FAIL;
    }
    if (i2c_master_bus_add_device(i2c_bus, &i2c_dev_conf, &dev_handle) != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to add I2C device at address 0x%02X", ATTINY88_ADDR);
        return ESP_FAIL;
    }

    esp_err_t ret = i2c_master_transmit(dev_handle, data_to_send, sizeof(data_to_send), ATTINY88_I2C_TIMEOUT_MS);
    if (ret != ESP_OK)
    {
        i2c_master_bus_rm_device(dev_handle);
        return ret;
    }

    i2c_master_bus_rm_device(dev_handle);

    return ESP_OK;
}

/**
 * @brief Write a single byte to an ATTINY88 register
 */
static esp_err_t attiny88_write_register(uint8_t reg, uint8_t val)
{
    return attiny88_send_command(reg, val);
}

/**
 * @brief Power on the display using the rpi-panel-attiny-regulator.c sequence.
 *
 * Mirrors attiny_lcd_power_enable() from drivers/regulator/rpi-panel-attiny-regulator.c:
 *   1. Assert all resets (REG_PORTC = 0)
 *   2. Set scan direction (REG_PORTA = PA_LCD_LR)
 *   3. Enable main regulator (REG_PORTB = PB_LCD_MAIN)
 *   4. Enable LED only — bridge stays in reset (REG_PORTC = PC_LED_EN)
 *   5. Wait 80 ms for power rails to stabilise
 *   6. Release TC358762 reset and write SYSPMCTRL=0 via ATTINY SPI proxy
 *      (mirrors attiny_gpio_set RST_BRIDGE_N=1 path)
 */
static esp_err_t lcd_power_on_sequence(void)
{
    IS_LCD_ENABLED = false;

    /* 1. Assert all resets */
    ESP_RETURN_ON_ERROR(attiny88_write_register(RPI_TOUCH_V1_REG_PORTC, 0x00), __func__, "REG_PORTC=0 failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 2. Default scan direction: left-to-right */
    ESP_RETURN_ON_ERROR(attiny88_write_register(RPI_TOUCH_V1_REG_PORTA, PA_LCD_LR), __func__, "REG_PORTA failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 3. Main regulator on, LCD VCC on */
    ESP_RETURN_ON_ERROR(attiny88_write_register(RPI_TOUCH_V1_REG_PORTB, PB_LCD_MAIN), __func__, "REG_PORTB failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 4. LED enable — bridge stays in reset until we explicitly release it */
    ESP_RETURN_ON_ERROR(attiny88_write_register(RPI_TOUCH_V1_REG_PORTC, PC_LED_EN), __func__, "REG_PORTC=LED failed");
    vTaskDelay(pdMS_TO_TICKS(80));

    IS_LCD_ENABLED = true;
    ESP_LOGI(__func__, "ATTINY88 power-on sequence complete (v1.1 regulator protocol, bridge still held in reset)");
    return ESP_OK;
}

/**
 * @brief Release TC358762 reset and wake bridge core via ATTINY88 SPI proxy.
 *
 * Mirrors rpi-panel-attiny-regulator.c attiny_gpio_set(RST_BRIDGE_N=1):
 *   - REG_PORTC |= PC_RST_BRIDGE_N | PC_RST_LCD_N
 *   - write TC358762 SYSPMCTRL(0x047C)=0x0000 through ATTINY proxy registers
 */
static esp_err_t lcd_release_bridge_reset_and_wake(void)
{
    ESP_RETURN_ON_ERROR(attiny88_write_register(RPI_TOUCH_V1_REG_PORTC,
        PC_LED_EN | PC_RST_LCD_N | PC_RST_BRIDGE_N), __func__, "REG_PORTC=BRIDGE_RST_RELEASE failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_RETURN_ON_ERROR(attiny88_write_register(RPI_TOUCH_V1_REG_ADDR_H,  0x04), __func__, "ADDR_H failed");
    vTaskDelay(pdMS_TO_TICKS(8));
    ESP_RETURN_ON_ERROR(attiny88_write_register(RPI_TOUCH_V1_REG_ADDR_L,  0x7c), __func__, "ADDR_L failed");
    vTaskDelay(pdMS_TO_TICKS(8));
    ESP_RETURN_ON_ERROR(attiny88_write_register(RPI_TOUCH_V1_REG_WRITE_DATA_H, 0x00), __func__, "WR_DATA_H failed");
    vTaskDelay(pdMS_TO_TICKS(8));
    ESP_RETURN_ON_ERROR(attiny88_write_register(RPI_TOUCH_V1_REG_WRITE_DATA_L, 0x00), __func__, "WR_DATA_L failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(__func__, "TC358762 reset released and SYSPMCTRL wake sequence sent");
    return ESP_OK;
}

/**
 * @brief Release touch controller reset just before FT5406 probe.
 */
static esp_err_t lcd_release_touch_reset(void)
{
    ESP_RETURN_ON_ERROR(attiny88_write_register(RPI_TOUCH_V1_REG_PORTC,
        PC_LED_EN | PC_RST_TP_N | PC_RST_LCD_N | PC_RST_BRIDGE_N), __func__, "REG_PORTC=TOUCH_RST_RELEASE failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

/**
 * @brief Initialize LCD brightness control and ATTINY88 microcontroller
 * 
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t lcd_brightness_init(void)
{
    esp_i2c_init();
    
    // Read firmware ID from ATTINY88
    uint8_t fw_id = 0;
    if (attiny88_read_register(RPI_TOUCH_V1_REG_ID, &fw_id) == ESP_OK) {
        ESP_LOGI(TAG, "ATTINY88 Firmware ID: 0x%02X", fw_id);
    } else {
        ESP_LOGW(TAG, "Failed to read ATTINY88 firmware ID");
    }
    
    return ESP_OK;
}

/**
 * @brief Set the LCD brightness
 * 
 * @param brightness Brightness level (0-255), 0=off, 255=max brightness
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t lcd_brightness_set(int brightness)
{
    if (!IS_LCD_ENABLED) return ESP_FAIL;

    // Clamp brightness to valid range 0-255
    if (brightness > 255) {
        brightness = 255;
    }
    if (brightness < 0) {
        brightness = 0;
    }

    // RPi V1: Direct PWM value (0-255) sent to Atmel ATTINY88
    return attiny88_send_command(RPI_TOUCH_V1_REG_PWM, (uint8_t)brightness);
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
    return lcd_brightness_set(255);  // Full brightness (0-255 range for V1)
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
            // Raw DSI DPI panel does not support esp_lcd_panel_mirror()/swap_xy().
            // Keep display transform disabled here to avoid unsupported-operation errors.
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

    /* Power on display using new v1.1 regulator protocol */
    ESP_RETURN_ON_ERROR(lcd_power_on_sequence(), __func__, "Power-on sequence failed");

    ESP_LOGI(__func__, "Configuring DSI and communicating with Bridge...");

    /* create MIPI DSI bus first, it will initialize the DSI PHY as well */
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = LCD_MIPI_DSI_LANE_NUM,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = LCD_MIPI_DSI_LANE_BITRATE_MBPS,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus), __func__, "New DSI bus init failed");

    // Create DBI IO solely to configure LP speed mode for Generic Long Writes in the DSI
    // controller hardware. The gen_lw_tx=LP setting persists after the IO is deleted.
    // TC358762 bridge_init() is NOT called here — it must happen AFTER panel_init() so that
    // the HS clock is running when TC358762 processes the incoming LP commands.
    esp_lcd_panel_io_handle_t dbi_io = NULL;
    esp_lcd_dbi_io_config_t lp_dbi_cfg = { .virtual_channel = 0, .lcd_cmd_bits = 8, .lcd_param_bits = 8 };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &lp_dbi_cfg, &dbi_io), __func__, "LP mode setup failed");
    esp_lcd_panel_io_del(dbi_io); // LP speed settings now latched in DSI controller hardware

    ESP_LOGI(__func__, "Install (RPi 7inch) MIPI DSI DPI panel");
    esp_lcd_panel_handle_t disp_panel = NULL;

#if USE_LCD_COLOR_FORMAT_RGB888
    esp_lcd_dpi_panel_config_t dpi_config = RPI_7INCH_TOUCH_DISPLAY_V1_CONFIG(LCD_COLOR_FMT_RGB888);
#else
    esp_lcd_dpi_panel_config_t dpi_config = RPI_7INCH_TOUCH_DISPLAY_V1_CONFIG(LCD_COLOR_FMT_RGB565);
#endif

    dpi_config.num_fbs = LCD_DPI_BUFFER_NUMS;

    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_dpi(mipi_dsi_bus, &dpi_config, &disp_panel), err, __func__, "New DPI panel failed");

    // esp_lcd_new_panel_dpi() hardcodes BURST_WITH_SYNC_PULSES. Override to NON-BURST
    // to match TC358762/Linux: MIPI_DSI_MODE_VIDEO_SYNC_PULSE.
    // Confirmed by ESP-IDF source: panel_init() does NOT reset this, so setting it
    // here (before panel_init) is safe and effective.
    {
        rpi_dsi_bus_priv_t *_priv = (rpi_dsi_bus_priv_t *)mipi_dsi_bus;
        mipi_dsi_host_ll_dpi_set_video_burst_type(_priv->hal.host,
            MIPI_DSI_LL_VIDEO_NON_BURST_WITH_SYNC_PULSES);
        // TC358762 does not provide BTA ACK for every video frame.
        // Keeping this enabled can stall or corrupt the stream on some hosts.
        mipi_dsi_host_ll_dpi_enable_frame_ack(_priv->hal.host, false);
        ESP_LOGI(__func__, "DSI video mode set to NON-BURST with sync pulses (matches Linux TC358762 driver)");
    }

    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(disp_panel), err, __func__, "LCD panel init failed");

    // Force continuous HS clock. Data lanes can still LP during blanking (disable_lp=0)
    // so the DSI host can insert LP Generic Long Write command packets during vertical blank.
    {
        rpi_dsi_bus_priv_t *_priv = (rpi_dsi_bus_priv_t *)mipi_dsi_bus;
        mipi_dsi_host_ll_set_clock_lane_state(_priv->hal.host, MIPI_DSI_LL_CLOCK_LANE_STATE_HS);
        ESP_LOGI(__func__, "DSI clock lane forced to continuous HS (TC358762 FLL requires stable reference)");
    }

    // Configure TC358762 NOW — video is running, HS clock is active.
    // TC358762 requires an active HS reference to receive LP commands. Sending in pure
    // command mode (clock LP, no video) is silently dropped. With the HS clock running
    // and disable_lp=0 allowing LP data blanking, the DSI host schedules each Generic
    // Long Write packet during the vertical blanking LP window.
    {
        rpi_dsi_bus_priv_t *_priv = (rpi_dsi_bus_priv_t *)mipi_dsi_bus;
        // Disable CMD ACK — TC358762 would not respond with BTA to ACK requests,
        // which could cause bus contention on LP data lanes during video blanking.
        mipi_dsi_host_ll_enable_cmd_ack(_priv->hal.host, false);
    }

    // Linux v1.1 flow releases bridge reset only right before bridge init.
    ESP_GOTO_ON_ERROR(lcd_release_bridge_reset_and_wake(), err, __func__, "Bridge reset release/wake failed");

    ESP_GOTO_ON_ERROR(tc358762_bridge_init(mipi_dsi_bus), err, __func__, "TC358762 bridge init failed");

    // Note: esp_lcd_panel_disp_on_off is not supported for raw DPI panels.
    // Display was powered on in lcd_power_on_sequence() above.

    /* Set backlight PWM to full brightness */
    attiny88_send_command(RPI_TOUCH_V1_REG_PWM, 255);

    /* Return all handles */
    ret_handles->io = NULL;  // No DBI IO needed for pure DPI panel
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
    /* Initialize I2C */
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
            .swap_xy = LCD_ROT_SWAP_XY,
            .mirror_x = LCD_ROT_MIRROR_X,
            .mirror_y = LCD_ROT_MIRROR_Y,
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = {
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_FT5x06_ADDRESS,  // FT5406 touch controller
        .control_phase_bytes = 1,
        .dc_bit_offset = 0,
        .lcd_cmd_bits = 8,  // FT5x06 uses 8-bit addressing
        .flags = {
            .disable_control_phase = 1,
        },
    };

    tp_io_config.scl_speed_hz = ATTINY88_I2C_SCL_HZ;

    i2c_master_bus_handle_t i2c_bus = esp_i2c_get_handle();
    ESP_LOGI(__func__, "I2C bus handle: %p", i2c_bus);
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_bus, &tp_io_config, &tp_io_handle), __func__, "");

    esp_err_t ret = esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, ret_touch);
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

    // Mirror Linux gpio consumer ordering: release TP reset right before FT5406 init.
    ESP_ERROR_CHECK_RETURN_NULL(lcd_release_touch_reset());

    ESP_ERROR_CHECK_RETURN_NULL(lcd_touch_new(NULL, &tp));
    assert(tp);

    /* Add touch input (for selected screen) */
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


#endif // CONFIG_RPI_7INCH_TOUCH_DISPLAY_V1
// End of file rpi_7inch_touch_v1.c
