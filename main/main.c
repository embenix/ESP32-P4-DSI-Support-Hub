#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_memory_utils.h"
#include "lvgl.h"
#include "lv_demos.h"
#include "driver/i2c.h"
#include "esp_log.h"


#define I2C_PORT        I2C_NUM_0
#define I2C_SCL_GPIO    8    // change to your SCL pin
#define I2C_SDA_GPIO    7    // change to your SDA pin
#define I2C_FREQ_HZ     100000

static const char *TAG = "i2c_scan";

/**
 * @brief Initialize I2C master interface
 * 
 */
static void i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

/**
 * @brief Scan I2C bus for devices
 * 
 */
static void i2c_scan(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 0x7F; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(20));
        i2c_cmd_link_delete(cmd);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    ESP_LOGI(TAG, "Scan done.");
}

/**
 * @brief Application entry point
 * 
 */
void app_main(void)
{
    // bsp_display_cfg_t cfg = {
    //     .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
    //     .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
    //     .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
    //     .flags = {
    //         .buff_dma = true,
    //         .buff_spiram = false,
    //         .sw_rotate = false,
    //     }
    // };
    // bsp_display_start_with_config(&cfg);
    // // bsp_display_backlight_on();
    // bsp_display_brightness_set(10);
    
    // bsp_display_lock(0);

    // // lv_demo_music();
    // // lv_demo_benchmark();
    // lv_demo_widgets();

    // bsp_display_unlock();
    
    // i2c_init();
    // i2c_scan();

    while(true) {
        //Add the touch data here
        // bsp_touch_data_t touch_data;
        // if (bsp_touch_read(&touch_data) == ESP_OK) {
        //     ESP_LOGI(TAG, "Touch detected: x=%d, y=%d", touch_data.x, touch_data.y);
        // }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}