#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_memory_utils.h"
#include "lvgl.h"
#include "lv_demos.h"
#include "esp_lvgl_port.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "init_lcd.h"
#include "init_hw.h"
#include "esp_err_check.h"

static const char *TAG = "DSI support hub";

/**
 * @brief Application entry point
 * 
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting DSI Support Hub");
    
    // Initialize LVGL and LCD
    lvgl_port_cfg_t lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_port_cfg);
    init_lcd();

    vTaskDelay(pdMS_TO_TICKS(500)); // Wait for half a second to ensure LCD is ready

    i2c_scan();

    // Use this method to set display rotation if needed
    // lv_display_set_rotation(display_get_handle(), LV_DISPLAY_ROTATION_90);

    // Run a demo application
    lvgl_port_lock(0);
    // lv_demo_music();
    // lv_demo_benchmark();
    lv_demo_widgets();
    lvgl_port_unlock();

    while(true) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}