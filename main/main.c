#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_memory_utils.h"
#include "esp_heap_caps.h"
#include "lvgl.h"
#include "lv_demos.h"
#include "esp_lvgl_port.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "init_lcd.h"
#include "init_hw.h"
#include "esp_err_check.h"
#include "esp_lcd_touch.h"

static const char *TAG = "DSI support hub";

 /**
 * @brief Application entry point
 * 
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting DSI Support Hub");

    /* Log PSRAM availability and stats at startup to help with debugging display buffer allocation issues */
    size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t psram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    if (psram_total > 0) {
        ESP_LOGI(TAG, "PSRAM available | total: %u bytes, free: %u bytes, largest block: %u bytes",
                 (unsigned)psram_total,
                 (unsigned)psram_free,
                 (unsigned)psram_largest);
    } else {
        ESP_LOGW(TAG, "PSRAM not available via heap caps; high-resolution/double-buffer display modes may fail");
    }
    
    // Initialize LVGL and LCD
    lvgl_port_cfg_t lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_cfg.task_priority = 5;
    lvgl_port_cfg.task_stack = CONFIG_ESP_LVGL_PORT_TASK_STACK_SIZE;  // Use Kconfig value (default 16KB, configurable in menuconfig)
    lvgl_port_cfg.task_affinity = -1;
    lvgl_port_cfg.task_max_sleep_ms = 20;
    lvgl_port_cfg.timer_period_ms = 2;
    lvgl_port_init(&lvgl_port_cfg);
    esp_err_t err = init_lcd();

    vTaskDelay(pdMS_TO_TICKS(500)); // Wait for half a second to ensure LCD is ready

    i2c_scan();

    if (err == ESP_OK) {
        // Use this method to set display rotation if needed
#if CONFIG_RPI_7INCH_TOUCH_DISPLAY_V2 == 1
        lv_display_set_rotation(display_get_handle(), LV_DISPLAY_ROTATION_270);
#elif CONFIG_RPI_5INCH_TOUCH_DISPLAY_V2 == 1
        lv_display_set_rotation(display_get_handle(), LV_DISPLAY_ROTATION_90);
// #elif CONFIG_RPI_7INCH_TOUCH_DISPLAY_V1 == 1
//         lv_display_set_rotation(display_get_handle(), LV_DISPLAY_ROTATION_180);
#elif CONFIG_BUYDISPLAY_5INCH_ER_TFT050_10 == 1
        lv_display_set_rotation(display_get_handle(), LV_DISPLAY_ROTATION_90); 
#endif

        // Run a demo application
        lvgl_port_lock(0);
        // lv_demo_music();
        // lv_demo_benchmark();
        lv_demo_widgets();
        lvgl_port_unlock();
    }
    else {
        ESP_LOGE(TAG, "LCD initialization failed | not running demo");
    }

    while(true) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
