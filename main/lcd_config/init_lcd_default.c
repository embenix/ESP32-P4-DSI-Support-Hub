#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "init_lcd.h"

#ifndef __has_attribute
  #define __has_attribute(x) 0
#endif

#if __has_attribute(weak)
#define WEAK __attribute__((weak))
#else
#define WEAK
#endif


/**
 * @brief Weak default implementation. Panel-specific sources provide a strong one when enabled.
 * 
 */
#if !defined(CONFIG_LUCKFOX_5INCH_DSI_TOUCHSCREEN) || !(CONFIG_LUCKFOX_5INCH_DSI_TOUCHSCREEN)

static const char *TAG_INIT_LCD = "init_lcd_default";

WEAK esp_err_t init_lcd(void)
{
  ESP_LOGW(TAG_INIT_LCD, "init_lcd(): no panel selected or panel driver disabled; default stub running");
  return ESP_OK;
}
#endif
  