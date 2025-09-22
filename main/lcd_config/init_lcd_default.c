#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "init_lcd.h"

#ifdef __GNUC__
#define WEAK __attribute__((weak))
#else
#define WEAK
#endif


/**
 * @brief Weak default implementation. Compiled only if no panel is selected.
 * Panel-specific sources provide a strong init_lcd() when their Kconfig symbol is set.
 */
#if !(CONFIG_LUCKFOX_5INCH_DSI_TOUCHSCREEN) && \
  !(CONFIG_DFROBOT_5INCH_DSI_TOUCHSCREEN) && \
  !(CONFIG_RPI_7INCH_TOUCH_DISPLAY_V2)

static const char *TAG_INIT_LCD = "init_lcd_default";

WEAK esp_err_t init_lcd(void)
{
  ESP_LOGW(TAG_INIT_LCD, "init_lcd(): no panel selected or panel driver disabled; default stub running");
  return ESP_FAIL;
}
#endif
  