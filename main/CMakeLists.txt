cmake_minimum_required(VERSION 3.16)

set(DISPLAY_CONF_DIR "${CMAKE_CURRENT_LIST_DIR}/lcd_config")
set(HARDWARE_CONF_DIR "${CMAKE_CURRENT_LIST_DIR}/hw_config")

# Collect sources from config folders explicitly (wildcards inside idf_component_register don't expand)
file(GLOB DISPLAY_CONF_SOURCES "${DISPLAY_CONF_DIR}/*.c")
file(GLOB HARDWARE_CONF_SOURCES "${HARDWARE_CONF_DIR}/*.c")

set(main_priv_requires
    esp_lcd
    esp_lcd_dsi
    esp_driver_ledc
    spiffs
    fatfs
)

# ESP-IDF v6.x provides esp_hal_lcd as a standalone component.
# In v5.5.x the LCD HAL is part of other components, so this target is absent.
if(EXISTS "${IDF_PATH}/components/esp_hal_lcd/CMakeLists.txt")
    list(APPEND main_priv_requires esp_hal_lcd)
endif()

idf_component_register(
    SRCS
        "${CMAKE_CURRENT_LIST_DIR}/main.c"
        ${DISPLAY_CONF_SOURCES}
        ${HARDWARE_CONF_SOURCES}
    INCLUDE_DIRS
        .
        "${DISPLAY_CONF_DIR}"
        "${HARDWARE_CONF_DIR}"
    REQUIRES
        lvgl__lvgl driver cchalk
    PRIV_REQUIRES ${main_priv_requires}
)
