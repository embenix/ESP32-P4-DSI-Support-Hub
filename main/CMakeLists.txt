cmake_minimum_required(VERSION 3.16)

# Resolve paths relative to this CMakeLists.txt for robustness
set(LV_DEMO_DIR "${CMAKE_CURRENT_LIST_DIR}/../managed_components/lvgl__lvgl/demos")
file(GLOB_RECURSE LV_DEMOS_SOURCES "${LV_DEMO_DIR}/*.c")

set(DISPLAY_CONF_DIR "${CMAKE_CURRENT_LIST_DIR}/lcd_config")
set(HARDWARE_CONF_DIR "${CMAKE_CURRENT_LIST_DIR}/hw_config")

# Collect sources from config folders explicitly (wildcards inside idf_component_register don't expand)
file(GLOB DISPLAY_CONF_SOURCES "${DISPLAY_CONF_DIR}/*.c")
file(GLOB HARDWARE_CONF_SOURCES "${HARDWARE_CONF_DIR}/*.c")

idf_component_register(
    SRCS
        "${CMAKE_CURRENT_LIST_DIR}/main.c"
        ${DISPLAY_CONF_SOURCES}
        ${HARDWARE_CONF_SOURCES}
        ${LV_DEMOS_SOURCES}
    INCLUDE_DIRS
        .
        "${DISPLAY_CONF_DIR}"
        "${HARDWARE_CONF_DIR}"
        "${LV_DEMO_DIR}"
    REQUIRES
        lvgl__lvgl driver
    PRIV_REQUIRES esp_lcd usb spiffs fatfs
)

# Apply LVGL-related compile definitions to this component so demo sources see them
target_compile_options(
    ${COMPONENT_LIB}
    PRIVATE
        -DLV_LVGL_H_INCLUDE_SIMPLE
        -DLV_USE_DEMO_MUSIC
)

# Keep also defining these for the LVGL library if present (harmless if duplicate)
idf_component_get_property(LVGL_LIB lvgl__lvgl COMPONENT_LIB)
if(LVGL_LIB)
    target_compile_options(
        ${LVGL_LIB}
        PRIVATE
            -DLV_LVGL_H_INCLUDE_SIMPLE
            -DLV_USE_DEMO_MUSIC
    )
endif()
