# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Parse project-root sdkconfig to set LCD_TYPE early (before project.cmake include)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig" SDKCONFIG_CONTENT)
    string(REGEX MATCH "CONFIG_BUYDISPLAY_5INCH_ER_TFT050_10=y" MATCHED_BUYDISPLAY "${SDKCONFIG_CONTENT}")
    string(REGEX MATCH "CONFIG_AMELIN_EK79007_7INCH_TOUCH_DISPLAY=y" MATCHED_AMELIN "${SDKCONFIG_CONTENT}")
    if(MATCHED_BUYDISPLAY)
        set(ENV{LCD_TYPE} "buydisplay_ertft050_10")
    elseif(MATCHED_AMELIN)
        set(ENV{LCD_TYPE} "amelin_ek79007")
    else()
        set(ENV{LCD_TYPE} "waveshare_ili9881c")
    endif()
else()
    # Default on first run (no sdkconfig yet)
    set(ENV{LCD_TYPE} "waveshare_ili9881c")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
    ./components
    )

add_compile_options(-Wno-format)
project(p4_dsi_lvgl_demo_v9)

# Optional: Log for verification (runs after everything, so safe)
message(STATUS "LCD_TYPE: $ENV{LCD_TYPE}")