# Append vendor-specific sdkconfig defaults based on menuconfig selection

# The generated sdkconfig.cmake provides CONFIG_* variables after the first configure.
# We include it here to read the selected vendor symbols.
if(EXISTS "${PROJECT_BINARY_DIR}/config/sdkconfig.cmake")
	include("${PROJECT_BINARY_DIR}/config/sdkconfig.cmake")
endif()

# Read current SDKCONFIG_DEFAULTS and append our board overlays.
idf_build_get_property(SDKCONFIG_DEFAULTS SDKCONFIG_DEFAULTS)

# Resolve overlay directory relative to the project root.
set(_BOARD_OVERLAYS_DIR "${CMAKE_CURRENT_LIST_DIR}/boards")

if(CONFIG_WAVESHARE_ESP32P4_MODULE_DEVKIT)
	list(APPEND SDKCONFIG_DEFAULTS "${_BOARD_OVERLAYS_DIR}/sdkconfig.defaults.waveshare")
elseif(CONFIG_OLIMEX_ESP32P4_DEVKIT)
	list(APPEND SDKCONFIG_DEFAULTS "${_BOARD_OVERLAYS_DIR}/sdkconfig.defaults.olimex")
elseif(CONFIG_GUITION_ESP32P4_DEVKIT)
	list(APPEND SDKCONFIG_DEFAULTS "${_BOARD_OVERLAYS_DIR}/sdkconfig.defaults.guition")
elseif(CONFIG_ESPRESSIF_ESP32P4_FUNCTION_EV_BOARD)
	list(APPEND SDKCONFIG_DEFAULTS "${_BOARD_OVERLAYS_DIR}/sdkconfig.defaults.espressif")
else()
	# Fallback to Espressif overlay if no vendor is selected yet.
	list(APPEND SDKCONFIG_DEFAULTS "${_BOARD_OVERLAYS_DIR}/sdkconfig.defaults.espressif")
endif()

# Apply back to the build.
idf_build_set_property(SDKCONFIG_DEFAULTS "${SDKCONFIG_DEFAULTS}" REPLACE)

