# Task List

## Completed Tasks ✅

### ESP-IDF Compatibility
- [x] Review olimex board header for I2C and board name definitions
- [x] Add ESP-IDF v6.0 compatibility (patched components with conditional compilation)
- [x] Maintain backward compatibility with ESP-IDF v5.5.3+
- [x] Fix linker errors with RTC fast memory heap (non-contiguous regions issue)
- [x] Fix LVGL IRAM section conflicts (disabled CONFIG_LV_ATTRIBUTE_FAST_MEM_USE_IRAM)
- [x] Fix esp_lcd_dsi dependency issue (removed conditional rule for all boards)

### Board Support
- [x] Add DFRobot FireBeetle 2 ESP32-P4 board support
- [x] Add chip revision support for ESP32-P4 v1.0+ (global default)
- [x] Support 5 ESP32-P4 development boards (Espressif, Olimex, DFRobot, Waveshare, Guition)

### Build & Configuration
- [x] Optimize partition table (10MB factory app, 5MB storage)
- [x] Reduce binary size (disabled extra LVGL demos, kept widgets only)
- [x] Add sdkconfig to .gitignore (users generate from sdkconfig.defaults)
- [x] Create board-specific sdkconfig overlays (boards/sdkconfig.defaults.*)

### Documentation
- [x] Update README.md with ESP-IDF v5.5.3/v6.0 compatibility notes
- [x] Add "Supported boards" section to README
- [x] Add troubleshooting section for ESP-IDF version switching
- [x] Document local component patches (components/README.md)
- [x] Update IDF version requirement to >=5.5.3 in idf_component.yml

### Display Support
- [x] Add BuyDisplay 5" IPS TFT ER-TFT050-10 (720x1280, DSI 2-lanes, GT911)
- [x] Add DFRobot 5" DSI (800x480, DSI 1-lane, FT5x06)
- [x] Add Luckfox 5" DSI (800x480, DSI 1-lane, FT5x06)
- [x] Add Raspberry Pi 5" Touch Display V2 (720x1280, DSI 2-lanes, GT911)
- [x] Add Raspberry Pi 7" Touch Display V2 (720x1280, DSI 2-lanes, GT911)

## In Progress 🚧

- [ ] Complete Amelin 7" 1024x600 LCD display driver

## Pending Tasks 📝

### Hardware Initialization
- [ ] Ensure init_hw.c uses hw_init() signature and proper macros uniformly across all boards

### Display Support
- [ ] Add Raspberry Pi Touch Display v1 support (5" and 7" variants)
  - Resolution: 800x480 (5"), 800x480 (7")
  - Interface: DSI (1-lane typical)
  - Touch controller: FT5406 (I2C)
- [ ] Add Waveshare DSI display models (specific models TBD)
- [ ] Add SeeedStudio DSI display models (specific models TBD)

### Performance & Optimization
- [ ] Optimize rendering performance on higher pixel density displays (1024x600, 1280x720)
- [ ] Investigate frame buffer optimization for LVGL on PSRAM
- [ ] Profile and optimize DSI lane bitrate settings per display

### Features
- [ ] Add SD card support for loading images/assets
- [ ] Add USB HID support for keyboard/mouse input
- [ ] Add rotation/orientation configuration support
- [ ] Implement display sleep/wake functionality

### Testing & Validation
- [ ] Test all displays with both ESP-IDF v5.5.3 and v6.0
- [ ] Validate touch calibration across different displays
- [ ] Create example applications showcasing different LVGL features

