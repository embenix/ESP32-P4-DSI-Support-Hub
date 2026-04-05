# ESP32-P4 MIPI DSI Support Hub

> Momentum beats perfection - keep moving, keep improving.

(Note: This is work in progress.)
A general-purpose boilerplate for bringing up MIPI DSI LCDs (with optional touch) on the ESP32‑P4. This repo aims to give developers a clean, extensible starting point: select a board, select a panel, build, and get pixels on the screen with LVGL.

## Feature

Modular, trim-to-fit panel support: choose your LCD and keep builds lean by removing unused drivers in `lcd_config/`.

## Goals

- Minimal, vendor-neutral scaffolding for MIPI DSI on ESP32‑P4
- Clean separation of hardware init, display config, and UI logic (LVGL 9)
- Pluggable panel drivers and touch controllers
- Menuconfig-driven board selection and automatic sdkconfig overlays per vendor

## Quick start

1. Prerequisites
    - **ESP-IDF v5.5.3+** or **v6.0** (P4 + MIPI DSI capable toolchain, tested with v5.5.3 and v6.0)
    - Python and Git installed

> **Note for ESP-IDF v6.0 Users**: This project includes modified components in the `components/` directory to ensure compatibility with ESP-IDF v6.0's new color/display API. These will be automatically used during build. The patches use conditional compilation to maintain backward compatibility with ESP-IDF v5.5.3+. See `components/README.md` for details.

2. Configure
    - Select your board in menuconfig under: Project Configuration → Hardware Vendor/Board
    - Select your LCD panel: Project Configuration → Display → Panel
    - No options will be added later

3. Build and flash
    - idf.py set-target esp32p4
    - idf.py menuconfig
    - idf.py -p PORT flash monitor

Note: Board selection applies an sdkconfig overlay from `boards/sdkconfig.defaults.<vendor>` to set sane SPI flash options and other defaults.

## Repo structure

- `main/`
  - `hw_config/` Board-level setup (I2C, LDO, MIPI DSI PHY power, etc.)
  - `lcd_config/` Panel selection and LVGL display/touch wiring
- `components/` Local component overrides (ESP-IDF v6.0 compatibility patches)
- `managed_components/` Third-party components (esp_lvgl_port, LCD panels, touch, etc.)
- `boards/` sdkconfig overlays applied per selected vendor/board
- `project_include.cmake` Logic to append the overlay based on Kconfig symbol

## Configuration highlights

- **ESP32-P4 chip revision support**: Configured to support all chip revisions (v1.0 and above) for maximum hardware compatibility
- **Partition table**: 10MB factory app partition, 5MB SPIFFS storage (optimized for larger binaries)
- **LVGL demos**: Only widgets demo enabled by default to reduce binary size (other demos can be enabled in menuconfig)
- LVGL 9 via `espressif__esp_lvgl_port`
- MIPI DSI panel bring-up path: create DSI bus → DBI IO → wrap DPI panel → register display with LVGL
- I2C v2 API for touch and aux devices

## Supported boards

The following ESP32-P4 development boards are supported and can be selected in menuconfig:

| Vendor | Board Name | Flash | URL | Status |
|-------:|:-----------|:------|:----|:-------|
| Espressif | ESP32-P4 Function EV Board | 16MB | [espressif.com](https://www.espressif.com/) | :white_check_mark: Works |
| Olimex | ESP32-P4-DevKit | 16MB | [olimex.com](https://www.olimex.com/Products/IoT/ESP32-P4/ESP32-P4-DevKit/) | :white_check_mark: Works |
| DFRobot | FireBeetle 2 ESP32-P4 | 16MB | [dfrobot.com](https://www.dfrobot.com/product-2841.html) | :white_check_mark: Works |
| Waveshare | ESP32-P4-Module-DEV-KIT | 16MB | [waveshare.com](https://www.waveshare.com/esp32-p4-module-dev-kit.htm) | :white_check_mark: Works |
| Guition | JC-ESP32P4-M3 DEV Board | 16MB | [surenoo.com](https://www.surenoo.com/products/27872758) | :white_check_mark: Works |

All boards use the same general I2C and DSI PHY configuration. Board-specific settings (flash mode, frequency) are applied automatically via sdkconfig overlays in `boards/`.

## Supported displays (growing)

Planned to be added incrementally as part-time development progresses.

| Vendor | Model | Resolution (WxH)* | Interface | Touch | Status |
|-------:|:------|:-----------|:----------|:------|:-------|
| BuyDisplay | [5" IPS TFT ER-TFT050-10][id5] | 720x1280 | DSI (2-lanes) | GT911 | :white_check_mark: Works |
| DFRobot | [5" DSI (SKU: DFR0550-V2)][id2] | 800x480 | DSI (1-lane) | FT5x06 | :white_check_mark: Works |
| Luckfox | [5" DSI (SKU: 28560)][id1] | 800x480 | DSI (1-lane) | FT5x06 | :white_check_mark: Works |
| RaspberyPi | [RPi 5" Touch Display V2][id3] | 720x1280 | DSI (2-lanes) | GT911 | :white_check_mark: Works |
| RaspberyPi | [RPi 7" Touch Display V2][id3] | 720x1280 | DSI (2-lanes) | GT911 | :white_check_mark: Works |
| Amelin | [7" 1024x600 LCD T D][id4] | 1024x600 | DSI (2-lanes) | GT911 | In progress |
| Waveshare | TBD | — | DSI | — | Planned |
| SeeedStudio | TBD | — | DSI | — | Planned |

> *Default Resolution. Display orientation can be changed in the firmware.

Contributions and test reports are welcome.

[id1]: https://www.luckfox.com/Displays/EN-5inch-DSI-Touchscreen
[id2]: https://www.dfrobot.com/product-2791.html
[id3]: https://www.raspberrypi.com/products/touch-display-2/
[id4]: https://www.vip-lcd.com/7-Inch-LCD-Touch-Display-Screen-1024-600-LVDS-Interface-with-Touch-Panel-7-0-Inch-Lcd-Module-pd591986658.html
[id5]: https://www.buydisplay.com/5-inch-720x1280-ips-tft-lcd-display-mipi-interface-ili9881-controller

## Quick tips for troubleshooting

During bring-up across multiple DSI panels, configuration changes can occasionally leave the panel in an unresponsive state. If you flash the ESP32‑P4 with different DSI settings while the same panel remains connected, the panel might not reflect the new configuration. In that case:

- **Switching ESP-IDF versions**: If you're switching between ESP-IDF v5.5.3 and v6.0 (or vice versa), you **must** perform a clean build to avoid compatibility issues. Delete the following before rebuilding:
  ```bash
  # Clean all cached build artifacts and configuration
  rm -rf build/
  rm -f sdkconfig
  rm -f dependencies.lock
  # Then reconfigure and rebuild
  idf.py reconfigure
  idf.py build
  ```
  This ensures cached build artifacts, component dependencies, and configuration files from the previous IDF version don't interfere with the new build.

- Fully power-cycle the system (board and display) to clear any latched state in the panel or bridge.
- If the display shows scrambled frames, review the `LCD_MIPI_DSI_LANE_BITRATE_MBPS` setting and lower it as needed. A typical stable range is 600–1500 Mbps.
- If the touch controller doesn’t initialize or the LCD (e.g., RPi 7") doesn’t power up, reduce the I2C bus speed. Some panels don’t tolerate higher rates. For example, the Luckfox 5" touch only initialized reliably at ≤100 kHz.

## Pictures

Below are a few photos from the project setup and output. Images are stored under `pictures/`.

![Luckfox 5" Touch Display](pictures/01.jpg)

![Picture 02](pictures/02.jpg)

![Picture 03](pictures/03.jpg)

![Guition ESP32-P4 devkit](pictures/04.jpg)

![Picture 05](pictures/05.jpg)

![Picture 06](pictures/06.jpg)

![Picture 07](pictures/07.jpg)

![Espressif ESP32-P4 Function Board](pictures/08.jpg)

## License

See `LICENSE` for details.
