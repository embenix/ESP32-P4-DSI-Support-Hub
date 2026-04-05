# Local Component Overrides for ESP-IDF v6.0 Compatibility

This directory contains modified versions of ESP component registry packages that have been patched for **ESP-IDF v6.0** compatibility while maintaining backward compatibility with **ESP-IDF v5.5.3+**.

## Why Local Components?

ESP-IDF v6.0 introduced breaking API changes in the color/display subsystem. The following components from the ESP registry have not yet been updated by their maintainers. We maintain patched versions here with conditional compilation to support both v5.5.3+ and v6.0.

## Modified Components

### 1. `esp_lvgl_port`
- **Original**: `espressif/esp_lvgl_port` v2.6.2 from ESP Component Registry
- **Changes**: Updated PPA (Pixel Processing Accelerator) color API
  - Changed `color_space_t` + `pixel_format` → `color_mode` 
  - Updated to use `PPA_SRM_COLOR_MODE_*` constants (v6.0)
  - Added conditional compilation for IDF v6.0 compatibility
- **Files Modified**:
  - `src/common/ppa/lcd_ppa.h`
  - `src/common/ppa/lcd_ppa.c`
  - `src/lvgl9/esp_lvgl_port_disp.c`

### 2. `esp_lcd_dsi`
- **Original**: `waveshare/esp_lcd_dsi` from ESP Component Registry
- **Changes**: Updated panel config API for ESP-IDF v6.0
  - Added conditional check for `rgb_ele_order` (v6.0) vs `color_space` (v5.x)
  - Added `esp_idf_version.h` include for version checking
- **Files Modified**:
  - `esp_lcd_dsi.c`

## How This Works

ESP-IDF's component system gives **priority to local components** over managed components. When you build:

1. Components in this `components/` directory are used first
2. If a component isn't found here, ESP-IDF looks in `managed_components/`
3. If still not found, it downloads from the component registry

## For Users

**No action needed!** Just clone and build with either ESP-IDF v5.5.3+ or v6.0. The build system automatically uses these patched components, and conditional compilation ensures compatibility with your IDF version.

## For Developers

If you need to update these components:

1. Check if official ESP-IDF v6.0-compatible versions are available on the component registry
2. If yes, you can remove this directory and update `main/idf_component.yml`
3. If no, apply similar patches to new versions if updating
4. **Important**: Maintain conditional compilation (`#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)`) for backward compatibility

## Future

These local overrides can be removed once the upstream components are updated for ESP-IDF v6.0 compatibility. Monitor:
- https://components.espressif.com/components/espressif/esp_lvgl_port
- https://components.espressif.com/components/waveshare/esp_lcd_dsi

---

**Compatibility**: ESP-IDF v5.5.3+ and v6.0 through conditional compilation.
