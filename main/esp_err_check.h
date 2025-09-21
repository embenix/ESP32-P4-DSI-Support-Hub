#pragma once

#include "esp_err.h"
#include "esp_check.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Assert on error, if selected in menuconfig. Otherwise return error code. */
#if CONFIG_ESP_ERROR_CHECK
#define ESP_ERROR_CHECK_RETURN_ERR(x)   do { esp_err_t _e = (x); ESP_ERROR_CHECK(_e); if (_e != ESP_OK) return _e; } while (0)
#define ESP_ERROR_CHECK_RETURN_NULL(x)  do { esp_err_t _e = (x); ESP_ERROR_CHECK(_e); if (_e != ESP_OK) return NULL; } while (0)
#define ESP_NULL_CHECK(x, ret)          do { if ((x) == NULL) { return (ret); } } while (0)
#define ESP_NULL_CHECK_GOTO(x, tag)     do { if ((x) == NULL) { goto tag; } } while (0)
#else
#define ESP_ERROR_CHECK_RETURN_ERR(x) do { \
        esp_err_t err_rc_ = (x);            \
        if (err_rc_ != ESP_OK) {            \
            return err_rc_;                 \
        }                                   \
    } while(0)

#define ESP_ERROR_CHECK_RETURN_NULL(x)  do { \
        if ((x) != ESP_OK) {                \
            return NULL;                    \
        }                                   \
    } while(0)

#define ESP_NULL_CHECK(x, ret) do { \
        if ((x) == NULL) {          \
            return ret;             \
        }                           \
    } while(0)

#define ESP_NULL_CHECK_GOTO(x, goto_tag) do { \
        if ((x) == NULL) {      \
            goto goto_tag;      \
        }                       \
    } while(0)
#endif

#ifdef __cplusplus
}
#endif
