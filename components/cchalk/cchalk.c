#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include "cchalk.h"

static void _chalk_print(bool with_line, chalk_color_t color, chalk_color_t background, chalk_color_t text_decoration, const char *format, va_list list);
static void _chalk_printf_rgb(bool is_background, uint8_t r, uint8_t g, uint8_t b, bool with_line, const char *format, va_list list);

static void _chalk_printf_rgb(bool is_background, uint8_t r, uint8_t g, uint8_t b, bool with_line, const char *format, va_list list)
{
    printf("\e[%d;2;%d;%d;%dm", is_background ? 48 : 38, r, g, b);
    vprintf(format, list);
    printf("\033[0m");
    if (with_line)
        printf("\n");
}

void chalk_printf_background_rgb_ln(uint8_t r, uint8_t g, uint8_t b, uint8_t bg_r, uint8_t bg_g, uint8_t bg_b, const char *format, ...)
{
    va_list list;
    va_start(list, format);
    _chalk_printf_rgb(true, r, g, b, true, format, list);
    va_end(list);
}

void chalk_printf_background_rgb(uint8_t r, uint8_t g, uint8_t b, uint8_t bg_r, uint8_t bg_g, uint8_t bg_b, const char *format, ...)
{
    va_list list;
    va_start(list, format);
    _chalk_printf_rgb(true, r, g, b, false, format, list);
    va_end(list);
}

void chalk_printf_foreground_rgb_ln(uint8_t r, uint8_t g, uint8_t b, const char *format, ...)
{
    va_list list;
    va_start(list, format);
    _chalk_printf_rgb(false, r, g, b, true, format, list);
    va_end(list);
}

void chalk_printf_foreground_rgb(uint8_t r, uint8_t g, uint8_t b, const char *format, ...)
{
    va_list list;
    va_start(list, format);
    _chalk_printf_rgb(false, r, g, b, false, format, list);
    va_end(list);
}

static void _chalk_printf_foreground_background_rgb(bool with_line, uint8_t r, uint8_t g, uint8_t b, uint8_t bg_r, uint8_t bg_g, uint8_t bg_b, const char *format, va_list list)
{
    printf("\e[38;2;%d;%d;%dm\e[48;2;%d;%d;%dm", r, g, b, bg_r, bg_g, bg_b);
    vprintf(format, list);
    printf("\033[0m");
    if (with_line)
        printf("\n");
}

void chalk_rgb(uint8_t r, uint8_t g, uint8_t b, uint8_t bg_r, uint8_t bg_g, uint8_t bg_b, const char *format, ...)
{
    va_list list;
    va_start(list, format);
    _chalk_printf_foreground_background_rgb(false, r, g, b, bg_r, bg_g, bg_b, format, list);
    va_end(list);
}

void chalk_rgb_ln(uint8_t r, uint8_t g, uint8_t b, uint8_t bg_r, uint8_t bg_g, uint8_t bg_b, const char *format, ...)
{
    va_list list;
    va_start(list, format);
    _chalk_printf_foreground_background_rgb(true, r, g, b, bg_r, bg_g, bg_b, format, list);
    va_end(list);
}

void chalk(bool with_line, chalk_color_t color, chalk_color_t background, chalk_color_t text_decoration, const char *format, ...)
{
    va_list list;
    va_start(list, format);
    _chalk_print(with_line, color, background, text_decoration, format, list);
    va_end(list);
}

void chalk_printf(chalk_color_t color, const char *format, ...)
{
    va_list list;
    va_start(list, format);
    _chalk_print(false, color, (chalk_color_t)NULL, (chalk_color_t)NULL, format, list);
    va_end(list);
}

void chalk_printf_decoration(chalk_color_t color, chalk_color_t decoration, const char *format, ...)
{
    va_list list;
    va_start(list, format);
    _chalk_print(false, color, (chalk_color_t)NULL, decoration, format, list);
    va_end(list);
}

void chalk_printf_background(chalk_color_t color, chalk_color_t background, const char *format, ...)
{
    va_list list;
    va_start(list, format);
    _chalk_print(false, color, background, (chalk_color_t)NULL, format, list);
    va_end(list);
}

void chalk_printf_ln(chalk_color_t color, const char *format, ...)
{
    va_list list;
    va_start(list, format);
    _chalk_print(true, color, (chalk_color_t)NULL, (chalk_color_t)NULL, format, list);
    va_end(list);
}

void chalk_printf_decoration_ln(chalk_color_t color, chalk_color_t decoration, const char *format, ...)
{
    va_list list;
    va_start(list, format);
    _chalk_print(true, color, (chalk_color_t)NULL, decoration, format, list);
    va_end(list);
}

void chalk_printf_background_ln(chalk_color_t color, chalk_color_t background, const char *format, ...)
{
    va_list list;
    va_start(list, format);
    _chalk_print(true, color, background, (chalk_color_t)NULL, format, list);
    va_end(list);
}

static void _chalk_print(bool with_line, chalk_color_t color, chalk_color_t background, chalk_color_t text_decoration, const char *format, va_list list)
{

    if (color < 100)
        printf("\033[%dm", color);
    else if (color > 100 && color < 1000)
        printf("\033[%d;1m", color - 100);
    else
        printf("\033[38;5;%dm", color - 1000);
    if (background)
    {
        if (background < 100)
            printf("\033[%dm", background + 10);
        else if (background > 100 && background < 1000)
            printf("\033[%d;1m", background - 100 + 10);
        else
            printf("\033[48;5;%dm", background - 1000);
    }
    if (text_decoration > CHALK_BOLD && text_decoration < CHALK_DOUBLE_UNDERLINE)
    {
        printf("\033[%dm", text_decoration);
    }

    vprintf(format, list);
    printf("\033[0m");
    if (with_line)
        printf("\n");
}
