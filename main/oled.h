#pragma once

#include <stdint.h>

/* ════════════════════════════════════════
   API
   ════════════════════════════════════════ */
void oled_init(void);
void oled_clear(void);

/**
 * In chuỗi lên OLED tại hàng page (0–7).
 * Tối đa 16 ký tự / hàng (font 8×8, màn 128 px).
 */
void oled_print_line(uint8_t page, const char *str);