/*
 * webui.h — Embedded web UI files (index.html, style.css, app.js)
 *
 * ESP-IDF EMBED_FILES generates symbols:
 *   _binary_<filename>_start  = pointer to file data
 *   _binary_<filename>_end    = pointer past end of data
 *   <filename>_size           = file size (via pointer arithmetic)
 *
 * Usage:
 *   extern const uint8_t index_html_start[] asm("_binary_index_html_start");
 *   extern const uint8_t index_html_end[]   asm("_binary_index_html_end");
 *   size_t len = index_html_end - index_html_start;
 */

#ifndef WEBUI_H
#define WEBUI_H

#include <stdint.h>
#include <stddef.h>

/* Tên file tương ứng với từng URI */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

extern const uint8_t style_css_start[] asm("_binary_style_css_start");
extern const uint8_t style_css_end[]   asm("_binary_style_css_end");

extern const uint8_t app_js_start[]  asm("_binary_app_js_start");
extern const uint8_t app_js_end[]    asm("_binary_app_js_end");

#endif
