#ifndef DISPLAY_GFX_H
#define DISPLAY_GFX_H

/*
 * display_gfx.h
 * ---------------------------------------------------------------------------
 * Dibujo 2D puro sobre un framebuffer RGB565 en RAM, SIN dependencias de
 * ESP-IDF (ni esp_lcd, ni FreeRTOS): display.c es quien envia el framebuffer
 * a la pantalla. Pensado para el panel redondo de 240x240 (GC9A01).
 * ---------------------------------------------------------------------------
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GFX_W 240
#define GFX_H 240

/* Color RGB565 con los bytes ya intercambiados para el bus SPI (little-endian
 * desde el host, big-endian como lo espera el panel). */
static inline uint16_t display_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t c = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    return (uint16_t)((c >> 8) | (c << 8));
}

/* Llena todo el framebuffer (GFX_W*GFX_H) con 'color'. */
void gfx_fill(uint16_t *fb, uint16_t color);

/* Texto 5x7 (se pasa a mayusculas; solo ASCII 0x20..0x5F, el resto se ve como
 * espacio). 'scale' >= 1. Se recorta automaticamente si algun glifo cae fuera
 * de 0..GFX_W-1 / 0..GFX_H-1. */
void gfx_text(uint16_t *fb, int x, int y, const char *s, int scale,
              uint16_t fg, uint16_t bg);

/* Ancho real en pixeles de 's' a la escala dada: (n*6-1)*scale (n = longitud;
 * cada caracter avanza 6*scale px, menos el ultimo espacio tras el final). */
int gfx_text_width(const char *s, int scale);

/* Como gfx_text, pero centrado horizontalmente en la fila 'y'. */
void gfx_text_centered(uint16_t *fb, int y, const char *s, int scale,
                       uint16_t fg, uint16_t bg);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_GFX_H */
