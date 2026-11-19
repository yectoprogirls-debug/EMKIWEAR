/*
 * display.c  -- ver display.h
 *
 * Driver del GC9A01 (panel de 240x240) sobre el componente esp_lcd (panel IO
 * SPI + panel gc9a01). Todo el dibujo se arma en un framebuffer en RAM
 * (display_gfx.h, sin dependencias de ESP-IDF) y se envia a la pantalla con
 * UN solo esp_lcd_panel_draw_bitmap() por cuadro, para evitar parpadeo.
 */

#include "display.h"
#include "display_gfx.h"
#include "hourly_summary.h"

#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_gc9a01.h"

static const char *TAG = "DISPLAY";

/* ---- pines (conector J5) ---- */
#define PIN_SCLK  18
#define PIN_MOSI  15
#define PIN_RST    3
#define PIN_DC     2
#define PIN_CS     4
#define PIN_BLK   14

/* nivel que ENCIENDE la retroiluminacion (si la pantalla queda negra con la
 * imagen enviada, cambiar a 0) */
#define BLK_ON_LEVEL 1

#define LCD_HOST    SPI2_HOST
#define LCD_PCLK_HZ (40 * 1000 * 1000)   /* bajar a 20 MHz si hay glitches */

/* Hora local (Colombia, UTC-5, sin horario de verano). */
#define LOCAL_UTC_OFFSET_S (-5 * 3600)

/* ---- Ajustes de orientacion/color del panel GC9A01 ----------------------
 * Si al encender la pantalla algo se ve mal, toca uno de estos 6 defines:
 *   - colores invertidos (negativo de la imagen)       -> LCD_INVERT
 *   - rojo y azul cambiados entre si                    -> LCD_BGR
 *   - imagen en espejo horizontal                        -> LCD_MIRROR_X
 *   - imagen en espejo vertical                          -> LCD_MIRROR_Y
 *   - imagen corrida / con un borde negro a un lado      -> LCD_GAP_X / LCD_GAP_Y
 * ------------------------------------------------------------------------- */
#define LCD_INVERT    true
#define LCD_BGR       true
#define LCD_MIRROR_X  true
#define LCD_MIRROR_Y  false
#define LCD_GAP_X     0
#define LCD_GAP_Y     0

/* Ancho maximo (en pixeles) para el bloque central de texto: debe caber
 * dentro del circulo de radio 120 centrado en (120,120). */
#define CENTER_MAX_W   200
#define CENTER_Y       115
#define CENTER_LINE_GAP 8

static esp_lcd_panel_handle_t s_panel;
static esp_lcd_panel_io_handle_t s_io;
static uint16_t *s_fb;              /* DISPLAY_W*DISPLAY_H, en RAM DMA */
static bool s_ready;
static SemaphoreHandle_t s_tx_done; /* dado cuando el ultimo draw_bitmap termino */
static SemaphoreHandle_t s_lock;    /* serializa todo acceso a s_fb y al panel */
static bool s_on = true;            /* pantalla encendida (boton alterna) */

/* estado de la cara del reloj (antes eran static dentro de display_show) */
static bool             s_have_last = false;
static tremor_class_t   s_last_category;
static int              s_last_minute;
static bool             s_last_analizando;
static bool             s_force_redraw = false;
static bool             s_have_res     = false;
static tremor_result_t  s_last_res;
static bool             s_last_ble;

/* ================================================================== */

/* Se llama desde ISR cuando el DMA del SPI termina de mandar el framebuffer.
 * Libera s_tx_done para que se pueda volver a escribir en s_fb. */
static bool IRAM_ATTR on_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                                          esp_lcd_panel_io_event_data_t *edata,
                                          void *user_ctx)
{
    (void)panel_io;
    (void)edata;
    SemaphoreHandle_t sem = (SemaphoreHandle_t)user_ctx;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(sem, &woken);
    return woken == pdTRUE;
}

/* Espera (200 ms maximo) a que termine el envio anterior antes de volver a
 * escribir en s_fb. Llamar SIEMPRE antes de empezar a dibujar un cuadro. */
static void wait_prev_blit(void)
{
    xSemaphoreTake(s_tx_done, pdMS_TO_TICKS(200));
}

/* Envia el framebuffer completo de un solo golpe. */
static void flush_fb(void)
{
    esp_lcd_panel_draw_bitmap(s_panel, 0, 0, DISPLAY_W, DISPLAY_H, s_fb);
}

esp_err_t display_init(void)
{
    s_fb = heap_caps_malloc(DISPLAY_W * DISPLAY_H * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!s_fb) {
        ESP_LOGE(TAG, "sin memoria DMA para el framebuffer");
        return ESP_ERR_NO_MEM;
    }

    s_tx_done = xSemaphoreCreateBinary();
    if (!s_tx_done) {
        ESP_LOGE(TAG, "sin memoria para el semaforo de envio");
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreGive(s_tx_done);   /* no hay ningun envio pendiente al arrancar */

    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) {
        ESP_LOGE(TAG, "sin memoria para el mutex de la pantalla");
        return ESP_ERR_NO_MEM;
    }

    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_SCLK,
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DISPLAY_W * 40 * sizeof(uint16_t),
    };
    esp_err_t err = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
        return err;
    }

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = PIN_CS,
        .dc_gpio_num = PIN_DC,
        .spi_mode = 0,
        .pclk_hz = LCD_PCLK_HZ,
        .trans_queue_depth = 10,
        .on_color_trans_done = on_color_trans_done,
        .user_ctx = s_tx_done,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &s_io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "panel_io_spi: %s", esp_err_to_name(err));
        return err;
    }

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_RST,
        .rgb_ele_order = LCD_BGR ? LCD_RGB_ELEMENT_ORDER_BGR : LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    err = esp_lcd_new_panel_gc9a01(s_io, &panel_cfg, &s_panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "new_panel_gc9a01: %s", esp_err_to_name(err));
        return err;
    }

    /* retroiluminacion APAGADA hasta tener el primer cuadro (sin basura) */
    gpio_config_t blk_cfg = {
        .pin_bit_mask = 1ULL << PIN_BLK,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&blk_cfg));
    gpio_set_level(PIN_BLK, !BLK_ON_LEVEL);

    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, LCD_INVERT));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, LCD_MIRROR_X, LCD_MIRROR_Y));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(s_panel, false));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, LCD_GAP_X, LCD_GAP_Y));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    /* primer cuadro (negro) y, cuando termino de enviarse, se enciende la luz:
     * asi el selftest que sigue ya se ve completo */
    wait_prev_blit();
    gfx_fill(s_fb, 0x0000);
    flush_fb();
    wait_prev_blit();
    xSemaphoreGive(s_tx_done);
    gpio_set_level(PIN_BLK, BLK_ON_LEVEL);

    s_ready = true;
    ESP_LOGI(TAG, "GC9A01 listo (SCLK=%d MOSI=%d RST=%d DC=%d CS=%d BLK=%d @ %d MHz)",
             PIN_SCLK, PIN_MOSI, PIN_RST, PIN_DC, PIN_CS, PIN_BLK, LCD_PCLK_HZ / 1000000);
    ESP_LOGI(TAG, "GC9A01 240x240 listo");
    return ESP_OK;
}

/* ------------------------------------------------------------------ */

/* Logo de arranque (mismo que al final de display_selftest). Llamar con
 * s_lock tomado. */
static void draw_logo_locked(void)
{
    wait_prev_blit();
    uint16_t bg = display_rgb(10, 20, 30);
    uint16_t fg = display_rgb(0, 220, 170);
    gfx_fill(s_fb, bg);
    gfx_text_centered(s_fb, 106, "EMKIWEAR", 4, fg, bg);
    flush_fb();
}

void display_selftest(void)
{
    if (!s_ready) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);

    const uint16_t cols[] = {
        display_rgb(255, 0, 0), display_rgb(0, 255, 0), display_rgb(0, 0, 255),
    };
    for (size_t i = 0; i < sizeof(cols) / sizeof(cols[0]); i++) {
        wait_prev_blit();
        gfx_fill(s_fb, cols[i]);
        flush_fb();
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    draw_logo_locked();
    xSemaphoreGive(s_lock);
}

/* --- disenio de la cara segun la categoria: color y 1-2 lineas de texto --- */
static void face_for_category(tremor_class_t cat, uint16_t *bg, uint16_t *fg,
                              const char **line1, const char **line2)
{
    switch (cat) {
    case TREMOR_CLASS_LEVE:
        *bg = display_rgb(250, 210, 0); *fg = display_rgb(0, 0, 0);
        *line1 = "LEVE"; *line2 = NULL;
        break;
    case TREMOR_CLASS_MODERADO:
        *bg = display_rgb(255, 120, 0); *fg = display_rgb(0, 0, 0);
        *line1 = "MODERADO"; *line2 = NULL;
        break;
    case TREMOR_CLASS_GRAVE:
        *bg = display_rgb(210, 0, 0); *fg = display_rgb(255, 255, 255);
        *line1 = "GRAVE"; *line2 = NULL;
        break;
    case TREMOR_CLASS_VOLUNTARIO:
        *bg = display_rgb(0, 80, 220); *fg = display_rgb(255, 255, 255);
        *line1 = "MOVIMIENTO"; *line2 = "VOLUNTARIO";
        break;
    case TREMOR_CLASS_SIN_TEMBLOR:
    default:
        *bg = display_rgb(0, 150, 60); *fg = display_rgb(255, 255, 255);
        *line1 = "SIN"; *line2 = "TEMBLOR";
        break;
    }
}

/* escala entera maxima (hasta 4) tal que la linea mas ancha de las 1-2 que
 * se van a dibujar entre en CENTER_MAX_W pixeles */
static int fit_scale(const char *line1, const char *line2)
{
    for (int scale = 4; scale >= 1; scale--) {
        int w1 = gfx_text_width(line1, scale);
        int w2 = line2 ? gfx_text_width(line2, scale) : 0;
        if (w1 <= CENTER_MAX_W && w2 <= CENTER_MAX_W) {
            return scale;
        }
    }
    return 1;
}

/* Dibuja la cara del reloj. Llamar con s_lock tomado. Si s_force_redraw
 * es true, dibuja aunque no hayan cambiado la clase ni el minuto. */
static void draw_face_locked(const tremor_result_t *res)
{
    /* hora local (Colombia): "--:--" si la app aun no sincronizo por BLE */
    uint32_t epoch_s;
    bool have_time = hs_get_epoch(&epoch_s);
    int minute = -1;
    char timestr[8];
    if (have_time) {
        int64_t local = (int64_t)epoch_s + LOCAL_UTC_OFFSET_S;
        uint32_t secs_of_day = (uint32_t)(local % 86400);
        int hh = (int)((secs_of_day / 3600) % 24);
        minute = (int)((secs_of_day / 60) % 60);
        snprintf(timestr, sizeof(timestr), "%02d:%02d", hh, minute);
    } else {
        snprintf(timestr, sizeof(timestr), "--:--");
    }

    /* solo redibuja si cambio la clase o el minuto desde la ultima vez
     * (o si se pidio un redibujo forzado al encender la pantalla) */
    if (!s_force_redraw && s_have_last &&
        res->category == s_last_category && minute == s_last_minute &&
        res->analizando == s_last_analizando) {
        return;
    }
    s_force_redraw    = false;
    s_have_last       = true;
    s_last_category   = res->category;
    s_last_minute     = minute;
    s_last_analizando = res->analizando;

    uint16_t bg, fg;
    const char *line1, *line2;
    if (res->analizando) {
        /* arranque (calentamiento): "ANALIZANDO" en gris, no "SIN TEMBLOR" */
        bg = display_rgb(110, 110, 110); fg = display_rgb(255, 255, 255);
        line1 = "ANALIZANDO"; line2 = NULL;
    } else {
        face_for_category(res->category, &bg, &fg, &line1, &line2);
    }

    int scale = fit_scale(line1, line2);
    int gh = 7 * scale;

    wait_prev_blit();
    gfx_fill(s_fb, bg);
    gfx_text_centered(s_fb, 40, "EMKIWEAR", 3, fg, bg);

    if (line2) {
        int total_h = 2 * gh + CENTER_LINE_GAP;
        int y1 = CENTER_Y - total_h / 2;
        int y2 = y1 + gh + CENTER_LINE_GAP;
        gfx_text_centered(s_fb, y1, line1, scale, fg, bg);
        gfx_text_centered(s_fb, y2, line2, scale, fg, bg);
    } else {
        int y1 = CENTER_Y - gh / 2;
        gfx_text_centered(s_fb, y1, line1, scale, fg, bg);
    }

    gfx_text_centered(s_fb, 172, timestr, 4, fg, bg);
    flush_fb();
}

void display_show(const tremor_result_t *res, bool ble_connected)
{
    if (!s_ready) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_last_res = *res;
    s_last_ble = ble_connected;
    s_have_res = true;
    if (s_on) {
        draw_face_locked(res);
    }
    xSemaphoreGive(s_lock);
}

/* El driver esp_lcd_gc9a01 no implementa disp_sleep: si la llamada estandar
 * devuelve ESP_ERR_NOT_SUPPORTED se manda el comando SLPIN/SLPOUT directo. */
static void panel_sleep(bool sleep)
{
    esp_err_t err = esp_lcd_panel_disp_sleep(s_panel, sleep);
    if (err == ESP_ERR_NOT_SUPPORTED) {
        err = esp_lcd_panel_io_tx_param(s_io, sleep ? LCD_CMD_SLPIN : LCD_CMD_SLPOUT,
                                        NULL, 0);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "sleep(%d): %s", (int)sleep, esp_err_to_name(err));
    }
}

void display_set_power(bool on)
{
    if (!s_ready) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (on == s_on) {
        xSemaphoreGive(s_lock);
        return;
    }

    wait_prev_blit();   /* termina el ultimo envio DMA */

    if (!on) {
        /* APAGAR: luz fuera, panel apagado y en reposo */
        xSemaphoreGive(s_tx_done);    /* deja el semaforo libre para el proximo cuadro */
        gpio_set_level(PIN_BLK, !BLK_ON_LEVEL);
        esp_lcd_panel_disp_on_off(s_panel, false);
        panel_sleep(true);
        s_on = false;
    } else {
        /* ENCENDER: la luz al final, cuando ya esta el cuadro nuevo */
        xSemaphoreGive(s_tx_done);    /* no hay envio pendiente */
        panel_sleep(false);
        vTaskDelay(pdMS_TO_TICKS(120));
        esp_lcd_panel_disp_on_off(s_panel, true);
        s_force_redraw = true;
        if (s_have_res) {
            draw_face_locked(&s_last_res);   /* redibuja ya el estado actual */
        } else {
            draw_logo_locked();              /* aun no hay resultado: logo */
        }
        wait_prev_blit();             /* espera a que el cuadro llegue al panel */
        xSemaphoreGive(s_tx_done);
        gpio_set_level(PIN_BLK, BLK_ON_LEVEL);
        s_on = true;
    }
    xSemaphoreGive(s_lock);
}

bool display_is_on(void)
{
    return s_on;
}

void display_backlight_hold(bool hold)
{
    if (!s_ready) {
        return;
    }
    if (hold) {
        gpio_set_level(PIN_BLK, !BLK_ON_LEVEL);
        gpio_hold_en(PIN_BLK);
    } else {
        gpio_hold_dis(PIN_BLK);
    }
}
