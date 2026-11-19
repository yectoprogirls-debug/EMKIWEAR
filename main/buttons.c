/*
 * buttons.c  -- ver buttons.h
 *
 * El ISR solo manda el numero de pin a una cola. La tarea "buttons" hace el
 * antirrebote (30 ms + confirmacion del nivel + 300 ms entre pulsaciones
 * validas), alterna la pantalla con display_set_power() (que es la que habla
 * por SPI) y espera a que se suelte el boton: una pulsacion = un cambio.
 */

#include "buttons.h"
#include "display.h"

#include <stdint.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "BUTTONS";

#define BTN_DEBOUNCE_MS   30
#define BTN_MIN_GAP_MS    300
#define BTN_POLL_MS       20      /* polling hasta que se suelte el boton */
#define BTN_RELEASE_MAX_MS 5000   /* limite de espera a que se suelte */
#define BTN_QUEUE_LEN     10
#define BTN_TASK_STACK    3072
#define BTN_TASK_PRIO     4

static QueueHandle_t s_btn_queue;
static bool          s_isr_ready;   /* buttons_init() llego a registrar el ISR */
static volatile bool s_suspended;   /* modo carga: las pulsaciones no alternan la pantalla */

/* ---- LED WS2812 que comparte GPIO8 con el boton ---- */
#define WS2812_RMT_HZ      (10 * 1000 * 1000)   /* 1 tick = 0.1 us */
#define WS2812_T0H_TICKS   3                    /* bit "0": 0.3 us en alto ... */
#define WS2812_T0L_TICKS   9                    /* ... y 0.9 us en bajo */
#define WS2812_BITS        24                   /* G, R, B */

static void IRAM_ATTR button_isr(void *arg)
{
    uint32_t pin = (uint32_t)arg;
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(s_btn_queue, &pin, &woken);
    if (woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void buttons_task(void *arg)
{
    (void)arg;
    int64_t last_us = INT64_MIN / 2;

    for (;;) {
        uint32_t pin;
        if (xQueueReceive(s_btn_queue, &pin, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        (void)pin;

        /* antirrebote: espera 30 ms y confirma que el pin sigue en bajo */
        vTaskDelay(pdMS_TO_TICKS(BTN_DEBOUNCE_MS));
        if (s_suspended || gpio_get_level(BTN_TOGGLE_GPIO) != 0) {
            continue;
        }

        /* ignora pulsaciones a menos de 300 ms de la ultima valida */
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_us < (int64_t)BTN_MIN_GAP_MS * 1000) {
            continue;
        }
        last_us = now_us;

        /* alterna: encendida -> apagada, apagada -> encendida */
        display_set_power(!display_is_on());
        ESP_LOGI(TAG, "Pantalla %s", display_is_on() ? "ON" : "OFF");

        /* espera a que se SUELTE (maximo 5 s) y descarta los rebotes que
         * llegaron mientras estaba presionado */
        for (int t = 0; t < BTN_RELEASE_MAX_MS && gpio_get_level(BTN_TOGGLE_GPIO) == 0;
             t += BTN_POLL_MS) {
            vTaskDelay(pdMS_TO_TICKS(BTN_POLL_MS));
        }
        xQueueReset(s_btn_queue);
    }
}

/* GPIO8 como boton: entrada con pull-up y flanco de bajada. */
static esp_err_t button_pin_config(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BTN_TOGGLE_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    return gpio_config(&io_conf);
}

void buttons_set_suspended(bool suspended)
{
    if (suspended) {
        s_suspended = true;
        gpio_intr_disable(BTN_TOGGLE_GPIO);
        return;
    }

    /* gpio_wakeup_enable() deja el pin en interrupcion por NIVEL: se vuelve
     * al flanco de bajada antes de reactivar el ISR, y se tira lo acumulado */
    gpio_set_intr_type(BTN_TOGGLE_GPIO, GPIO_INTR_NEGEDGE);
    if (s_btn_queue) {
        xQueueReset(s_btn_queue);
    }
    if (s_isr_ready) {
        gpio_intr_enable(BTN_TOGGLE_GPIO);
    }
    s_suspended = false;
}

bool buttons_pressed(void)
{
    return gpio_get_level(BTN_TOGGLE_GPIO) == 0;
}

void buttons_led_off(void)
{
    /* Con el boton presionado el pin esta puesto a GND: no se maneja como
     * salida contra ese corto. */
    if (buttons_pressed()) {
        ESP_LOGW(TAG, "boton presionado: no se envia el apagado al WS2812");
        return;
    }

    gpio_intr_disable(BTN_TOGGLE_GPIO);   /* la trama no debe verse como pulsaciones */

    rmt_channel_handle_t chan = NULL;
    rmt_encoder_handle_t enc  = NULL;
    rmt_tx_channel_config_t chan_cfg = {
        .gpio_num          = BTN_TOGGLE_GPIO,
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = WS2812_RMT_HZ,
        .mem_block_symbols = 48,
        .trans_queue_depth = 1,
    };
    rmt_copy_encoder_config_t enc_cfg = {};

    esp_err_t err = rmt_new_tx_channel(&chan_cfg, &chan);
    if (err == ESP_OK) {
        err = rmt_new_copy_encoder(&enc_cfg, &enc);
    }
    if (err == ESP_OK) {
        err = rmt_enable(chan);
    }
    if (err == ESP_OK) {
        /* 24 bits "0" = verde, rojo y azul en 0 */
        static rmt_symbol_word_t zeros[WS2812_BITS];
        for (int i = 0; i < WS2812_BITS; i++) {
            zeros[i].level0    = 1;
            zeros[i].duration0 = WS2812_T0H_TICKS;
            zeros[i].level1    = 0;
            zeros[i].duration1 = WS2812_T0L_TICKS;
        }
        rmt_transmit_config_t tx_cfg = { .loop_count = 0 };   /* termina en bajo */
        err = rmt_transmit(chan, enc, zeros, sizeof(zeros), &tx_cfg);
        if (err == ESP_OK) {
            err = rmt_tx_wait_all_done(chan, 100);
        }
        vTaskDelay(pdMS_TO_TICKS(10));     /* linea en bajo: el LED fija el dato */
        rmt_disable(chan);
    }
    if (enc) {
        rmt_del_encoder(enc);
    }
    if (chan) {
        rmt_del_channel(chan);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "apagado del WS2812: %s", esp_err_to_name(err));
    }

    /* el pin vuelve a ser el boton (gpio_config reactiva la interrupcion) */
    button_pin_config();
    if (s_suspended || !s_isr_ready) {
        gpio_intr_disable(BTN_TOGGLE_GPIO);
    }
    if (s_btn_queue) {
        xQueueReset(s_btn_queue);
    }
}

void buttons_init(void)
{
    s_btn_queue = xQueueCreate(BTN_QUEUE_LEN, sizeof(uint32_t));
    if (!s_btn_queue) {
        ESP_LOGE(TAG, "sin memoria para la cola de botones");
        return;
    }

    esp_err_t err = button_pin_config();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config: %s", esp_err_to_name(err));
        return;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {   /* INVALID_STATE = ya instalado */
        ESP_LOGE(TAG, "gpio_install_isr_service: %s", esp_err_to_name(err));
        return;
    }

    if (xTaskCreate(buttons_task, "buttons", BTN_TASK_STACK, NULL,
                    BTN_TASK_PRIO, NULL) != pdPASS) {
        ESP_LOGE(TAG, "no se pudo crear la tarea de botones");
        return;
    }

    gpio_isr_handler_add(BTN_TOGGLE_GPIO, button_isr, (void *)(uint32_t)BTN_TOGGLE_GPIO);
    s_isr_ready = true;

    ESP_LOGI(TAG, "Boton listo (ALTERNAR pantalla = GPIO%d)", BTN_TOGGLE_GPIO);
}
