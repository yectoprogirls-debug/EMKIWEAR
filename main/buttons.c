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

#include "esp_log.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "driver/gpio.h"
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
        if (gpio_get_level(BTN_TOGGLE_GPIO) != 0) {
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

void buttons_init(void)
{
    s_btn_queue = xQueueCreate(BTN_QUEUE_LEN, sizeof(uint32_t));
    if (!s_btn_queue) {
        ESP_LOGE(TAG, "sin memoria para la cola de botones");
        return;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BTN_TOGGLE_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    esp_err_t err = gpio_config(&io_conf);
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

    ESP_LOGI(TAG, "Boton listo (ALTERNAR pantalla = GPIO%d)", BTN_TOGGLE_GPIO);
}
