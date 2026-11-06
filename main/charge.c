/*
 * charge.c  -- ver charge.h
 *
 * Se lee el pin por polling y solo se acepta un cambio si se mantiene 1 s, para
 * que un contacto flojo al enchufar o un LED parpadeando no corten el BLE.
 */

#include "charge.h"
#include "ble_stream.h"

#include <stdbool.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CHARGE";

#define CHARGE_POLL_MS        250
#define CHARGE_CONFIRM_READS  4       /* 4 x 250 ms = 1 s estable */
#define CHARGE_SETTLE_MS      10      /* el pull estabiliza el pin tras configurarlo */
#define CHARGE_TASK_STACK     3072
#define CHARGE_TASK_PRIO      3

static bool read_charging(void)
{
    return gpio_get_level(CHARGE_DET_GPIO) == CHARGE_ACTIVE_LEVEL;
}

static void charge_task(void *arg)
{
    bool stable = (bool)(uintptr_t)arg;
    int  diff_reads = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(CHARGE_POLL_MS));

        if (read_charging() == stable) {
            diff_reads = 0;
            continue;
        }
        if (++diff_reads < CHARGE_CONFIRM_READS) {
            continue;
        }

        diff_reads = 0;
        stable = !stable;
        ESP_LOGI(TAG, "%s", stable ? "Carga detectada: BLE detenido"
                                   : "Carga terminada: BLE reanudado");
        ble_stream_set_enabled(!stable);
    }
}

void charge_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << CHARGE_DET_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = CHARGE_ACTIVE_LEVEL ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE,
        .pull_down_en = CHARGE_ACTIVE_LEVEL ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config: %s", esp_err_to_name(err));
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(CHARGE_SETTLE_MS));

    bool charging = read_charging();
    ble_stream_set_enabled(!charging);
    ESP_LOGI(TAG, "Deteccion de carga lista (GPIO%d, %s)", CHARGE_DET_GPIO,
             charging ? "cargando: BLE detenido" : "sin carga");

    if (xTaskCreate(charge_task, "charge", CHARGE_TASK_STACK,
                    (void *)(uintptr_t)charging, CHARGE_TASK_PRIO, NULL) != pdPASS) {
        ESP_LOGE(TAG, "no se pudo crear la tarea de carga");
        ble_stream_set_enabled(true);   /* sin vigilancia, no dejar el BLE parado */
    }
}
