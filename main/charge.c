/*
 * charge.c  -- ver charge.h
 *
 * El callback BLE del comando 0xC0 solo avisa a la tarea "charge" con una
 * notificacion de FreeRTOS; la secuencia de apagado, el light sleep y la
 * vuelta se ejecutan en esa tarea.
 */

#include "charge.h"
#include "ble_stream.h"
#include "buttons.h"
#include "display.h"
#include "test_bmi270.h"

#include <stdint.h>

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CHARGE";

#define CHARGE_TASK_STACK     4096
#define CHARGE_TASK_PRIO      3

#define CHARGE_SLEEP_US       ((int64_t)CHARGE_SLEEP_HOURS * 3600 * 1000000LL)
#define CHARGE_NOTICE_MS      500     /* margen para que BATT = 0xFF llegue a la app */
#define CHARGE_WAKE_HOLD_MS   2000    /* pulsacion larga que saca del modo carga */
#define CHARGE_BTN_POLL_MS    20
#define CHARGE_BTN_SETTLE_MS  30      /* rebote del boton */
#define CHARGE_RELEASE_MAX_MS 10000   /* espera maxima a que suelten el boton */

static TaskHandle_t s_task;

void charge_mode_request(void)
{
    if (s_task) {
        xTaskNotifyGive(s_task);
    } else {
        ESP_LOGW(TAG, "Modo carga solicitado, pero la tarea de carga no existe");
    }
}

static void charge_mode_enter(void)
{
    /* a */
    ESP_LOGI(TAG, "Modo carga solicitado");

    /* desde ya, el boton no alterna la pantalla */
    buttons_set_suspended(true);

    /* b: fin del muestreo y de las notificaciones de datos; lo pendiente se
     *    descarta (la FIFO se vacia al reanudar) */
    if (!test_bmi270_pause()) {
        ESP_LOGW(TAG, "la tarea de la IMU no confirmo la pausa");
    }
    ble_stream_stop_data();

    /* c: aviso a la app y margen para que salga por el aire */
    ble_stream_push_battery(BLE_BATT_CHARGE_MODE);
    vTaskDelay(pdMS_TO_TICKS(CHARGE_NOTICE_MS));

    /* d: desconexion ordenada, fin del advertising y stack BLE apagado */
    ble_stream_deinit();

    /* e: BMI270 en suspend */
    if (!test_bmi270_sensor_sleep()) {
        ESP_LOGW(TAG, "el BMI270 no entro en suspend");
    }

    /* f: misma funcion que usa el boton: corta BLK y manda DISPOFF + SLPIN (0x10) */
    display_set_power(false);

    /* g: WS2812 con los tres canales en 0 */
    buttons_led_off();

    /* h: BLK fijo en apagado durante el sueno */
    display_backlight_hold(true);
}

static const char *wakeup_name(esp_sleep_wakeup_cause_t cause)
{
    switch (cause) {
    case ESP_SLEEP_WAKEUP_TIMER: return "temporizador";
    case ESP_SLEEP_WAKEUP_GPIO:  return "boton (GPIO)";
    default:                     return "otra";
    }
}

/* true si el boton sigue presionado 'ms' seguidos. */
static bool button_held(int ms)
{
    vTaskDelay(pdMS_TO_TICKS(CHARGE_BTN_SETTLE_MS));
    for (int t = CHARGE_BTN_SETTLE_MS; t < ms; t += CHARGE_BTN_POLL_MS) {
        if (!buttons_pressed()) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(CHARGE_BTN_POLL_MS));
    }
    return buttons_pressed();
}

/* Duerme (light sleep) hasta una pulsacion larga del boton o hasta que pasen
 * CHARGE_SLEEP_HOURS desde la entrada al modo carga. Una pulsacion corta
 * vuelve a dormir por el tiempo que quede, sin reiniciar la cuenta. */
static void charge_mode_sleep(void)
{
    const int64_t deadline_us = esp_timer_get_time() + CHARGE_SLEEP_US;

    for (;;) {
        int64_t left_us = deadline_us - esp_timer_get_time();
        if (left_us <= 0) {
            ESP_LOGI(TAG, "Pasaron %d h: fin del modo carga", CHARGE_SLEEP_HOURS);
            break;
        }

        /* Con el boton ya presionado el despertar por nivel bajo saltaria en
         * el acto: en ese caso se evalua la pulsacion sin dormir. */
        if (!buttons_pressed()) {
            gpio_wakeup_enable(BTN_TOGGLE_GPIO, GPIO_INTR_LOW_LEVEL);
            esp_sleep_enable_gpio_wakeup();
            esp_sleep_enable_timer_wakeup((uint64_t)left_us);

            ESP_LOGI(TAG, "Entrando en light sleep (quedan %lu min)",
                     (unsigned long)(left_us / 60000000LL));

            int64_t t0_us = esp_timer_get_time();
            esp_err_t err = esp_light_sleep_start();
            int64_t slept_us = esp_timer_get_time() - t0_us;
            esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

            gpio_wakeup_disable(BTN_TOGGLE_GPIO);

            if (err != ESP_OK) {
                /* algo impidio dormir (p.ej. una fuente de despertar ya activa) */
                ESP_LOGW(TAG, "Light sleep rechazado: %s", esp_err_to_name(err));
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            ESP_LOGI(TAG, "Light sleep: durmio %lu s (%lu ms), desperto por %s",
                     (unsigned long)(slept_us / 1000000LL),
                     (unsigned long)(slept_us / 1000LL), wakeup_name(cause));

            if (cause == ESP_SLEEP_WAKEUP_TIMER) {
                break;
            }
            if (cause != ESP_SLEEP_WAKEUP_GPIO) {
                continue;
            }
        }

        if (button_held(CHARGE_WAKE_HOLD_MS)) {
            ESP_LOGI(TAG, "Pulsacion larga (%d ms): fin del modo carga", CHARGE_WAKE_HOLD_MS);
            break;
        }
        ESP_LOGI(TAG, "Pulsacion corta: vuelve a dormir");
    }

    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
}

static void charge_mode_exit(void)
{
    /* pantalla: se libera BLK y display_set_power(true) manda SLPOUT (0x11),
     * redibuja y enciende la luz */
    display_backlight_hold(false);
    display_set_power(true);

    /* BMI270 fuera de suspend y muestreo en marcha */
    test_bmi270_resume();

    /* BLE: stack arriba y publicitando de inmediato para que la app se
     * reconecte */
    ble_stream_init();

    /* El boton vuelve a alternar la pantalla solo cuando ya lo soltaron: asi
     * la pulsacion larga que desperto al reloj no cuenta ademas como toggle. */
    for (int t = 0; buttons_pressed() && t < CHARGE_RELEASE_MAX_MS; t += CHARGE_BTN_POLL_MS) {
        vTaskDelay(pdMS_TO_TICKS(CHARGE_BTN_POLL_MS));
    }
    vTaskDelay(pdMS_TO_TICKS(CHARGE_BTN_SETTLE_MS));
    buttons_set_suspended(false);

    ESP_LOGI(TAG, "Modo carga terminado");
}

static void charge_task(void *arg)
{
    (void)arg;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   /* espera el CMD 0xC0 */

        charge_mode_enter();
        charge_mode_sleep();
        charge_mode_exit();

        ulTaskNotifyTake(pdTRUE, 0);    /* descarta peticiones repetidas de la app */
    }
}

void charge_init(void)
{
    if (xTaskCreate(charge_task, "charge", CHARGE_TASK_STACK, NULL,
                    CHARGE_TASK_PRIO, &s_task) != pdPASS) {
        ESP_LOGE(TAG, "no se pudo crear la tarea de carga");
        s_task = NULL;
        return;
    }
    ESP_LOGI(TAG, "Modo carga listo (CMD 0xC0, %d h de sueno)", CHARGE_SLEEP_HOURS);
}
