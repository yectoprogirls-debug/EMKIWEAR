#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "ble_stream.h"
#include "hourly_summary.h"
#include "display.h"
#include "buttons.h"
#include "charge.h"
#include "test_bmi270.h"

void app_main(void)
{
    /* NVS: lo necesita el stack BLE (NimBLE) para su configuracion. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    hs_init();          /* carga resumenes horarios guardados en NVS */

    if (display_init() == ESP_OK) {
        display_selftest();   /* cicla colores + logo: comprueba el cableado */
        buttons_init();       /* GPIO8 = alternar pantalla ON/OFF */
    }

    charge_init();      /* modo carga (CMD 0xC0 de la app) */
    ble_stream_init();

    xTaskCreate(test_bmi270_task, "test_bmi270", 8192, NULL, 5, NULL);
}
