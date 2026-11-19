#ifndef PKWATCH_DISPLAY_H
#define PKWATCH_DISPLAY_H

/*
 * display.h
 * ---------------------------------------------------------------------------
 * Pantalla redonda GC9A01 (240x240, TFT) por SPI de 4 hilos.
 *
 * Cableado con el ESP32-C6 (conector J5):
 *   VCC -> 3V3      GND -> GND
 *   SCL -> GPIO18   (reloj  SPI, SCLK)
 *   SDA -> GPIO15   (datos  SPI, MOSI)   <- OJO: GPIO15 es pin de strapping en
 *                                           el C6 (selecciona la fuente JTAG).
 *                                           Suele ir bien; si el arranque se
 *                                           vuelve inestable, mover a otro GPIO.
 *   RES -> GPIO3    DC -> GPIO2
 *   CS  -> GPIO4    <- GPIO4 tambien es pin de strapping del C6; como salida
 *                      y en alto en reposo no deberia afectar el arranque.
 *   BLK -> GPIO14   (retroiluminacion, BLK_ON_LEVEL en display.c)
 * ---------------------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "tremor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DISPLAY_W  240
#define DISPLAY_H  240

/* Inicializa SPI + panel GC9A01. Llamar una vez. */
esp_err_t display_init(void);

/* Auto-test de arranque: cicla colores para comprobar el cableado. */
void display_selftest(void);

/* Cara del reloj: fondo + texto segun la clase del temblor, hora local
 * abajo. Solo redibuja si cambio la clase o el minuto desde la ultima vez
 * (si no, no hace nada). 'ble_connected' no se usa. */
void display_show(const tremor_result_t *res, bool ble_connected);

/* Enciende (true) o apaga (false) la pantalla. Al apagar corta la
 * retroiluminacion (BLK) y deja el panel en reposo. Al encender redibuja el
 * ultimo estado recibido por display_show(), o el logo si aun no hay
 * ninguno, y enciende la luz al final.
 * Mientras esta apagada, display_show() solo guarda el estado. */
void display_set_power(bool on);

/* true si la pantalla esta encendida. */
bool display_is_on(void);

/* true: fuerza BLK al nivel de apagado y lo fija con gpio_hold_en, para que
 * no cambie durante el light sleep. false: libera el pin (gpio_hold_dis).
 * Usar con la pantalla ya apagada por display_set_power(false), y liberar
 * antes de display_set_power(true). */
void display_backlight_hold(bool hold);

#ifdef __cplusplus
}
#endif

#endif /* PKWATCH_DISPLAY_H */
