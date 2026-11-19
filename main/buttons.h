#ifndef PKWATCH_BUTTONS_H
#define PKWATCH_BUTTONS_H

/*
 * buttons.h
 * ---------------------------------------------------------------------------
 * Un solo boton fisico que ALTERNA la pantalla (encendida <-> apagada).
 *
 *   Boton: GPIO8 -> GND
 *
 * Activo en bajo, con pull-up interno (sin resistencias externas).
 * OJO: GPIO8 es pin de strapping del ESP32-C6. Solo se configura como
 * entrada con pull-up, y solo despues de que arranca app_main. No mantener
 * presionado el boton al encender o resetear la placa.
 * ---------------------------------------------------------------------------
 */

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BTN_TOGGLE_GPIO 8

/* Configura el pin, la interrupcion por flanco de bajada y la tarea
 * "buttons" que aplica el antirrebote y llama a display_set_power(). */
void buttons_init(void);

/* ---- Modo carga (ver charge.h) ---- */

/* true: las pulsaciones dejan de alternar la pantalla y se desactiva la
 * interrupcion del pin, que queda libre para despertar del light sleep.
 * false: vuelve al funcionamiento normal descartando lo acumulado; llamar
 * con el boton ya SUELTO para que esa pulsacion no cuente como toggle. */
void buttons_set_suspended(bool suspended);

/* true si el boton esta presionado ahora mismo (lectura directa del pin). */
bool buttons_pressed(void);

/* Apaga el LED WS2812 de la placa, que comparte GPIO8 con el boton: el pin
 * pasa a salida ~1 ms para mandar 24 bits en 0 y vuelve a ser la entrada del
 * boton. No hace nada si el boton esta presionado en ese momento. */
void buttons_led_off(void);

#ifdef __cplusplus
}
#endif

#endif /* PKWATCH_BUTTONS_H */
