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

#ifdef __cplusplus
extern "C" {
#endif

#define BTN_TOGGLE_GPIO 8

/* Configura el pin, la interrupcion por flanco de bajada y la tarea
 * "buttons" que aplica el antirrebote y llama a display_set_power(). */
void buttons_init(void);

#ifdef __cplusplus
}
#endif

#endif /* PKWATCH_BUTTONS_H */
