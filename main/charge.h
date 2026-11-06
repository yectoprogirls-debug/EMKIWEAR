#ifndef PKWATCH_CHARGE_H
#define PKWATCH_CHARGE_H

/*
 * charge.h
 * ---------------------------------------------------------------------------
 * Detecta la carga de la bateria leyendo el LED del cargador y detiene la
 * transmision BLE mientras el LED este encendido. El analisis y la pantalla
 * siguen funcionando.
 *
 *   Nodo del LED de carga -> GPIO1
 *
 * ANTES DE CONECTAR: medir con multimetro el nodo del LED respecto a GND, con
 * el cargador enchufado. Nunca debe superar 3.3 V en el GPIO; si supera ese
 * valor, usar un divisor resistivo. Las tierras deben ser comunes.
 *
 * Si el pin queda sin conectar, se lee "sin carga" y el BLE funciona normal.
 * ---------------------------------------------------------------------------
 */

#ifdef __cplusplus
extern "C" {
#endif

#define CHARGE_DET_GPIO      1

/* Nivel del GPIO con el LED ENCENDIDO. 0 = el cargador lleva el nodo a GND
 * (caso tipico, CHRG en drenador abierto; usa pull-up interno). 1 = el nodo
 * sube al encender el LED (usa pull-down interno). */
#define CHARGE_ACTIVE_LEVEL  0

/* Configura el pin, aplica el estado inicial y arranca la tarea "charge".
 * Llamar ANTES de ble_stream_init(), para no publicitar si ya esta cargando. */
void charge_init(void);

#ifdef __cplusplus
}
#endif

#endif /* PKWATCH_CHARGE_H */
