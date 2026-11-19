#ifndef PKWATCH_CHARGE_H
#define PKWATCH_CHARGE_H

/*
 * charge.h
 * ---------------------------------------------------------------------------
 * MODO CARGA: consumo minimo mientras el reloj carga. Lo activa UNICAMENTE la
 * app, con el comando 0xC0 de la caracteristica CMD (ver ble_stream.h). No hay
 * deteccion de carga por hardware.
 *
 * Al entrar: para el muestreo y los datos, avisa a la app (BATT = 0xFF),
 * cierra la conexion y apaga el BLE, pone el BMI270 en suspend, apaga la
 * pantalla y el LED, y entra en light sleep.
 *
 * Sale con una pulsacion de 2 s del boton (GPIO8) o al pasar
 * CHARGE_SLEEP_HOURS; entonces enciende todo y vuelve a publicitar.
 * ---------------------------------------------------------------------------
 */

#ifdef __cplusplus
extern "C" {
#endif

/* Horas que el reloj duerme en modo carga si nadie lo despierta con el boton. */
#define CHARGE_SLEEP_HOURS   72

/* Arranca la tarea "charge", que espera el comando de la app. */
void charge_init(void);

/* Pide entrar en modo carga. Solo avisa a la tarea "charge" (notificacion de
 * FreeRTOS) y vuelve en el acto: se puede llamar desde el callback BLE. */
void charge_mode_request(void);

#ifdef __cplusplus
}
#endif

#endif /* PKWATCH_CHARGE_H */
