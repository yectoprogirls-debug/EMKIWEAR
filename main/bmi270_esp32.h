#ifndef BMI270_ESP32_H
#define BMI270_ESP32_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Datos ya convertidos a unidades físicas
typedef struct {
    float ax, ay, az; // aceleración en g (1g = 9.80665 m/s^2)
    float gx, gy, gz; // velocidad angular en grados/segundo (dps)
} bmi270_data_t;

// Muestra CRUDA (LSB del ADC, sin escalar). Para convertir:
//   accel [m/s^2] = lsb * (ACC_RANGE_G * 9.80665f / 32768.0f)
//   gyro  [dps]   = lsb * (GYR_RANGE_DPS / 32768.0f)
// con ACC_RANGE_G=4 y GYR_RANGE_DPS=500 (ver bmi270_sensor_init).
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} bmi270_raw_sample_t;

// Inicializa el bus I2C y el sensor BMI270 (carga config interna,
// configura accel+gyro a 50Hz fijo, y los habilita). Llamar una sola vez.
esp_err_t bmi270_sensor_init(void);

// Lee una muestra nueva del sensor. Devuelve true si los datos son
// válidos (accel y gyro con dato nuevo listo), false si aún no hay
// dato nuevo o hubo un error de comunicación.
//
// OJO: lee "el dato de ahora mismo" de los registros directos (no la
// FIFO); a igual sondeo que ODR puede repetir una muestra ocasionalmente.
// Para una captura garantizada sin perdidas ni duplicados usa la FIFO
// (bmi270_fifo_enable / bmi270_fifo_read) en vez de esta funcion.
bool bmi270_sensor_read(bmi270_data_t *out);

// ---- Bajo consumo (modo carga) ----

// Pone el BMI270 en suspend (accel+gyro apagados, advanced power save) y lo
// saca de ahi. La configuracion (ODR, rangos, FIFO) se conserva; resume vacia
// la FIFO. Llamar desde la MISMA tarea que lee el sensor (no hay mutex de I2C).
// Devuelven ESP_ERR_INVALID_STATE si el sensor no llego a inicializarse.
esp_err_t bmi270_sensor_suspend(void);
esp_err_t bmi270_sensor_resume(void);

// ---- FIFO: captura sin perdidas ni duplicados (para la característica 0x08) ----

// Habilita la FIFO del BMI270 (header + accel + gyro). Llamar una vez,
// despues de que bmi270_sensor_init() haya devuelto ESP_OK.
esp_err_t bmi270_fifo_enable(void);

// Vuelca en 'out' (hasta 'max_out' elementos) las muestras acumuladas en la
// FIFO desde la ultima llamada, en orden cronologico, en LSB crudos.
// Devuelve el numero de muestras leidas (>=0), o -1 si hubo un error de
// comunicacion. Si el BMI270 tuvo que descartar muestras por desborde de
// su FIFO interna (el firmware tardo en venir a leerla), '*skipped_out' se
// pone al numero de muestras descartadas detectadas EN ESTA LLAMADA (no es
// acumulativo: cuenta las de esta lectura, no las de toda la sesion).
int bmi270_fifo_read(bmi270_raw_sample_t *out, int max_out, uint16_t *skipped_out);

#ifdef __cplusplus
}
#endif

#endif // BMI270_ESP32_H
