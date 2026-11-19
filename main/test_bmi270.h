#ifndef TEST_BMI270_H
#define TEST_BMI270_H

#include <stdbool.h>

// Tarea de FreeRTOS que inicializa el BMI270 y va imprimiendo por UART
// las lecturas reales de accel (g) y gyro (dps) en formato CSV, para
// verificar que la comunicación I2C funciona.
void test_bmi270_task(void *pvParameters);

// ---- Modo carga (ver charge.h). Se llaman desde otra tarea. ----

// Detiene el muestreo: la tarea deja de leer la IMU, de analizar, de mandar
// por BLE y de dibujar. Lo que quedara pendiente se descarta. Espera a que la
// tarea este parada; devuelve false si no lo confirmo a tiempo (p.ej. el
// BMI270 no inicializo y la tarea no existe).
bool test_bmi270_pause(void);

// Con el muestreo ya detenido, pone el BMI270 en suspend. Devuelve false si
// no se pudo.
bool test_bmi270_sensor_sleep(void);

// Saca el BMI270 de suspend (si lo estaba) y reanuda el muestreo con el
// analizador reiniciado.
void test_bmi270_resume(void);

#endif // TEST_BMI270_H
