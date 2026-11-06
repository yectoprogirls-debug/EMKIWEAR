#ifndef TEST_BMI270_H
#define TEST_BMI270_H

// Tarea de FreeRTOS que inicializa el BMI270 y va imprimiendo por UART
// las lecturas reales de accel (g) y gyro (dps) en formato CSV, para
// verificar que la comunicación I2C funciona.
void test_bmi270_task(void *pvParameters);

#endif // TEST_BMI270_H
