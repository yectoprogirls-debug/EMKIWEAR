#ifndef BLE_STREAM_H
#define BLE_STREAM_H

/*
 * ble_stream.h
 * ---------------------------------------------------------------------------
 * Servidor GATT (NimBLE) que publica los datos del reloj de Parkinson por
 * BLE hacia la app Android. Un solo servicio primario con 3 caracteristicas:
 *
 *   Servicio  UUID 7A9B0001-2D4E-4E6F-8A1B-0C2D3E4F5061
 *
 *   - SAMPLE  7A9B0002-...  NOTIFY + READ   (stream a ~25 Hz para graficar)
 *       struct empaquetado little-endian, 14 bytes:
 *         uint32 t_ms            tiempo desde arranque (ms)
 *         int16  roll_cdeg       roll  en grados * 100
 *         int16  pitch_cdeg      pitch en grados * 100
 *         int16  lin_acc_mg      aceleracion lineal |a| en g * 1000
 *         int16  gyro_mag_ddps   |giro| en grados/s * 10
 *         int16  tremor_amp_ddps amplitud en banda en grados/s * 10
 *
 *   - STATUS  7A9B0003-...  NOTIFY + READ   (resultado del detector, ~2 Hz)
 *       struct empaquetado little-endian, 14 bytes:
 *         uint32 t_ms
 *         uint8  flags            bit0    = temblor_presente (LEVE/MODERADO/GRAVE sostenido)
 *                                 bit1    = frecuencia_estable
 *                                 bit2    = temblor_detectado en esta ventana (sin histeresis)
 *                                 bits3-5 = CLASE: 0 SIN_TEMBLOR, 1 LEVE, 2 MODERADO,
 *                                           3 GRAVE, 4 ACCION VOLUNTARIA -> (flags>>3)&0x07
 *                                 bit6    = reservado (siempre 0)
 *         uint8  dom_freq_ddhz    frecuencia dominante en Hz * 10 (0 = sin señal)
 *         int16  tremor_amp_ddps  amplitud en banda en grados/s * 10
 *         int16  movement_ddps    movimiento voluntario RMS en grados/s * 10
 *         uint8  band_frac_pct    fraccion de energia en banda (0..100)
 *         uint8  duty_pct         % de tiempo con temblor en el ultimo minuto
 *         uint16 episode_s        duracion del episodio actual (s)
 *
 *   - RAW     7A9B0007-...  NOTIFY + READ   (IMU en crudo, ~50 Hz)
 *       No es para mostrar en la app: es para subir a Firebase / analisis
 *       offline. struct empaquetado little-endian, 16 bytes:
 *         uint32 t_ms
 *         int16  ax_mg    acelerometro eje X, g * 1000   (crudo, sin filtrar)
 *         int16  ay_mg    acelerometro eje Y, g * 1000
 *         int16  az_mg    acelerometro eje Z, g * 1000
 *         int16  gx_ddps  giroscopio eje X, grados/s * 10 (crudo, CON bias de fabrica)
 *         int16  gy_ddps  giroscopio eje Y, grados/s * 10
 *         int16  gz_ddps  giroscopio eje Z, grados/s * 10
 *
 *   - RAWFIFO 7A9B0008-...  NOTIFY (sin READ)  (lotes de la FIFO, LSB crudos)
 *       Para MATLAB: frecuencia fija (50 Hz), sin perdidas ni duplicados
 *       (viene de bmi270_fifo_read, no de bmi270_sensor_read). Solo transmite
 *       mientras haya un cliente suscrito a sus notificaciones.
 *       Cabecera de 7 bytes + n_muestras * 12 bytes, todo little-endian:
 *         uint16 seq_primera   numero de secuencia de la 1a muestra del paquete.
 *                               Cuenta MUESTRAS REALES del sensor: si la FIFO
 *                               se desbordo, este numero SALTA respecto al
 *                               ultimo paquete -> asi se ve cuantas se perdieron.
 *                               Da la vuelta (wraps) en 65535 -> 0.
 *         uint32 t0_us          esp_timer_get_time() (truncado a 32 bits) del
 *                               instante de la 1a muestra de ESTE paquete.
 *         uint8  n_muestras     numero de muestras en este paquete.
 *         n_muestras x {
 *           int16 ax, ay, az    acelerometro, LSB CRUDOS (sin escalar)
 *           int16 gx, gy, gz    giroscopio,   LSB CRUDOS (sin escalar)
 *         }
 *       Para convertir a unidades fisicas usar los factores que da INFO
 *       (comando 0x30 de CMD): accel[m/s^2] = lsb*LSB2MS2, gyro[dps] = lsb*LSB2DPS.
 *       Como el ODR es fijo, el instante de la muestra i-esima del paquete es
 *       t0_us + i * (1000000 / ODR).
 *       n_muestras se ajusta solo al MTU negociado: con MTU=247 (payload 244)
 *       caben 19 muestras por paquete (7 + 19*12 = 235 <= 244).
 *
 *   - CMD     7A9B0004-...  WRITE + NOTIFY + READ (NOTIFY/READ solo para INFO)
 *         1 byte:  0x00 = pausar stream SAMPLE
 *                  0x01 = reanudar stream SAMPLE
 *                  0x02 = pausar stream RAW (0x07)
 *                  0x03 = reanudar stream RAW (0x07)
 *                  0x30 = INFO: responde por NOTIFY (y queda disponible por
 *                         READ) con un texto ASCII "clave=valor;clave=valor;...":
 *                           FW=<version>;ODR=<Hz>;ACC_RANGE_G=<g>;
 *                           GYR_RANGE_DPS=<dps>;LSB2MS2=<factor>;LSB2DPS=<factor>
 *                         Recomendado negociar MTU >= 96 antes de pedir INFO
 *                         por NOTIFY (si no, se trunca); por READ no hace
 *                         falta, el cliente BLE fragmenta solo.
 *                  0xC0 = entrar en MODO CARGA (ver charge.h): el reloj avisa
 *                         con BATT = 0xFF, cierra la conexion, apaga todo y
 *                         duerme hasta que se mantenga el boton 2 s o pasen
 *                         CHARGE_SLEEP_HOURS horas; entonces vuelve a publicitar.
 *         5 bytes: 0x10 + uint32 epoch_le  = sincronizar hora UNIX (s)
 *                  0x20 + uint32 seq_le    = confirmar resumenes recibidos hasta 'seq'
 *
 *   - SUMMARY 7A9B0006-...  NOTIFY + READ   (resumenes horarios, store-and-forward)
 *       struct 'hs_record_t' (hourly_summary.h), 36 bytes little-endian.
 *       El firmware notifica cada resumen nuevo y, al (re)conectar, reenvia
 *       todos los que tengan seq > last_acked. La app DEBE confirmar con
 *       CMD 0x20 tras persistirlos (p.ej. en Firebase) o se reenviaran.
 *       READ devuelve el resumen pendiente mas antiguo (o longitud 0).
 *
 *   - EXT     7A9B0005-...  NOTIFY + READ   (rama acelerometro, ~2 Hz)
 *       struct empaquetado little-endian, 12 bytes. Aditivo: la app actual
 *       puede ignorarlo. Sirve para comparabilidad con acelerometria y para
 *       distinguir vibracion externa (coche/herramientas) de temblor articular.
 *         uint32 t_ms
 *         uint16 accel_amp_mg     RMS de la aceleracion lineal en banda (milli-g)
 *         uint16 accel_gyro_x100  relacion accel(mg)/giro(dps) * 100
 *         uint16 ang_disp_cdeg    amplitud angular pico estimada, grados * 100
 *         uint8  flags2           bit0 = rotational (coherente con rotacion articular)
 *         uint8  category         0 SIN_TEMBLOR, 1 LEVE, 2 MODERADO, 3 GRAVE,
 *                                 4 ACCION VOLUNTARIA
 *
 *   - SEV     7A9B000A-...  NOTIFY + READ   (severidad, ~2 Hz)
 *       struct empaquetado little-endian, 8 bytes. Aditivo (la app actual
 *       puede ignorarlo); se envia junto con STATUS. No se metio en EXT para
 *       no cambiarle el tamaño (otros clientes ya lo esperan en 12 bytes).
 *         uint32 t_ms
 *         uint16 theta_cdeg   angulo calibrado (theta_cal_deg), grados * 100
 *         uint16 desplaz_mm   desplazamiento lineal pico-a-pico estimado en
 *                             la punta de los dedos (desplaz_cm), milimetros
 *
 *   - BATT    7A9B0009-...  NOTIFY + READ   (bateria / aviso de modo carga)
 *       1 byte:  0xFF = "entrando en modo carga" (se notifica ~500 ms antes
 *                       de cerrar la conexion, en respuesta a CMD 0xC0)
 *                0xFE = nivel desconocido. El hardware no mide el voltaje de
 *                       la bateria: READ devuelve SIEMPRE este valor en
 *                       funcionamiento normal.
 *
 * El nombre en publicidad BLE es "ParkinsonWatch".
 * ---------------------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>
#include "tremor.h"
#include "bmi270_esp32.h"

#ifdef __cplusplus
extern "C" {
#endif

/* bits del campo 'flags' de STATUS */
#define BLE_STATUS_FLAG_TREMOR    0x01
#define BLE_STATUS_FLAG_FREQ_OK   0x02
#define BLE_STATUS_FLAG_DETECTED  0x04   /* tremor_detected: temblor en esta ventana, sin histeresis */
#define BLE_STATUS_CLASS_SHIFT    3       /* clase en bits 3-5 (3 bits, 0..4) */
#define BLE_STATUS_CLASS_MASK     0x38
#define BLE_STATUS_FLAG_ANALIZANDO 0x40  /* bit6: aun no hay resultado valido; la app
                                          * muestra "Analizando..." */

/* comando de CMD: entrar en modo carga */
#define BLE_CMD_CHARGE_MODE       0xC0

/* valores de BATT */
#define BLE_BATT_CHARGE_MODE      0xFF   /* entrando en modo carga */
#define BLE_BATT_UNKNOWN          0xFE   /* nivel desconocido */

/* bits del campo 'flags2' de EXT */
#define BLE_EXT_FLAG_ROTATIONAL  0x01

/* Arranca el stack BLE y empieza a publicitar. Llamar despues de
 * nvs_flash_init(). Se puede volver a llamar tras ble_stream_deinit(); si el
 * stack ya esta arriba no hace nada. */
void ble_stream_init(void);

/* Para el reenvio periodico de STATUS (lo reactiva ble_stream_init()). */
void ble_stream_stop_data(void);

/* Cierra la conexion de forma ordenada, deja de publicitar y apaga por
 * completo el host NimBLE y el controlador (radio apagada). Bloquea hasta
 * ~2 s. NO llamar desde un callback de NimBLE. Ninguna otra tarea debe estar
 * usando ble_stream_push_*() mientras tanto. */
void ble_stream_deinit(void);

/* Publica 1 byte en la caracteristica BATT (si hay cliente suscrito). */
void ble_stream_push_battery(uint8_t value);

/* true si hay un cliente (la app) conectado. */
bool ble_stream_connected(void);

/* true si el stream SAMPLE esta activo (la app no lo ha pausado con CMD 0x00).
 * El bucle principal puede usarlo para no calcular/enviar muestras cuando la
 * app pide "calma" -> ahorro de bateria y ancho de banda. */
bool ble_stream_streaming(void);

/* Publica una muestra en la caracteristica SAMPLE (si hay cliente suscrito
 * y el stream no esta pausado). Llamar a ~25 Hz. */
void ble_stream_push_sample(uint32_t t_ms,
                            float roll_deg, float pitch_deg,
                            float lin_acc_g, float gyro_mag_dps,
                            float tremor_amp_dps);

/* true si el stream RAW esta activo (la app no lo ha pausado con CMD 0x02). */
bool ble_stream_raw_streaming(void);

/* Publica una muestra CRUDA de la IMU (6 ejes) en la caracteristica RAW.
 * Valores tal cual salen del sensor (accel en g, giro en grados/s, sin
 * filtrar ni corregir bias). Llamar a la frecuencia de muestreo (~50 Hz). */
void ble_stream_push_raw(uint32_t t_ms,
                         float ax_g, float ay_g, float az_g,
                         float gx_dps, float gy_dps, float gz_dps);

/* Publica el resultado del detector en las caracteristicas STATUS, EXT y SEV.
 * Llamar cada vez que tremor_process() devuelve un resultado nuevo (~2 Hz). */
void ble_stream_push_status(uint32_t t_ms, const tremor_result_t *res);

/* Publica un lote de muestras crudas (LSB, de la FIFO) en la caracteristica
 * RAWFIFO (0x08), troceandolo en tantas notificaciones como haga falta segun
 * el MTU negociado. 'seq_primera' y 't0_us' corresponden a samples[0].
 * No hace nada (devuelve 0) si no hay cliente suscrito a 0x08.
 * Devuelve cuantos paquetes se enviaron realmente. */
int ble_stream_push_raw_batch(uint16_t seq_primera, uint32_t t0_us,
                              const bmi270_raw_sample_t *samples, int n);

#ifdef __cplusplus
}
#endif

#endif /* BLE_STREAM_H */
