/*
 * test_bmi270.c
 * ---------------------------------------------------------------------------
 * Cadena de procesamiento para el monitor de temblor parkinsoniano.
 *
 * FUENTE DE MUESTRAS: la FIFO del BMI270 (bmi270_fifo_read), leida cada
 * 20 ms. Es la via UNICA hacia el analizador de temblor (tremor_process) y
 * el resto del pipeline por muestra: a diferencia de bmi270_sensor_read()
 * (registro directo sondeado con el jitter del scheduler, ~8 ms sobre un
 * periodo de 20 ms), la FIFO entrega las muestras reales del sensor, en
 * orden, equiespaciadas, sin duplicarlas ni saltarlas. Si bmi270_fifo_enable()
 * falla al arrancar, se cae a bmi270_sensor_read() como respaldo (el
 * comportamiento que tenia el proyecto antes de tener FIFO).
 *
 *   FIFO (50 Hz, ODR fijo)
 *     -> por cada muestra del lote, en orden: LSB -> unidades fisicas
 *     -> process_sample():
 *          bias del giro -> filtro complementario (roll/pitch) ->
 *          aceleracion lineal -> CSV por UART -> tremor_process() ->
 *          BLE SAMPLE/STATUS, hs_feed(), display_show()
 *
 * Huecos por desborde de la FIFO (skipped_now > 0): si son pocas muestras
 * (<=5, <=0.1 s) se rellenan por interpolacion lineal entre la ultima
 * muestra procesada y la primera del lote nuevo, y esas muestras sinteticas
 * SI pasan por process_sample() (para no romper la continuidad de los
 * filtros/ventanas del analizador) pero NUNCA se mandan por BLE crudo
 * (0x07 ni 0x08): no son datos reales del sensor. Si el hueco es mayor,
 * se asume que la señal dejo de ser fiable y se reinicia el analizador
 * (tremor_reset) en vez de inventar 6+ muestras seguidas.
 *
 * El envio por BLE 0x08 (lotes con cabecera) y el log de la FIFO cada 5 s
 * usan exactamente las mismas muestras reales de siempre, sin cambios.
 *
 * Salida por UART:
 *   - CSV crudo decimado  (para diseñar/validar el filtrado offline)
 *   - lineas de log (W/I) cada 0.5 s con las features y la deteccion
 *   - linea de log cada 5 s con el estado de la FIFO/0x08 (leidas/paquetes/perdidas)
 * ---------------------------------------------------------------------------
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "bmi270_esp32.h"
#include "signal_proc.h"
#include "tremor.h"
#include "ble_stream.h"
#include "hourly_summary.h"
#include "display.h"
#include "test_bmi270.h"

static const char *TAG = "PARKINSON";

#define FS_HZ              50.0f
#define SAMPLE_PERIOD_MS   20                  /* 1000/FS_HZ; deben coincidir */
#define PERIOD_TICKS       pdMS_TO_TICKS(SAMPLE_PERIOD_MS)  /* FREERTOS_HZ=100 -> 2 ticks = 20 ms */
#define GYRO_BIAS_SAMPLES  150                 /* ~3 s con el reloj quieto (a 50 Hz) */
#define COMP_TAU_S         0.75f
#define G_TO_MS2           9.80665f
#define DEG2RAD            0.01745329251994f
#define BLE_SAMPLE_DECIM   2                   /* 50 Hz / 2 = 25 Hz por BLE */

/* Rangos configurados en bmi270_sensor_init(): ACC_RANGE_G=4, GYR_RANGE_DPS=500 */
#define LSB_TO_G(lsb)   ((float)(lsb) * 4.0f   / 32768.0f)
#define LSB_TO_DPS(lsb) ((float)(lsb) * 500.0f / 32768.0f)

#define FIFO_GAP_INTERP_MAX  5     /* huecos de hasta esta cantidad de muestras (<=0.1 s)
                                    * se interpolan; por encima, se reinicia el analizador */

/* Log CSV "V2,..." con las caracteristicas del detector V2 (tremor.c), para
 * comparar directamente contra MATLAB. Poner a 0 (antes de este include) si
 * se quiere apagar. */
#ifndef EMK_LOG_V2
#define EMK_LOG_V2 1
#endif

/* Estado del pipeline por muestra (antes eran variables sueltas del bucle;
 * ahora las usa process_sample()). */
typedef struct {
    comp_filter_t cf;
    gyro_bias_t   gb;
    tremor_ctx_t *tr;
    uint32_t      n;
    float         last_tremor_amp;
    bool primer_res;   /* ya hubo al menos un resultado del analizador */
    tremor_class_t shown_cat;   /* clase que se muestra (pantalla/app) */
    uint8_t        sin_run;     /* ventanas seguidas con SIN_TEMBLOR del detector */

    /* ultima muestra fisica procesada (antes del bias en el giro), para
     * poder interpolar un hueco pequeño de la FIFO en la siguiente llamada */
    bool     have_last;
    uint32_t last_t_ms;
    float    last_ax_g, last_ay_g, last_az_g;
    float    last_gx_dps, last_gy_dps, last_gz_dps;
} pipeline_state_t;

/* STATUS "analizando" (~2 Hz) durante el bias del giroscopio y el llenado de la
 * ventana de 5 s, para que la app no muestre "Sin temblor" en ese tiempo. */
static void push_status_analizando(uint32_t t_ms)
{
    static uint32_t cnt = 0;
    if ((cnt++ % 25u) != 0u) {          /* 25 muestras = 0.5 s */
        return;
    }
    tremor_result_t res;
    memset(&res, 0, sizeof(res));
    res.category   = TREMOR_CLASS_SIN_TEMBLOR;
    res.analizando = true;
    ble_stream_push_status(t_ms, &res);
}

#define SHOW_SIN_MIN_HOPS 3   /* SIN TEMBLOR debe durar 3 ventanas (1.5 s) para mostrarse */

/* Pipeline completo por muestra (una llamada = una muestra real o
 * interpolada). 'is_real_sample' controla si se manda por BLE crudo (0x07):
 * las interpoladas alimentan el analizador para no romper su continuidad,
 * pero no son datos reales del sensor y no deben salir como si lo fueran. */
static void process_sample(pipeline_state_t *st,
                           float ax_g, float ay_g, float az_g,
                           float gx_dps, float gy_dps, float gz_dps,
                           uint32_t t_ms, bool is_real_sample)
{
    /* -------- 0) BLE: IMU en crudo (~50 Hz) -> solo para subir a Firebase,
     *            no se muestra en la app. Solo muestras reales. -------- */
    if (is_real_sample && ble_stream_raw_streaming()) {
        ble_stream_push_raw(t_ms, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);
    }

    /* -------- 1) bias del giroscopio -------- */
    if (!st->gb.done) {
        push_status_analizando(t_ms);
        if (gyro_bias_feed(&st->gb, gx_dps, gy_dps, gz_dps)) {
            ESP_LOGI(TAG, "Bias gyro = [%.2f, %.2f, %.2f] dps. Analisis en marcha.",
                     st->gb.bx, st->gb.by, st->gb.bz);
        }
        return;   /* no procesamos hasta tener el bias */
    }

    float gx = gx_dps - st->gb.bx;
    float gy = gy_dps - st->gb.by;
    float gz = gz_dps - st->gb.bz;
    st->n++;

    /* -------- 2) orientacion (roll / pitch) -------- */
    comp_filter_update(&st->cf, ax_g, ay_g, az_g, gx, gy, gz);

    /* -------- 3) aceleracion lineal (sin gravedad) -------- */
    float rr = st->cf.roll_deg  * DEG2RAD;
    float pp = st->cf.pitch_deg * DEG2RAD;
    float grav_x = -sinf(pp);
    float grav_y =  sinf(rr) * cosf(pp);
    float grav_z =  cosf(rr) * cosf(pp);
    float lin_ax = (ax_g - grav_x) * G_TO_MS2;
    float lin_ay = (ay_g - grav_y) * G_TO_MS2;
    float lin_az = (az_g - grav_z) * G_TO_MS2;
    float lin_mag_g = sqrtf(lin_ax * lin_ax + lin_ay * lin_ay + lin_az * lin_az)
                      / G_TO_MS2;
    float gyro_mag  = sqrtf(gx * gx + gy * gy + gz * gz);

    /* -------- 4) CSV crudo (1 de cada 2 muestras = ~25 Hz por UART;
     *            el filtro/analisis recibe las 50 muestras/s completas).
     *            El tiempo sale del contador de muestras (equiespaciado
     *            exacto), no del reloj de pared. -------- */
    if ((st->n & 1u) == 0u) {
        printf("%.2f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f\n",
               st->n / FS_HZ, ax_g, ay_g, az_g, gx, gy, gz,
               st->cf.roll_deg, st->cf.pitch_deg);
    }

    /* -------- 5) analisis de temblor (giro + acelerometro crudo) -------- */
    tremor_result_t res;
    bool have_res = tremor_process(st->tr, gx, gy, gz, ax_g, ay_g, az_g, &res);
    if (have_res) {
        st->primer_res = true;
        st->last_tremor_amp = res.tremor_amp_dps;   /* se mantiene entre hops */
    }

    /* -------- 6) BLE: stream de muestras (~25 Hz) y estado (~2 Hz) --------
     * SAMPLE solo si la app no lo pauso (CMD 0x00); STATUS siempre, para
     * que la deteccion/frecuencia/amplitud no se congelen en la app. Estos
     * SI se calculan tambien para muestras interpoladas: son estado
     * procesado/estimado, no "crudo del sensor". */
    if ((st->n % BLE_SAMPLE_DECIM) == 0u && ble_stream_streaming()) {
        ble_stream_push_sample(t_ms, st->cf.roll_deg, st->cf.pitch_deg,
                               lin_mag_g, gyro_mag, st->last_tremor_amp);
    }
    if (have_res) {
        tremor_result_t res_show = res;          /* copia solo para mostrar */
        if (!res.analizando) {
            if (res.category == TREMOR_CLASS_SIN_TEMBLOR) {
                if (st->sin_run < 255) st->sin_run++;
                if (st->sin_run < SHOW_SIN_MIN_HOPS) {
                    res_show.category = st->shown_cat;   /* mantener el estado anterior */
                }
            } else {
                st->sin_run = 0;
            }
            st->shown_cat = res_show.category;
        }

        ble_stream_push_status(t_ms, &res_show);   /* STATUS + EXT + SEV */
        hs_feed(&res);                 /* agregador de resumenes horarios (clase REAL) */
        display_show(&res_show, ble_stream_connected());   /* cara del reloj (~2 Hz) */
    } else if (!st->primer_res) {
        /* aun no hubo NINGUN resultado (ventana llenandose): "Analizando..." */
        push_status_analizando(t_ms);
    }

    if (have_res) {
        /* linea de clasificacion (siempre) */
        const char *lvl = tremor_class_name(res.category);
        if (res.category == TREMOR_CLASS_LEVE ||
            res.category == TREMOR_CLASS_MODERADO ||
            res.category == TREMOR_CLASS_GRAVE) {
            ESP_LOGW(TAG,
                     "[%s] f=%.2fHz amp=%.1f dps (~%.2f deg) accel=%.0fmg episodio=%.1fs duty1min=%.0f%%",
                     lvl, res.dominant_freq_hz, res.tremor_amp_dps, res.ang_disp_deg,
                     res.accel_amp_mg, res.episode_ms / 1000.0f,
                     res.duty_last_min * 100.0f);
        } else {
            ESP_LOGI(TAG,
                     "[%s] amp=%.1f dps  giroTotal=%.1f dps  f=%.2fHz  mov=%.1f dps  accel=%.0fmg",
                     lvl, res.tremor_amp_dps, res.gyro_total_dps,
                     res.dominant_freq_hz, res.movement_dps, res.accel_amp_mg);
        }

#if EMK_LOG_V2
        /* CSV para comparar directamente contra MATLAB: t_s,votes,active_axes,
         * detect_v2,sigma gx/gy/gz,prel gx/gy/gz,rms gx/gy/gz,fpk gx/gy/gz,
         * theta_cal_deg,tremor_detected, y al final (diagnostico desde el
         * monitor serie): ruta,lin,prel3,rms_pc,vol110,movement_dps,
         * gyro_total_dps,categoria(num),desplaz_cm */
        printf("V2,%.2f,%u,%u,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,"
               "%u,%.3f,%.1f,%.1f,%.2f,%.1f,%.1f,%d,%.3f\n",
               st->n / FS_HZ, res.votes, res.active_axes, res.detect_v2 ? 1 : 0,
               res.sigma_f_hz[0], res.sigma_f_hz[1], res.sigma_f_hz[2],
               res.prel_pct[0], res.prel_pct[1], res.prel_pct[2],
               res.rms_axis_dps[0], res.rms_axis_dps[1], res.rms_axis_dps[2],
               res.fpeak_axis_hz[0], res.fpeak_axis_hz[1], res.fpeak_axis_hz[2],
               res.theta_cal_deg, res.tremor_detected ? 1 : 0,
               (unsigned)res.ruta, res.lin, res.prel3, res.rms_pc, res.vol110_pct,
               res.movement_dps, res.gyro_total_dps, (int)res.category, res.desplaz_cm);
#endif
    }

    /* recordar esta muestra (real o interpolada) para poder interpolar el
     * siguiente hueco pequeño que aparezca */
    st->last_t_ms   = t_ms;
    st->last_ax_g   = ax_g;
    st->last_ay_g   = ay_g;
    st->last_az_g   = az_g;
    st->last_gx_dps = gx_dps;
    st->last_gy_dps = gy_dps;
    st->last_gz_dps = gz_dps;
    st->have_last   = true;
}

void test_bmi270_task(void *pvParameters)
{
    (void)pvParameters;

#ifdef EMK_SELFTEST
    tremor_selftest_filters();
#endif

    if (bmi270_sensor_init() != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar el BMI270, abortando tarea.");
        vTaskDelete(NULL);
        return;
    }

    bool fifo_ok = (bmi270_fifo_enable() == ESP_OK);
    if (!fifo_ok) {
        ESP_LOGW(TAG, "FIFO no disponible: usando bmi270_sensor_read() como respaldo "
                      "(sin garantia de muestras equiespaciadas ni caracteristica 0x08).");
    }

    pipeline_state_t st;
    memset(&st, 0, sizeof(st));
    comp_filter_init(&st.cf, FS_HZ, COMP_TAU_S);
    gyro_bias_start(&st.gb, GYRO_BIAS_SAMPLES);
    st.tr = tremor_create(FS_HZ);
    if (!st.tr) {
        ESP_LOGE(TAG, "Sin memoria para el analizador de temblor, abortando.");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Calibrando bias del giroscopio: MANTEN EL RELOJ QUIETO ~3 s...");

    /* Cabecera CSV (datos crudos calibrados + orientacion) */
    printf("t_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg\n");

    bmi270_data_t d;                    /* solo se usa en el modo de respaldo */
    bmi270_raw_sample_t fifo_batch[64];
    TickType_t next = xTaskGetTickCount();

    /* ---- estado de la caracteristica 0x08 (FIFO -> lotes con cabecera) ---- */
    uint16_t fifo_next_seq = 0;               /* contador de muestras reales del sensor */
    uint32_t fifo_samples_total = 0, fifo_packets_total = 0, fifo_lost_total = 0;
    uint32_t fifo_read_errors = 0;
    uint32_t fifo_samples_prev = 0, fifo_packets_prev = 0, fifo_lost_prev = 0;
    uint32_t fifo_log_last_ms = 0;

    while (1) {
        vTaskDelayUntil(&next, PERIOD_TICKS);

        uint32_t t_read_ms = (uint32_t)(esp_timer_get_time() / 1000);

        if (fifo_ok) {
            /* -------- FIFO (0x08 + analizador): se drena SIEMPRE, cada
             * 20 ms, pase lo que pase en el resto del pipeline. -------- */
            uint32_t t_read_us = (uint32_t)esp_timer_get_time();
            uint16_t skipped_now = 0;
            int nf = bmi270_fifo_read(fifo_batch,
                                      (int)(sizeof(fifo_batch) / sizeof(fifo_batch[0])),
                                      &skipped_now);
            if (nf < 0) {
                fifo_read_errors++;
            } else {
                if (skipped_now > 0) {
                    fifo_lost_total += skipped_now;
                    /* el hueco SIEMPRE se ve en la numeracion de 0x08, se
                     * interpole o no internamente */
                    fifo_next_seq = (uint16_t)(fifo_next_seq + skipped_now);

                    if (skipped_now <= FIFO_GAP_INTERP_MAX && nf > 0 && st.have_last) {
                        /* hueco pequeño (<=0.1 s): interpolacion lineal entre
                         * la ultima muestra procesada y la primera del lote
                         * nuevo. Alimenta el analizador (is_real_sample=false)
                         * pero NUNCA sale por BLE crudo. */
                        float fax = LSB_TO_G(fifo_batch[0].ax);
                        float fay = LSB_TO_G(fifo_batch[0].ay);
                        float faz = LSB_TO_G(fifo_batch[0].az);
                        float fgx = LSB_TO_DPS(fifo_batch[0].gx);
                        float fgy = LSB_TO_DPS(fifo_batch[0].gy);
                        float fgz = LSB_TO_DPS(fifo_batch[0].gz);
                        uint32_t t_first_new_ms =
                            t_read_ms - (uint32_t)((nf - 1) * SAMPLE_PERIOD_MS);

                        for (uint16_t k = 1; k <= skipped_now; k++) {
                            float frac = (float)k / (float)(skipped_now + 1);
                            uint32_t t_interp_ms = st.last_t_ms + (uint32_t)
                                ((float)(t_first_new_ms - st.last_t_ms) * frac);
                            process_sample(&st,
                                st.last_ax_g   + (fax - st.last_ax_g)   * frac,
                                st.last_ay_g   + (fay - st.last_ay_g)   * frac,
                                st.last_az_g   + (faz - st.last_az_g)   * frac,
                                st.last_gx_dps + (fgx - st.last_gx_dps) * frac,
                                st.last_gy_dps + (fgy - st.last_gy_dps) * frac,
                                st.last_gz_dps + (fgz - st.last_gz_dps) * frac,
                                t_interp_ms, false);
                        }
                        ESP_LOGI(TAG, "Hueco de %u muestra(s) en la FIFO: interpolado.",
                                 (unsigned)skipped_now);
                    } else if (skipped_now > FIFO_GAP_INTERP_MAX) {
                        ESP_LOGW(TAG, "Hueco de %u muestras en la FIFO (>%d, >0.1s): "
                                      "reiniciando el analizador de temblor.",
                                 (unsigned)skipped_now, FIFO_GAP_INTERP_MAX);
                        tremor_reset(st.tr);
                        st.have_last = false;   /* ya no hay continuidad que interpolar */
                    }
                }

                if (nf > 0) {
                    /* --- 0x08: publicar el lote real (sin cambios) --- */
                    uint32_t period_us = 1000000u / (uint32_t)FS_HZ;
                    uint32_t t0_us = t_read_us - (uint32_t)(nf - 1) * period_us;
                    uint16_t seq_first = fifo_next_seq;
                    fifo_next_seq = (uint16_t)(fifo_next_seq + nf);
                    fifo_samples_total += (uint32_t)nf;
                    fifo_packets_total += (uint32_t)ble_stream_push_raw_batch(
                        seq_first, t0_us, fifo_batch, nf);

                    /* --- analizador: cada muestra real del lote, en orden --- */
                    for (int i = 0; i < nf; i++) {
                        float ax = LSB_TO_G(fifo_batch[i].ax);
                        float ay = LSB_TO_G(fifo_batch[i].ay);
                        float az = LSB_TO_G(fifo_batch[i].az);
                        float gx = LSB_TO_DPS(fifo_batch[i].gx);
                        float gy = LSB_TO_DPS(fifo_batch[i].gy);
                        float gz = LSB_TO_DPS(fifo_batch[i].gz);
                        uint32_t t_i_ms = t_read_ms -
                            (uint32_t)((nf - 1 - i) * SAMPLE_PERIOD_MS);
                        process_sample(&st, ax, ay, az, gx, gy, gz, t_i_ms, true);
                    }
                }
            }

            if (t_read_ms - fifo_log_last_ms >= 5000) {
                uint32_t dr = fifo_samples_total - fifo_samples_prev;
                uint32_t dp = fifo_packets_total - fifo_packets_prev;
                uint32_t dl = fifo_lost_total   - fifo_lost_prev;
                ESP_LOGI(TAG, "FIFO 0x08 (5s): leidas=%u paquetes=%u perdidas=%u "
                              "| total leidas=%u perdidas=%u errores_lectura=%u",
                         (unsigned)dr, (unsigned)dp, (unsigned)dl,
                         (unsigned)fifo_samples_total, (unsigned)fifo_lost_total,
                         (unsigned)fifo_read_errors);
                fifo_samples_prev = fifo_samples_total;
                fifo_packets_prev = fifo_packets_total;
                fifo_lost_prev = fifo_lost_total;
                fifo_log_last_ms = t_read_ms;
            }
        } else {
            /* -------- respaldo: bmi270_sensor_read(), comportamiento
             * anterior a la FIFO (sin garantia de equiespaciado ni 0x08) -------- */
            if (bmi270_sensor_read(&d)) {
                process_sample(&st, d.ax, d.ay, d.az, d.gx, d.gy, d.gz,
                               t_read_ms, true);
            }
        }
    }
}
