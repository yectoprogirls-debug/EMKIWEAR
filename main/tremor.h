#ifndef TREMOR_H
#define TREMOR_H

/*
 * tremor.h
 * ---------------------------------------------------------------------------
 * Analizador de temblor parkinsoniano a partir de la IMU (giroscopio + acelerometro).
 *
 * AMPLITUD: la magnitud primaria es el RMS de la VELOCIDAD ANGULAR en banda
 * (grados/s, dps). Se elige el giroscopio porque el temblor de reposo es una
 * rotacion (supinacion-pronacion) y la velocidad angular NO depende de donde
 * se coloque el reloj en la muñeca (todo punto de un solido rigido gira igual);
 * la aceleracion lineal si depende del radio al eje (a ~ r*w^2). En paralelo se
 * calcula tambien el RMS de la aceleracion lineal en banda (milli-g) para
 * comparabilidad con la literatura de acelerometria y para discriminar
 * vibracion externa (coche, herramientas): esa da mucho acelerometro con poco
 * giro -> relacion accel/giro anomala -> se marca rotational=false.
 *
 * Escala tipica del temblor en dps (guia para umbrales):
 *     0-10 dps   ruido del sensor / micro-movimientos
 *    10-50 dps   temblor visible y ritmico
 *     >100 dps   normalmente movimiento voluntario (o temblor muy severo)
 *
 * Se le alimenta muestra a muestra a fs constante (el proyecto usa 50 Hz;
 * tremor_create() recibe la fs y adapta ventanas y filtros -- y avisa por
 * ESP_LOGE si no coincide con EMK_FS_HZ, ver emkiwear_filtros_coef.h). Las
 * muestras vienen de la FIFO del BMI270 (bmi270_fifo_read, ver
 * test_bmi270.c): equiespaciadas de verdad, sin duplicados ni huecos (los
 * huecos pequeños se interpolan antes de llegar aqui; los grandes disparan
 * tremor_reset()). Internamente:
 *   - giro: pasa-altos Butterworth orden 2 a 0.3 Hz (quita deriva/DC) seguido
 *     de pasa-banda Butterworth orden 3, 2.5-8 Hz (3 biquads en cascada,
 *     coeficientes de emkiwear_filtros_coef.h, diseñados y validados en
 *     MATLAB/SciPy con datos reales: 4 actividades cotidianas + 17 pruebas
 *     con robot a 3-7 Hz). Respuesta: -32 dB a 1 Hz, -20 dB a 1.5 Hz y a
 *     12 Hz, rizado ~0.6 dB entre 3 y 7 Hz.
 *   - acelerometro: el mismo pasa-banda 2.5-8 Hz orden 3 aplicado directo
 *     (sin el pasa-altos previo); ya quita la gravedad por si solo.
 *   - pasa-bajo 2 Hz en paralelo sobre el giro -> movimiento voluntario (sin cambios)
 *   - cada 'hop' (0.5 s), sobre una ventana de 5 s: amplitud RMS en banda,
 *     frecuencia dominante (Goertzel, barrido 3-12 Hz), concentracion
 *     espectral (energia cerca del pico / total, mas robusta que la pureza
 *     pico/total frente a actividades como escribir o cepillarse que meten
 *     energia de banda ancha en 3-9 Hz), estabilidad de frecuencia sostenida
 *     ~5 s, reposo (mov. voluntario absoluto Y relativo a la propia amplitud
 *     + rotacion neta), fraccion en banda y coherencia accel/giro.
 *   - decide "temblor de reposo" con histeresis (debounce) y lleva la
 *     duracion del episodio y el % de tiempo con temblor en el ultimo minuto.
 *
 * Los umbrales de decision estan en tremor.c y DEBEN ajustarse con
 * registros reales de pacientes; los valores por defecto son un punto
 * de partida razonable.
 * ---------------------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Clasificacion del estado actual (una por hop).
 *
 * LEVE/MODERADO/GRAVE solo existen si hay temblor parkinsoniano DETECTADO
 * (tremor_detected == true); se gradua por el angulo calibrado theta_cal_deg
 * (ver EMK_SEV_LEVE_MAX_DEG/EMK_SEV_MODERADO_MAX_DEG en
 * emkiwear_filtros_coef.h). Si no hay temblor, el estado es SIN_TEMBLOR
 * (reposo/ruido) o VOLUNTARIO (movimiento deliberado, sin patron de temblor).
 */
typedef enum {
    TREMOR_CLASS_SIN_TEMBLOR = 0,  /* 🟢 reposo o ruido: sin temblor detectado */
    TREMOR_CLASS_LEVE        = 1,  /* 🟡 temblor detectado, severidad leve
                                    *    (theta_cal_deg < EMK_SEV_LEVE_MAX_DEG) */
    TREMOR_CLASS_MODERADO    = 2,  /* 🟠 temblor detectado, severidad moderada */
    TREMOR_CLASS_GRAVE       = 3,  /* 🔴 temblor detectado, severidad alta
                                    *    (theta_cal_deg >= EMK_SEV_MODERADO_MAX_DEG) */
    TREMOR_CLASS_VOLUNTARIO  = 4,  /* 🔵 "Movimiento Detectado": velocidad
                                    *    > 100 dps sin patron de temblor ->
                                    *    accion deliberada (alzar el brazo, andar) */
} tremor_class_t;

/* Etiqueta legible de la clase (para logs / UI). */
const char *tremor_class_name(tremor_class_t c);

typedef struct {
    tremor_class_t category;     /* CLASIFICACION del estado actual (LEVE/MODERADO/GRAVE/VOLUNTARIO) */
    bool     tremor_present;     /* deteccion final de temblor PD (con histeresis, ~3 s sostenido) */
    float    tremor_amp_dps;     /* AMPLITUD PRIMARIA: RMS de la velocidad angular en banda (grados/s) */
    float    gyro_total_dps;     /* RMS de la velocidad angular total (toda la banda, AC) */
    float    dominant_freq_hz;   /* frecuencia dominante dentro de la banda (0 si no hay señal) */
    float    spectral_ratio;     /* CONCENTRACION espectral: potencia en +-0.5 Hz
                                  * del pico / potencia total del barrido (0..1).
                                  * Seno puro ~1.0, ruido de banda ancha ~0.2-0.35 */
    float    movement_dps;       /* RMS del movimiento voluntario (<2 Hz) */
    float    net_rotation_deg;   /* rotacion neta del miembro en la ventana */
    float    band_fraction;      /* energia en banda / energia total (0..1) */
    float    freq_spread_hz;     /* dispersion de la frecuencia dominante en las ultimas ventanas */
    bool     freq_stable;        /* true si la frecuencia se mantuvo estable */
    float    duty_last_min;      /* fraccion de tiempo con temblor en 60 s (0..1) */
    uint32_t episode_ms;         /* duracion del episodio actual (0 si no hay) */

    /* --- rama en paralelo del ACELEROMETRO (comparabilidad + discriminacion) --- */
    float    accel_amp_mg;       /* RMS de la aceleracion lineal en banda (milli-g) */
    float    accel_gyro_ratio;   /* accel_amp_mg / tremor_amp_dps  (mg por grado/s) */
    bool     rotational;         /* true si accel y giro son coherentes con una rotacion
                                  * articular; false -> vibracion externa/traslacion */
    float    ang_disp_deg;       /* amplitud angular estimada pico (grados) = derivada del
                                  * giro: tremor_amp_dps*sqrt2 / (2*pi*f) */

    /* --- Detector V2: caracteristicas por eje [0]=gx [1]=gy [2]=gz sobre
     * la ventana de 5 s, y resultado de la votacion (ver tremor.c). Se
     * calculan y reportan siempre, decidan o no (segun TREMOR_DETECTOR_V2). */
    float    sigma_f_hz[3];      /* dispersion espectral (Welch) en la banda 2.5-7.5 Hz */
    float    prel_pct[3];        /* potencia relativa en banda / potencia total del giro, % */
    float    rms_axis_dps[3];    /* RMS de la señal de giro en banda (2.5-8 Hz), por eje */
    float    fpeak_axis_hz[3];   /* frecuencia de pico (Welch) por eje */
    uint8_t  votes;              /* nº de ejes que votaron temblor */
    uint8_t  active_axes;        /* nº de ejes con señal por encima del piso de ruido */
    bool     gate_ok;            /* al menos un eje activo */
    bool     detect_v2;          /* veredicto COMBINADO del detector V2: (ruta1: v1 de 8
                                  * condiciones OR v2 por eje+votacion, con 'rotational'
                                  * ya incluido en ambas; OR ruta2: temblor de gran
                                  * amplitud en un solo eje via covarianza/PCA) Y
                                  * compuerta de energia voluntaria (vol110_pct por
                                  * debajo del umbral). Ver 'ruta' para saber cual
                                  * de las dos disparo */
    float    theta_cal_deg;      /* severidad calibrada (grados), eje dominante */
    bool     analizando;         /* true durante el calentamiento: aun no hay resultado
                                  * valido; la app NO debe mostrar "Sin temblor" */

    bool     tremor_detected;    /* true si hay temblor parkinsoniano detectado en este
                                  * hop (independiente de tremor_present, que exige
                                  * histeresis); solo entonces category puede ser
                                  * LEVE/MODERADO/GRAVE */
    float    vol110_pct;         /* compuerta de energia voluntaria (Parseval): 100 *
                                  * energia 1-2.5 Hz / energia 1-10 Hz del giro (suma de
                                  * los 3 ejes). Alto -> movimiento voluntario de baja
                                  * frecuencia, no temblor; se reporta siempre */

    /* --- RUTA2: temblor de gran amplitud en un solo eje (covarianza/PCA
     * de bp[0..2], ver tremor.c). Diagnostico; se reporta siempre. --- */
    uint8_t  ruta;                /* cual ruta disparo la deteccion: 0 ninguna,
                                  * 1 ruta1 (v1 OR v2 por votacion), 2 ruta2 (PCA),
                                  * 3 ruta3 (temblor leve humano); prioridad si
                                  * varias disparan: 2, luego 1, luego 3 */
    float    lin;                 /* autovalor mayor / traza de la covarianza 3x3
                                  * de bp[0..2] (0..1); cerca de 1 = giro casi en
                                  * una sola direccion (un eje domina) */
    float    prel3;               /* potencia relativa en banda, 3 ejes juntos, % */
    float    rms_pc;              /* RMS sobre el eje principal (autovector de
                                  * mayor varianza), dps */

    float    desplaz_cm;          /* desplazamiento lineal pico-a-pico estimado en la
                                  * punta de los dedos (cm; EMK_PALANCA_CM en
                                  * emkiwear_filtros_coef.h). Solo informativo: la
                                  * severidad LEVE/MODERADO/GRAVE se decide por
                                  * theta_cal_deg, no por este campo. Se reporta siempre */
} tremor_result_t;

typedef struct tremor_ctx tremor_ctx_t;

/* Crea el analizador (reserva memoria). Devuelve NULL si no hay RAM. */
tremor_ctx_t *tremor_create(float fs_hz);

/* Libera el analizador. */
void tremor_destroy(tremor_ctx_t *ctx);

/* Reinicia estado y filtros (mantiene la configuracion). */
void tremor_reset(tremor_ctx_t *ctx);

/* Alimenta una muestra de la IMU:
 *   gx/gy/gz  giroscopio en grados/s, con el bias ya restado
 *   ax/ay/az  acelerometro CRUDO en g (la gravedad la quita el pasa-alto interno)
 * Devuelve true cuando hay un resultado nuevo (una vez por hop, ~0.5 s);
 * 'result' solo es valido si devuelve true. */
bool tremor_process(tremor_ctx_t *ctx,
                    float gx_dps, float gy_dps, float gz_dps,
                    float ax_g, float ay_g, float az_g,
                    tremor_result_t *result);

#ifdef EMK_SELFTEST
/* Autoprueba de los filtros nuevos (pasa-altos 0.3 Hz + pasa-banda 2.5-8 Hz)
 * con senos de referencia a 1, 1.5, 5 y 12 Hz. Registra el RMS medido de
 * cada uno por ESP_LOGI, comparado contra los valores esperados en MATLAB.
 * Compilada solo si EMK_SELFTEST esta definido (ver tremor.c). */
void tremor_selftest_filters(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* TREMOR_H */
