/*
 * tremor.c  -- ver tremor.h
 *
 * Detector de temblor de reposo parkinsoniano a partir de la IMU.
 * La deteccion NO se basa solo en "hay oscilacion de 3-12 Hz": eso lo
 * cumplen muchas actividades cotidianas (escribir y cepillarse ponen el
 * 50-90% de su energia en 3-9 Hz, segun grabaciones de referencia en MATLAB
 * de personas sin temblor: ningun pasa-banda por si solo las separa del
 * temblor real). Se exigen 8 condiciones simultaneas y sostenidas:
 *
 *   1) hay señal apreciable en banda (amplitud de giro por encima de un piso
 *      de ruido, AMP_TH_DPS; el techo MODERADO/GRAVE lo pone la clasificacion)
 *   2) frecuencia dentro de 3.0-12.0 Hz (temblor de reposo clasico 4-6 Hz,
 *      banda ampliada para cubrir tambien postural/de accion)
 *   3) CONCENTRACION espectral alta: potencia en +-0.5 Hz del pico / potencia
 *      total del barrido > CONC_TH -> señal tonal, no ruido de banda ancha
 *      (sustituye a la antigua pureza pico/total, insuficiente para distinguir
 *      un temblor real de la energia de banda ancha de escribir/cepillarse)
 *   4) frecuencia ESTABLE durante STAB_HOPS hops seguidos (~5 s)
 *   5) miembro EN REPOSO: poco movimiento voluntario absoluto (<REST_TH_DPS)
 *      y poca rotacion neta (no se reorienta el brazo -> no es "comer",
 *      "abrir puerta")
 *   6) la mayor parte de la energia de movimiento esta EN la banda del temblor
 *   7) accel y giro COHERENTES con una rotacion articular (relacion accel/giro
 *      moderada) -> descarta vibracion externa (coche, herramientas)
 *   8) compuerta voluntaria RELATIVA: el movimiento voluntario tambien debe
 *      ser pequeño EN RELACION a la propia amplitud del temblor
 *      (movement < VOL_TREM_RATIO_MAX * tremor_amp) -> filtra escribir y
 *      cepillarse, que meten movimiento de baja frecuencia comparable o
 *      mayor que su energia aparente en banda (el giroscopio, ademas, tiene
 *      menos energia falsa en banda que el acelerometro: 9 de 12 grabaciones
 *      de referencia, por eso la amplitud primaria sigue siendo el giro)
 *
 * ...y todo ello mantenido DEBOUNCE_HOPS hops (~3 s) para descartar gestos.
 *
 * FILTRADO (giro y acelerometro, coeficientes en emkiwear_filtros_coef.h,
 * diseñados y validados en MATLAB/SciPy con datos reales: 4 actividades
 * cotidianas + 17 pruebas con robot a 3-7 Hz): pasa-altos Butterworth orden 2
 * a 0.3 Hz (solo en el giro, quita deriva/DC) seguido de pasa-banda
 * Butterworth orden 3, 2.5-8 Hz (3 biquads en cascada; en el acelerometro se
 * aplica directo, sin el pasa-altos previo, porque ya quita la gravedad por
 * si solo). Respuesta del pasa-banda: -32 dB a 1 Hz, -20 dB a 1.5 Hz y a
 * 12 Hz, rizado ~0.6 dB entre 3 y 7 Hz. Ventana de analisis de 5 s (antes
 * 3 s), validada junto con este filtro. La logica de decision (banda de
 * busqueda de la frecuencia dominante 3.0-12.0 Hz, umbrales, debounce, etc.)
 * NO cambio con este filtro nuevo.
 *
 * Se alimenta EXCLUSIVAMENTE desde la FIFO del BMI270 (ver test_bmi270.c):
 * muestras reales, equiespaciadas, sin duplicados ni saltos por el jitter
 * (~8 ms sobre 20 ms) de la lectura por software.
 *
 * CLASIFICACION (campo 'category', una por hop): LEVE/MODERADO/GRAVE solo
 * existen si hay temblor parkinsoniano DETECTADO ('tremor_detected'). Con
 * TREMOR_DETECTOR_V2, 'tremor_detected = detect_v2 && rotational', donde
 * 'detect_v2 = (ruta1 OR ruta2) && (vol110_pct < EMK_UMBRAL_VOL110_PCT)':
 *   - ruta1: la deteccion "clasica" (v1, las 8 condiciones de arriba, OR
 *     el detector por eje+votacion, ambas con 'rotational' ya incluido).
 *   - ruta2: temblor de GRAN AMPLITUD en un SOLO eje de giro (covarianza 3x3
 *     de bp[0..2] + autovalor mayor por iteracion de potencia; ver tremor.c).
 *     Complementa a ruta1, que exige 2 ejes activos cuando hay mas de uno
 *     por encima del piso de ruido y puede fallar con un temblor muy fuerte
 *     pero unidireccional.
 *   - vol110_pct: compuerta de energia voluntaria (Parseval, ver
 *     emkiwear_filtros_coef.h): descarta movimiento voluntario rapido
 *     (agitar la mano, ~1-2 Hz) que de otro modo pasaria ruta1 o ruta2.
 * Con v1 puro (switch en 0), 'tremor_detected = rhythmic_pd' sin mas. Si hay
 * temblor, se gradua por el angulo calibrado theta_cal_deg (escala unica,
 * calibrada con el banco robotico y con temblor imitado por personas; ver
 * EMK_SEV_LEVE_MAX_DEG/EMK_SEV_MODERADO_MAX_DEG en emkiwear_filtros_coef.h):
 *   GRAVE        hay temblor y theta_cal_deg >= EMK_SEV_MODERADO_MAX_DEG
 *   MODERADO     hay temblor y theta_cal_deg >= EMK_SEV_LEVE_MAX_DEG
 *   LEVE         hay temblor y theta_cal_deg < EMK_SEV_LEVE_MAX_DEG
 * 'desplaz_cm' (desplazamiento lineal pico-a-pico estimado en la punta de
 * los dedos) se sigue calculando y reportando, pero solo como dato: no
 * decide la severidad. El temblor detectado se mira SIEMPRE PRIMERO:
 *   VOLUNTARIO   NO hay temblor y velocidad angular (movement o raw_ac) > 100 dps
 *   SIN_TEMBLOR  el resto (reposo, ruido del sensor, oscilacion insignificante)
 * 'tremor_present' = clase LEVE, MODERADO o GRAVE sostenida ~3 s (histeresis).
 *
 * Los umbrales estan pensados como punto de partida y deben re-ajustarse con
 * registros reales de pacientes y de actividades de la vida diaria.
 */

#include "tremor.h"
#include "signal_proc.h"
#include "emkiwear_filtros_coef.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "TREMOR";

#define TR_PI 3.14159265358979323846f

/* Switch para comparar el detector nuevo (validado en MATLAB: 48/48
 * ventanas de robot, 0/25 falsos positivos) contra el anterior (v1). Con
 * ==1 el v2 decide (rhythmic_pd/dominant_freq_hz/ang_disp_deg); con ==0
 * todo queda exactamente como antes. Las dos ramas se calculan siempre. */
#ifndef TREMOR_DETECTOR_V2
#define TREMOR_DETECTOR_V2 1
#endif

/* ---- Ventana ---------------------------------------------------------- */
#define WIN_SEC        5.0f      /* validada en MATLAB junto con el filtro nuevo */
#define HOP_SEC        0.5f

/* ---- Filtro del giro/accel: coeficientes de emkiwear_filtros_coef.h
 * (Butterworth pasa-altos orden 2 a 0.3 Hz + pasa-banda orden 3, 2.5-8 Hz,
 * diseñados y validados en MATLAB/SciPy). Ya no se disenan aqui con
 * biquad_design_*: se cargan tal cual con sos_init(). Solo validos para
 * fs = EMK_FS_HZ (se comprueba en tremor_create). */
#define VOL_LP_HZ      2.0f      /* <2 Hz = movimiento voluntario (sin cambios, 2º orden) */

#define F_SCAN_MIN     3.0f      /* barrido Goertzel: 3-12 Hz, paso 0.25 Hz -> 37 bins */
#define F_SCAN_MAX     12.0f
#define F_SCAN_STEP    0.25f
#define N_FSCAN        ((int)((F_SCAN_MAX - F_SCAN_MIN) / F_SCAN_STEP) + 1)
#define CONC_HALF_HZ   0.5f      /* ventana de concentracion alrededor del pico */

/* ---- Umbrales de decision (AJUSTAR CON DATOS REALES) ----------------
 * Escala del giroscopio (dps): 0-10 ruido, 10-50 temblor tipico, >100
 * normalmente voluntario o temblor muy severo. */
#define AMP_TH_DPS     10.0f     /* RMS minimo en banda -> frontera LEVE / MODERADO */
#define AMP_SEVERE_DPS 50.0f     /* RMS en banda -> frontera MODERADO / GRAVE */
#define VOLUNTARY_DPS  100.0f    /* velocidad angular total por encima de la cual,
                                  * si NO hay patron de temblor, es accion voluntaria */
#define CONC_TH        0.50f     /* concentracion espectral (pico +-0.5Hz / total).
                                  * Seno puro ~1.0, ruido de banda ancha ~0.2-0.35 */
#define REST_TH_DPS    20.0f     /* mov. voluntario absoluto por debajo -> "en reposo" */
#define VOL_TREM_RATIO_MAX 1.0f  /* compuerta relativa: mov. voluntario < este factor
                                  * x tremor_amp -> descarta escribir/cepillarse, que
                                  * meten movimiento de baja frecuencia comparable o
                                  * mayor que su propia energia en banda */
#define NET_ROT_TH_DEG 15.0f     /* rotacion neta del miembro en la ventana */
#define BAND_FRAC_TH   0.45f     /* energia en banda / energia total */
#define STAB_TOL_HZ    0.6f      /* dispersion maxima de la frecuencia */
#define STAB_HOPS      10        /* ventanas seguidas con frecuencia estable (~5 s) */
#define DEBOUNCE_HOPS  6         /* hops sostenidos para conmutar (~3 s) */
#define EMK_VOL_RATIO_MIN      0.75f   /* mov(<2 Hz) / RMS en banda: > esto = voluntario */
#define EMK_VOL_BANDFRAC_MAX   0.60f   /* fraccion de energia en banda: < esto = voluntario */
#define EMK_COLA_N        50      /* ultimo segundo de la ventana (50 muestras) */
#define EMK_COLA_FRAC     0.35f   /* RMS banda ultimo 1 s / RMS banda ventana: < esto = cola */
#define EMK_QUIETO_DPS    3.0f    /* giro total del ultimo 1 s: < esto = mano quieta */
#define EMK_HOLD_HOPS          3       /* ventanas (1.5 s) que se mantiene la clase de temblor */
#define EMK_R3_LIN_MIN          0.95f   /* giro casi en un solo eje */
#define EMK_R3_PREL_MIN_PCT     55.0f   /* energia en banda (3 ejes), % */
#define EMK_R3_VOL110_MAX_PCT   25.0f   /* poca energia voluntaria 1-2.5 Hz */
#define EMK_R3_AGR_MAX          3.5f    /* accel/giro (mg por dps): actividades >= 4.0 */
#define EMK_R3_RMS_MIN_DPS      15.0f   /* RMS minimo en el eje principal */
#define EMK_R3_MOV_MAX_DPS      60.0f   /* movimiento lento (<2 Hz) maximo */
#define WARMUP_HOPS    6         /* hops a ignorar tras llenar la ventana (~3 s) */

/* Coherencia acelerometro/giroscopio: una rotacion articular da una relacion
 * accel(mg)/giro(dps) moderada (~2-6, depende del radio al eje y de f). La
 * vibracion externa (coche, herramientas) o la traslacion del brazo dan mucho
 * acelerometro con poco giro -> relacion alta -> NO es temblor articular. */
#define VIB_RATIO_MAX  10.0f     /* mg por dps por encima -> vibracion externa */
#define VIB_MIN_MG     15.0f     /* por debajo de esta accel no se aplica el gate
                                  * (evita dividir por señal casi nula) */

#define DUTY_HOPS      120       /* 120 * 0.5 s = 60 s */

struct tremor_ctx {
    float fs;
    float dt;
    int   N;         /* muestras por ventana */
    int   hop;       /* muestras por hop */
    int   fill;
    int   head;
    int   count;

    /* giro: pasa-altos 0.3 Hz (hp03) -> pasa-banda 2.5-8 Hz orden 3 (bp_g).
     * acelerometro: pasa-banda 2.5-8 Hz orden 3 directo (bp_a), sin pasa-altos
     * previo (ya quita la gravedad por si solo). Coeficientes de
     * emkiwear_filtros_coef.h, por eje [0..2]. */
    sos_cascade_t hp03[3];
    sos_cascade_t bp_g[3];
    sos_cascade_t bp_a[3];
    biquad_t      vol_lp[3];    /* pasa-bajo del movimiento voluntario, por eje (sin cambios) */

    /* compuerta de energia voluntaria (Parseval, ver emkiwear_filtros_coef.h):
     * dos pasa-bandas mas sobre hp_val (giro tras el pasa-altos 0.3 Hz), por
     * eje. f_vol = 1-2.5 Hz, f_110 = 1-10 Hz; vol110_pct = 100*var(rv)/var(r110). */
    sos_cascade_t f_vol[3];
    sos_cascade_t f_110[3];

    float *raw[3];              /* ring: giro crudo (con bias restado) */
    float *hp[3];               /* ring: giro tras el pasa-altos de 0.3 Hz
                                 * (señal total, reservada para una Etapa 2
                                 * futura; no se usa aun en la decision) */
    float *bp[3];               /* ring: señal de giro en banda (2.5-8 Hz) */
    float *vol[3];              /* ring: señal voluntaria (<2 Hz) */
    float *abp[3];              /* ring: aceleracion lineal en banda (2.5-8 Hz) */
    float *rv[3];                /* ring: giro en banda 1-2.5 Hz (compuerta voluntaria) */
    float *r110[3];               /* ring: giro en banda 1-10 Hz (compuerta voluntaria) */
    float *tmp;                 /* buffer lineal (N) */
    float *hann;                /* ventana de Hann (N) */

    /* historial de frecuencia dominante para el test de estabilidad */
    float freq_hist[STAB_HOPS];
    int   freq_hist_n;
    int   freq_hist_idx;

    /* decision */
    int      warmup;
    int            hold_left;        /* ventanas restantes para mantener la clase de temblor */
    tremor_class_t last_tremor_cat;
    int      deb_true, deb_false;
    bool     present;
    uint32_t episode_ms;
    bool     duty_ring[DUTY_HOPS];
    int      duty_idx, duty_n;

    /* --- Detector V2: espectro tipo Welch por eje (ver welch_sigma_fpk) ---
     * welch_hann se precalcula una vez en tremor_create(); welch_seg es el
     * buffer de trabajo reutilizado cada hop (sin malloc por hop). */
    float welch_hann[EMK_WELCH_SEG];
    float welch_seg[EMK_WELCH_SEG];

    /* --- perfilado del calculo por hop (V2), reportado cada ~10 s --- */
    int64_t  perf_last_log_us;
    uint64_t perf_sum_us;
    uint32_t perf_max_us;
    uint32_t perf_count;
};

/* ------------------------------------------------------------------ */

const char *tremor_class_name(tremor_class_t c)
{
    switch (c) {
    case TREMOR_CLASS_SIN_TEMBLOR: return "SIN TEMBLOR (reposo/ruido)";
    case TREMOR_CLASS_LEVE:        return "LEVE (temblor parkinsoniano leve)";
    case TREMOR_CLASS_MODERADO:    return "MODERADO (temblor parkinsoniano)";
    case TREMOR_CLASS_GRAVE:       return "GRAVE (temblor severo)";
    case TREMOR_CLASS_VOLUNTARIO:  return "ACCION VOLUNTARIA";
    default:                       return "?";
    }
}

static float goertzel_power(const float *x, int n, float f_hz, float fs)
{
    float w     = 2.0f * TR_PI * (f_hz / fs);
    float coeff = 2.0f * cosf(w);
    float s1 = 0.0f, s2 = 0.0f, s0;

    for (int i = 0; i < n; i++) {
        s0 = x[i] + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    return s1 * s1 + s2 * s2 - coeff * s1 * s2;
}

/* Numero de bins del barrido del detector V2: 2.5-7.5 Hz, paso 0.125 Hz -> 41 */
#define EMK_SIGMA_N_BINS \
    ((int)((EMK_SIGMA_BANDA_HI_HZ - EMK_SIGMA_BANDA_LO_HZ) / EMK_SIGMA_PASO_HZ) + 1)

/* Espectro tipo Welch (4 segmentos de EMK_WELCH_SEG muestras, solape 50%,
 * ventana de Hann periodica, promediados) de 'x_win' (c->N muestras de un
 * eje del giro tras el pasa-altos de 0.3 Hz, en orden cronologico) sobre la
 * banda EMK_SIGMA_BANDA_LO_HZ..HI_HZ. Devuelve la frecuencia de pico, su
 * dispersion (sigma_f, ponderada por la potencia de cada bin) y la potencia
 * total de la banda (Pb). Usa c->welch_seg como buffer de trabajo (sin
 * memoria dinamica nueva). Si Pb es practicamente cero (sin señal), reporta
 * sigma_f=99 y fpk=0 en vez de dividir por casi nada. */
static void welch_sigma_fpk(tremor_ctx_t *c, const float *x_win,
                            float *out_sigma_f, float *out_fpk, float *out_pb)
{
    const int n_bins = EMK_SIGMA_N_BINS;
    float p[EMK_SIGMA_N_BINS];
    for (int k = 0; k < n_bins; k++) {
        p[k] = 0.0f;
    }

    for (int seg = 0; seg < 4; seg++) {
        int start = seg * EMK_WELCH_SALTO;

        double mean = 0.0;
        for (int i = 0; i < EMK_WELCH_SEG; i++) {
            mean += x_win[start + i];
        }
        mean /= EMK_WELCH_SEG;

        for (int i = 0; i < EMK_WELCH_SEG; i++) {
            c->welch_seg[i] = (float)((double)x_win[start + i] - mean) * c->welch_hann[i];
        }

        for (int k = 0; k < n_bins; k++) {
            float f_k = EMK_SIGMA_BANDA_LO_HZ + k * EMK_SIGMA_PASO_HZ;
            p[k] += goertzel_power(c->welch_seg, EMK_WELCH_SEG, f_k, c->fs);
        }
    }
    for (int k = 0; k < n_bins; k++) {
        p[k] *= 0.25f;   /* promedio de los 4 segmentos */
    }

    float pb = 0.0f;
    int   best_idx = 0;
    float best_p = p[0];
    for (int k = 0; k < n_bins; k++) {
        pb += p[k];
        if (p[k] > best_p) {
            best_p = p[k];
            best_idx = k;
        }
    }
    *out_pb = pb;

    if (pb < 1e-6f) {
        *out_sigma_f = 99.0f;
        *out_fpk = 0.0f;
        return;
    }

    float fc = 0.0f;
    for (int k = 0; k < n_bins; k++) {
        float f_k = EMK_SIGMA_BANDA_LO_HZ + k * EMK_SIGMA_PASO_HZ;
        fc += f_k * p[k];
    }
    fc /= pb;

    float var_f = 0.0f;
    for (int k = 0; k < n_bins; k++) {
        float f_k = EMK_SIGMA_BANDA_LO_HZ + k * EMK_SIGMA_PASO_HZ;
        float d = f_k - fc;
        var_f += d * d * p[k];
    }
    var_f /= pb;

    *out_sigma_f = sqrtf(var_f);
    *out_fpk     = EMK_SIGMA_BANDA_LO_HZ + best_idx * EMK_SIGMA_PASO_HZ;
}

/* copia el ring a dst[N] en orden temporal (mas antiguo -> mas nuevo) */
static void unwrap(const float *ring, int N, int head, int count, float *dst)
{
    int start = (head - count) % N;
    if (start < 0) {
        start += N;
    }
    for (int i = 0; i < count; i++) {
        dst[i] = ring[(start + i) % N];
    }
    for (int i = count; i < N; i++) {
        dst[i] = 0.0f;
    }
}

/* media cuadratica (mean square) de dst[0..N) */
static double mean_square(const float *x, int n)
{
    double s = 0.0;
    for (int i = 0; i < n; i++) {
        s += (double)x[i] * x[i];
    }
    return s / n;
}

/* varianza (mean square tras quitar la media) */
static double detrended_var(const float *x, int n)
{
    double m = 0.0;
    for (int i = 0; i < n; i++) {
        m += x[i];
    }
    m /= n;
    double s = 0.0;
    for (int i = 0; i < n; i++) {
        double d = x[i] - m;
        s += d * d;
    }
    return s / n;
}

/* ------------------------------------------------------------------ */

tremor_ctx_t *tremor_create(float fs_hz)
{
    /* fs_hz llega como parametro en tiempo de ejecucion, no como constante
     * de compilacion: el chequeo de coherencia con los coeficientes fijos
     * de emkiwear_filtros_coef.h no puede ser un _Static_assert. */
    if (fabsf(fs_hz - (float)EMK_FS_HZ) > 0.5f) {
        ESP_LOGE(TAG, "tremor_create(): fs=%.1f Hz no coincide con EMK_FS_HZ=%d Hz. "
                      "Los coeficientes de emkiwear_filtros_coef.h SOLO valen para %d Hz; "
                      "regeneralos con emkiwear_filtros.m para la fs actual.",
                 fs_hz, EMK_FS_HZ, EMK_FS_HZ);
    }

    tremor_ctx_t *c = calloc(1, sizeof(*c));
    if (!c) {
        return NULL;
    }

    c->fs  = fs_hz;
    c->dt  = 1.0f / fs_hz;
    c->N   = (int)(WIN_SEC * fs_hz + 0.5f);
    c->hop = (int)(HOP_SEC * fs_hz + 0.5f);

    bool ok = true;
    for (int a = 0; a < 3; a++) {
        /* filtros fijos (coeficientes de MATLAB/SciPy), no se diseñan aqui */
        sos_init(&c->hp03[a], EMK_HP_SOS, EMK_HP_N_BIQUADS);
        sos_init(&c->bp_g[a], EMK_BP_SOS, EMK_BP_N_BIQUADS);
        sos_init(&c->bp_a[a], EMK_BP_SOS, EMK_BP_N_BIQUADS);
        sos_init(&c->f_vol[a], EMK_VOL_SOS, EMK_VOL_N_BIQUADS);
        sos_init(&c->f_110[a], EMK_B110_SOS, EMK_B110_N_BIQUADS);
        biquad_design_lowpass(&c->vol_lp[a], VOL_LP_HZ, fs_hz, BUTTERWORTH_Q);

        c->raw[a]  = calloc(c->N, sizeof(float));
        c->hp[a]   = calloc(c->N, sizeof(float));
        c->bp[a]   = calloc(c->N, sizeof(float));
        c->vol[a]  = calloc(c->N, sizeof(float));
        c->abp[a]  = calloc(c->N, sizeof(float));
        c->rv[a]   = calloc(c->N, sizeof(float));
        c->r110[a] = calloc(c->N, sizeof(float));
        ok = ok && c->raw[a] && c->hp[a] && c->bp[a] && c->vol[a] && c->abp[a] &&
             c->rv[a] && c->r110[a];
    }
    c->tmp  = calloc(c->N, sizeof(float));
    c->hann = calloc(c->N, sizeof(float));
    ok = ok && c->tmp && c->hann;

    if (!ok) {
        tremor_destroy(c);
        return NULL;
    }

    for (int i = 0; i < c->N; i++) {
        c->hann[i] = 0.5f * (1.0f - cosf(2.0f * TR_PI * i / (c->N - 1)));
    }
    /* Hann PERIODICA de EMK_WELCH_SEG puntos (denominador = N, no N-1, a
     * diferencia de la de arriba) para el espectro Welch del detector V2.
     * Se precalcula una sola vez aqui, no en cada hop. */
    for (int i = 0; i < EMK_WELCH_SEG; i++) {
        c->welch_hann[i] = 0.5f * (1.0f - cosf(2.0f * TR_PI * i / EMK_WELCH_SEG));
    }
    tremor_reset(c);
    return c;
}

void tremor_destroy(tremor_ctx_t *c)
{
    if (!c) {
        return;
    }
    for (int a = 0; a < 3; a++) {
        free(c->raw[a]);
        free(c->hp[a]);
        free(c->bp[a]);
        free(c->vol[a]);
        free(c->abp[a]);
        free(c->rv[a]);
        free(c->r110[a]);
    }
    free(c->tmp);
    free(c->hann);
    free(c);
}

void tremor_reset(tremor_ctx_t *c)
{
    for (int a = 0; a < 3; a++) {
        sos_reset(&c->hp03[a]);
        sos_reset(&c->bp_g[a]);
        sos_reset(&c->bp_a[a]);
        sos_reset(&c->f_vol[a]);
        sos_reset(&c->f_110[a]);
        biquad_reset(&c->vol_lp[a]);
        memset(c->raw[a],  0, c->N * sizeof(float));
        memset(c->hp[a],   0, c->N * sizeof(float));
        memset(c->bp[a],   0, c->N * sizeof(float));
        memset(c->vol[a],  0, c->N * sizeof(float));
        memset(c->abp[a],  0, c->N * sizeof(float));
        memset(c->rv[a],   0, c->N * sizeof(float));
        memset(c->r110[a], 0, c->N * sizeof(float));
    }
    c->fill = c->head = c->count = 0;
    c->freq_hist_n = c->freq_hist_idx = 0;
    memset(c->freq_hist, 0, sizeof(c->freq_hist));
    c->warmup = WARMUP_HOPS;
    c->hold_left = 0;
    c->last_tremor_cat = TREMOR_CLASS_SIN_TEMBLOR;
    c->deb_true = c->deb_false = 0;
    c->present = false;
    c->episode_ms = 0;
    c->duty_idx = c->duty_n = 0;
    memset(c->duty_ring, 0, sizeof(c->duty_ring));
}

/* ------------------------------------------------------------------ */

bool tremor_process(tremor_ctx_t *c,
                    float gx_dps, float gy_dps, float gz_dps,
                    float ax_g, float ay_g, float az_g,
                    tremor_result_t *r)
{
    const float in[3]  = { gx_dps, gy_dps, gz_dps };
    const float acc[3] = { ax_g,   ay_g,   az_g   };

    /* --- filtrado por muestra: giro = pasa-altos 0.3 Hz -> pasa-banda
     * 2.5-8 Hz orden 3; accel = pasa-banda 2.5-8 Hz orden 3 directo --- */
    for (int a = 0; a < 3; a++) {
        float hp_val = sos_process(&c->hp03[a], in[a]);
        float bp     = sos_process(&c->bp_g[a], hp_val);

        float vl = biquad_process(&c->vol_lp[a], in[a]);

        /* el pasa-banda del acelerometro ya quita la gravedad (DC) sin
         * necesidad de la orientacion */
        float ab = sos_process(&c->bp_a[a], acc[a]);

        /* compuerta de energia voluntaria (Parseval): dos pasa-bandas mas
         * sobre la MISMA señal que alimenta a bp_g (hp_val) */
        float rv_val   = sos_process(&c->f_vol[a], hp_val);
        float r110_val = sos_process(&c->f_110[a], hp_val);

        c->raw[a][c->head]  = in[a];
        c->hp[a][c->head]   = hp_val;
        c->bp[a][c->head]   = bp;
        c->vol[a][c->head]  = vl;
        c->abp[a][c->head]  = ab;
        c->rv[a][c->head]   = rv_val;
        c->r110[a][c->head] = r110_val;
    }
    c->head = (c->head + 1) % c->N;
    if (c->count < c->N) {
        c->count++;
    }

    if (++c->fill < c->hop) {
        return false;
    }
    c->fill = 0;
    if (c->count < c->N) {
        return false;               /* aun sin ventana completa */
    }

    int64_t t_hop_start_us = esp_timer_get_time();   /* perfilado del calculo (V2) */

    /* --- energias por eje --- */
    float bp_pow[3], raw_var[3];
    float net_rot_axis[3];
    for (int a = 0; a < 3; a++) {
        unwrap(c->bp[a], c->N, c->head, c->count, c->tmp);
        bp_pow[a] = (float)mean_square(c->tmp, c->N);

        unwrap(c->raw[a], c->N, c->head, c->count, c->tmp);
        raw_var[a] = (float)detrended_var(c->tmp, c->N);

        /* rotacion neta = integral del giro voluntario (<2 Hz) en la ventana */
        unwrap(c->vol[a], c->N, c->head, c->count, c->tmp);
        double s = 0.0;
        for (int i = 0; i < c->N; i++) {
            s += c->tmp[i];
        }
        net_rot_axis[a] = (float)(s * c->dt);
    }

    float tremor_amp = sqrtf(bp_pow[0] + bp_pow[1] + bp_pow[2]);       /* dps RMS en banda */
    float raw_ac     = sqrtf(raw_var[0] + raw_var[1] + raw_var[2]);    /* dps RMS total (AC) */
    float net_rot    = sqrtf(net_rot_axis[0] * net_rot_axis[0] +
                             net_rot_axis[1] * net_rot_axis[1] +
                             net_rot_axis[2] * net_rot_axis[2]);       /* grados */
    float band_frac  = (raw_ac > 1e-3f) ? ((tremor_amp * tremor_amp) /
                                           (raw_ac * raw_ac)) : 0.0f;

    /* movimiento voluntario RMS (<2 Hz) */
    float vol_var_sum = 0.0f;
    for (int a = 0; a < 3; a++) {
        unwrap(c->vol[a], c->N, c->head, c->count, c->tmp);
        vol_var_sum += (float)detrended_var(c->tmp, c->N);
    }
    float movement = sqrtf(vol_var_sum);

    /* --- rama del acelerometro: RMS en banda (milli-g) --- */
    float abp_pow_sum = 0.0f;
    for (int a = 0; a < 3; a++) {
        unwrap(c->abp[a], c->N, c->head, c->count, c->tmp);
        abp_pow_sum += (float)mean_square(c->tmp, c->N);
    }
    float accel_amp_mg = sqrtf(abp_pow_sum) * 1000.0f;   /* g -> milli-g */
    float accel_gyro_ratio = accel_amp_mg / (tremor_amp > 1.0f ? tremor_amp : 1.0f);

    /* coherente con una rotacion articular si la accel es despreciable o la
     * relacion accel/giro es moderada; si no, es vibracion externa/traslacion */
    bool rotational = (accel_amp_mg < VIB_MIN_MG) ||
                      (accel_gyro_ratio <= VIB_RATIO_MAX);

    /* --- frecuencia dominante (Goertzel + Hann) sobre el eje con mas temblor --- */
    int dom = 0;
    if (bp_pow[1] > bp_pow[dom]) dom = 1;
    if (bp_pow[2] > bp_pow[dom]) dom = 2;

    unwrap(c->bp[dom], c->N, c->head, c->count, c->tmp);
    for (int i = 0; i < c->N; i++) {
        c->tmp[i] *= c->hann[i];
    }
    /* potencia de cada bin del barrido (sin memoria dinamica: N_FSCAN=37
     * fijo, va en la pila) -> hace falta guardada para la concentracion */
    float bin_p[N_FSCAN];
    float best_p = 0.0f, sum_p = 0.0f, best_f = 0.0f;
    int   best_idx = 0;
    for (int k = 0; k < N_FSCAN; k++) {
        float f = F_SCAN_MIN + k * F_SCAN_STEP;
        float p = goertzel_power(c->tmp, c->N, f, c->fs);
        bin_p[k] = p;
        sum_p += p;
        if (p > best_p) {
            best_p = p;
            best_f = f;
            best_idx = k;
        }
    }
    /* concentracion espectral = potencia de los bins a +-CONC_HALF_HZ del
     * pico / potencia total del barrido. Un seno puro concentra casi toda
     * la energia ahi (~1.0); ruido de banda ancha la reparte (~0.2-0.35). */
    int neigh = (int)(CONC_HALF_HZ / F_SCAN_STEP + 0.5f);
    float conc_p = 0.0f;
    for (int k = best_idx - neigh; k <= best_idx + neigh; k++) {
        if (k >= 0 && k < N_FSCAN) {
            conc_p += bin_p[k];
        }
    }
    float ratio = (sum_p > 0.0f) ? (conc_p / sum_p) : 0.0f;

    /* sin señal apreciable -> frecuencia/ritmicidad no significan nada */
    bool has_signal = (tremor_amp >= (0.25f * AMP_TH_DPS));
    if (!has_signal) {
        best_f = 0.0f;
        ratio  = 0.0f;
    }

    /* --- test de estabilidad de frecuencia --- */
    c->freq_hist[c->freq_hist_idx] = best_f;
    c->freq_hist_idx = (c->freq_hist_idx + 1) % STAB_HOPS;
    if (c->freq_hist_n < STAB_HOPS) {
        c->freq_hist_n++;
    }
    bool freq_stable = false;
    float freq_spread = 99.0f;
    if (c->freq_hist_n >= STAB_HOPS) {
        float fmin = c->freq_hist[0], fmax = c->freq_hist[0];
        bool all_in_band = true;
        for (int i = 0; i < STAB_HOPS; i++) {
            float f = c->freq_hist[i];
            if (f < fmin) fmin = f;
            if (f > fmax) fmax = f;
            if (f < F_SCAN_MIN || f > F_SCAN_MAX) {
                all_in_band = false;
            }
        }
        freq_spread = fmax - fmin;
        freq_stable = all_in_band && (freq_spread <= STAB_TOL_HZ);
    }

    /* amplitud angular pico estimada (grados): para un temblor casi sinusoidal
     * theta0 = (RMS * sqrt2) / (2*pi*f) */
    float ang_disp = (best_f > 0.0f)
                     ? (tremor_amp * 1.41421356f) / (2.0f * TR_PI * best_f)
                     : 0.0f;

    /* --- ¿la oscilacion parece un temblor parkinsoniano en reposo? ---
     * (mismos criterios de especificidad que antes, salvo la amplitud, que
     *  ahora sirve para separar MODERADO de GRAVE) */
    bool rhythmic_pd = has_signal &&
                       (best_f >= F_SCAN_MIN && best_f <= F_SCAN_MAX) &&
                       (ratio > CONC_TH) &&
                       freq_stable &&
                       (band_frac > BAND_FRAC_TH) &&
                       (net_rot < NET_ROT_TH_DEG) &&
                       (movement < REST_TH_DPS) &&
                       (movement < VOL_TREM_RATIO_MAX * tremor_amp) &&
                       rotational;

    /* ================================================================
     * DETECTOR V2: caracteristicas POR EJE del giroscopio sobre la ventana
     * completa (5 s) y decision por votacion. Validado con datos reales:
     * 48/48 ventanas de robot detectadas, 0/25 falsos positivos en
     * actividades cotidianas. Se calcula SIEMPRE (para poder comparar y
     * reportar), decida o no segun TREMOR_DETECTOR_V2.
     * ================================================================ */
    float rms_axis[3], var_hp[3], prel_axis[3], sigma_f_axis[3], fpk_axis[3];
    bool  activo_axis[3], voto_axis[3];

    for (int a = 0; a < 3; a++) {
        rms_axis[a] = sqrtf(bp_pow[a]);           /* 2a: RMS de la señal en banda */

        unwrap(c->hp[a], c->N, c->head, c->count, c->tmp);
        var_hp[a] = (float)detrended_var(c->tmp, c->N);         /* 2b */
        prel_axis[a] = 100.0f * bp_pow[a] / (var_hp[a] + 1e-6f); /* 2c: % */

        float pb_unused;
        welch_sigma_fpk(c, c->tmp, &sigma_f_axis[a], &fpk_axis[a], &pb_unused);  /* 2d */

        activo_axis[a] = rms_axis[a] > EMK_UMBRAL_RMS_MIN_DPS;  /* 2e */
        voto_axis[a]   = activo_axis[a] &&
                        (sigma_f_axis[a] < EMK_UMBRAL_SIGMA_HZ) &&
                        (prel_axis[a] > EMK_UMBRAL_PREL_PCT);
    }

    int n_activos = 0, votos = 0;
    for (int a = 0; a < 3; a++) {
        if (activo_axis[a]) n_activos++;
        if (voto_axis[a])   votos++;
    }
    /* Solo votan los ejes con señal: asi un temblor en un solo eje del
     * reloj tambien se detecta, y el ruido de los ejes quietos no bloquea
     * la votacion. */
    int  votos_necesarios = (n_activos >= 2) ? EMK_VOTOS_MIN : 1;
    bool gate_ok   = (n_activos >= 1);
    bool detect_v2 = gate_ok && (votos >= votos_necesarios);

    /* severidad: eje dominante = el de mayor rms_axis. Coincide con 'dom'
     * (elegido arriba por mayor bp_pow): sqrt es monotona, mismo eje. */
    float theta_cal_deg = (fpk_axis[dom] > 0.0f)
                          ? (EMK_K_CAL * 1.41421356f * rms_axis[dom] /
                             (2.0f * TR_PI * fpk_axis[dom]))
                          : 0.0f;

    /* --- RUTA2: temblor de gran amplitud concentrado en UN SOLO eje ---
     * covarianza 3x3 (double) de bp[0..2] (giro en banda, en orden
     * cronologico), restando la media; autovalor mayor por iteracion de
     * potencia (vector inicial = eje dominante 'dom', 30 iteraciones). Si el
     * giro oscila casi en una sola direccion, el mayor autovalor domina la
     * traza (lin cerca de 1): eso es un temblor de un solo eje, no ruido ni
     * vibracion repartida. Complementa a la votacion por eje (de arriba), que
     * exige 2 ejes activos cuando hay mas de uno por encima del piso de
     * ruido y puede fallar con un temblor muy fuerte pero unidireccional. */
    int start_pca = (c->head - c->count) % c->N;
    if (start_pca < 0) {
        start_pca += c->N;
    }
    double mean_bp[3] = { 0.0, 0.0, 0.0 };
    for (int i = 0; i < c->N; i++) {
        int idx = (start_pca + i) % c->N;
        for (int a = 0; a < 3; a++) {
            mean_bp[a] += c->bp[a][idx];
        }
    }
    for (int a = 0; a < 3; a++) {
        mean_bp[a] /= c->N;
    }
    double cov[3][3] = { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };
    for (int i = 0; i < c->N; i++) {
        int idx = (start_pca + i) % c->N;
        double d[3];
        for (int a = 0; a < 3; a++) {
            d[a] = (double)c->bp[a][idx] - mean_bp[a];
        }
        for (int p = 0; p < 3; p++) {
            for (int q = 0; q < 3; q++) {
                cov[p][q] += d[p] * d[q];
            }
        }
    }
    for (int p = 0; p < 3; p++) {
        for (int q = 0; q < 3; q++) {
            cov[p][q] /= c->N;
        }
    }

    double vpw[3] = { 0.0, 0.0, 0.0 };
    vpw[dom] = 1.0;
    double lam = 0.0;
    for (int iter = 0; iter < 30; iter++) {
        double w[3];
        for (int p = 0; p < 3; p++) {
            w[p] = cov[p][0] * vpw[0] + cov[p][1] * vpw[1] + cov[p][2] * vpw[2];
        }
        double norm = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
        if (norm < 1e-12) {
            lam = 0.0;
            break;
        }
        vpw[0] = w[0] / norm;
        vpw[1] = w[1] / norm;
        vpw[2] = w[2] / norm;
        lam = norm;
    }
    double traza = cov[0][0] + cov[1][1] + cov[2][2];
    float  lin    = (traza > 1e-9) ? (float)(lam / traza) : 0.0f;
    float  rms_pc = (float)sqrt(lam);
    float  prel3  = 100.0f * (bp_pow[0] + bp_pow[1] + bp_pow[2]) /
                    (var_hp[0] + var_hp[1] + var_hp[2] + 1e-6f);

    bool ruta2_pca = (lin >= EMK_R2_LIN_MIN) &&
                     (prel3 >= EMK_R2_PREL_MIN_PCT) &&
                     (rms_pc >= EMK_R2_RMS_MIN_DPS);

    /* --- compuerta de energia voluntaria (Parseval) ---
     * la varianza de una señal ya filtrada en una sub-banda ES la energia de
     * esa sub-banda (sin FFT). Un temblor parkinsoniano (3-8 Hz) casi no
     * tiene energia por debajo de 2.5 Hz; un movimiento voluntario rapido
     * (agitar la mano, gesticular, ~1-2 Hz) si, y eso se ve como una fraccion
     * alta de la energia de 1-10 Hz concentrada en 1-2.5 Hz. Se calcula y se
     * reporta siempre (r->vol110_pct), decida o no segun TREMOR_DETECTOR_V2. */
    float ev = 0.0f, e110 = 0.0f;
    for (int a = 0; a < 3; a++) {
        unwrap(c->rv[a], c->N, c->head, c->count, c->tmp);
        ev += (float)detrended_var(c->tmp, c->N);

        unwrap(c->r110[a], c->N, c->head, c->count, c->tmp);
        e110 += (float)detrended_var(c->tmp, c->N);
    }
    float vol110_pct = (e110 > 1e-9f) ? (100.0f * ev / e110) : 100.0f;

    /* veredicto combinado del detector V2: ruta1 (la deteccion que ya
     * existia: v1 de 8 condiciones OR v2 por eje+votacion, con la coherencia
     * accel/giro de 'rotational' ya incluida en ambas) OR ruta2 (temblor de
     * gran amplitud en un solo eje, arriba), y ademas la compuerta de
     * energia voluntaria. 'ruta' es solo diagnostico (log/CSV). */
    bool ruta1 = rhythmic_pd || (detect_v2 && rotational);
    bool ruta2 = ruta2_pca;
    /* RUTA3: temblor leve humano (giro pequeño e irregular). Su espectro es ancho
     * (sigma_f > 0.40 Hz), no vota en la ruta 1, y su amplitud no alcanza la
     * ruta 2. Se reconoce porque es un giro casi en un solo eje, con la energía en
     * la banda, sin componente voluntaria y con poca aceleración lineal por cada
     * dps de giro (las actividades cotidianas tienen accel/giro >= 4.0 mg/dps). */
    bool ruta3 = (lin >= EMK_R3_LIN_MIN) &&
                 (prel3 >= EMK_R3_PREL_MIN_PCT) &&
                 (vol110_pct < EMK_R3_VOL110_MAX_PCT) &&
                 (accel_gyro_ratio <= EMK_R3_AGR_MAX) &&
                 (rms_pc >= EMK_R3_RMS_MIN_DPS) &&
                 (movement < EMK_R3_MOV_MAX_DPS);
    detect_v2 = (ruta1 || ruta2 || ruta3) && (vol110_pct < EMK_UMBRAL_VOL110_PCT);
    uint8_t ruta_num = ruta2 ? 2 : (ruta1 ? 1 : (ruta3 ? 3 : 0));

    /* el temblor detectado = veredicto combinado Y coherencia accel/giro. */
#if TREMOR_DETECTOR_V2
    bool tremor_detected = detect_v2 && rotational;
#else
    bool tremor_detected = rhythmic_pd;   /* v1 puro: sin cambios */
#endif

    /* desplazamiento lineal pico-a-pico estimado en la punta de los dedos
     * (cm): arco = palanca[cm] * theta[rad] (una amplitud, de pico a cero);
     * *2 para pico-a-pico. SOLO INFORMATIVO (p.ej. para la caracteristica
     * SEV por BLE): la severidad se decide por theta_cal_deg directo, no
     * por este valor. */
    float desplaz_cm = 2.0f * EMK_PALANCA_CM * theta_cal_deg * (TR_PI / 180.0f);

    /* --- clasificacion del estado actual ---
     * El temblor detectado va PRIMERO: si tremor_detected es true, se
     * gradua por theta_cal_deg contra la escala unica (EMK_SEV_LEVE_MAX_DEG/
     * EMK_SEV_MODERADO_MAX_DEG, calibrada con el banco robotico y con
     * temblor imitado por personas). Solo si NO hay temblor se mira
     * 'movement'/'raw_ac' para VOLUNTARIO. OJO: el pasa-bajos de 2 Hz
     * (vol_lp) deja pasar una fraccion nada despreciable de un temblor muy
     * fuerte (cientos de °/s, por el roll-off finito de un filtro de 2º
     * orden) -> 'movement' puede superar VOLUNTARY_DPS por esa fuga, NO
     * porque sea movimiento voluntario real. Por eso NUNCA puede decidir
     * antes que el detector (si no, las imitaciones de temblor fuerte
     * salen como VOLUNTARIO). */
    /* voluntario REAL: mucho movimiento que NO viene de la banda del temblor */
    /* --- la ventana termina en reposo: lo que queda en ella es "cola" --- */
    double q2 = 0.0;
    for (int a = 0; a < 3; a++) {
        unwrap(c->hp[a], c->N, c->head, c->count, c->tmp);
        for (int i = c->N - EMK_COLA_N; i < c->N; i++) {
            q2 += (double)c->tmp[i] * (double)c->tmp[i];
        }
    }
    bool mano_quieta = ((float)sqrt(q2 / EMK_COLA_N) < EMK_QUIETO_DPS);

    bool cola = false;
    if (tremor_detected) {
        unwrap(c->bp[dom], c->N, c->head, c->count, c->tmp);
        double s2 = 0.0;
        for (int i = c->N - EMK_COLA_N; i < c->N; i++) {
            s2 += (double)c->tmp[i] * (double)c->tmp[i];
        }
        float rms_rec = (float)sqrt(s2 / EMK_COLA_N);
        if (mano_quieta || rms_rec < EMK_COLA_FRAC * rms_axis[dom]) {
            tremor_detected = false;
            cola = true;
        }
    }
    bool reposo_ya = cola || mano_quieta;

    bool mov_grande = (movement > VOLUNTARY_DPS || raw_ac > VOLUNTARY_DPS);
    bool vol_real   = mov_grande &&
                      ((movement > EMK_VOL_RATIO_MIN * tremor_amp) ||
                       (band_frac < EMK_VOL_BANDFRAC_MAX));
    tremor_class_t category;
    if (tremor_detected) {
        if (theta_cal_deg < EMK_SEV_LEVE_MAX_DEG) {
            category = TREMOR_CLASS_LEVE;
        } else if (theta_cal_deg < EMK_SEV_MODERADO_MAX_DEG) {
            category = TREMOR_CLASS_MODERADO;
        } else {
            category = TREMOR_CLASS_GRAVE;
        }
    } else if (reposo_ya) {
        category = TREMOR_CLASS_SIN_TEMBLOR;   /* la mano ya esta quieta */
    } else if (c->present && !vol_real && c->hold_left > 0 &&
               (c->last_tremor_cat == TREMOR_CLASS_LEVE ||
                c->last_tremor_cat == TREMOR_CLASS_MODERADO ||
                c->last_tremor_cat == TREMOR_CLASS_GRAVE)) {
        /* temblor YA confirmado y una ventana suelta sin detección: se mantiene la
         * última clase de temblor (máx. EMK_HOLD_HOPS) */
        category = c->last_tremor_cat;
    } else if (mov_grande) {
        category = TREMOR_CLASS_VOLUNTARIO;     /* regla original: > 100 °/s */
    } else {
        category = TREMOR_CLASS_SIN_TEMBLOR;
    }

#if TREMOR_DETECTOR_V2
    best_f   = tremor_detected ? fpk_axis[dom]  : 0.0f;
    ang_disp = tremor_detected ? theta_cal_deg  : 0.0f;
#endif

    /* deteccion "oficial" de temblor PD (confirmacion con histeresis) =
     * clase con temblor (LEVE, MODERADO o GRAVE) sostenida */
    if (tremor_detected) {
        c->last_tremor_cat = category;
        c->hold_left = EMK_HOLD_HOPS;
    } else if (vol_real || reposo_ya) {
        c->hold_left = 0;
    } else if (c->hold_left > 0) {
        c->hold_left--;
    }

    bool raw_det = (category == TREMOR_CLASS_LEVE ||
                    category == TREMOR_CLASS_MODERADO ||
                    category == TREMOR_CLASS_GRAVE);

    bool analizando = (c->warmup > 0);
    if (c->warmup > 0) {
        c->warmup--;
        category = TREMOR_CLASS_SIN_TEMBLOR;
        tremor_detected = false;
        raw_det = false;
        c->deb_true = c->deb_false = 0;
    }

    if (raw_det) {
        c->deb_true++;
        c->deb_false = 0;
    } else {
        c->deb_false++;
        c->deb_true = 0;
    }
    if (!c->present && c->deb_true >= DEBOUNCE_HOPS) {
        c->present = true;
        c->episode_ms = 0;
    }
    if (c->present && c->deb_false >= DEBOUNCE_HOPS) {
        c->present = false;
        c->episode_ms = 0;
    }
    if (c->present) {
        c->episode_ms += (uint32_t)(HOP_SEC * 1000.0f);
    }

    /* --- duty cycle del ultimo minuto --- */
    c->duty_ring[c->duty_idx] = c->present;
    c->duty_idx = (c->duty_idx + 1) % DUTY_HOPS;
    if (c->duty_n < DUTY_HOPS) {
        c->duty_n++;
    }
    int on = 0;
    for (int i = 0; i < c->duty_n; i++) {
        if (c->duty_ring[i]) {
            on++;
        }
    }

    r->category         = category;
    r->analizando       = analizando;
    r->tremor_present   = c->present;
    r->tremor_amp_dps   = tremor_amp;
    r->gyro_total_dps   = raw_ac;
    r->dominant_freq_hz = best_f;
    r->spectral_ratio   = ratio;
    r->movement_dps     = movement;
    r->net_rotation_deg = net_rot;
    r->band_fraction    = band_frac;
    r->freq_spread_hz   = (c->freq_hist_n >= STAB_HOPS) ? freq_spread : -1.0f;
    r->freq_stable      = freq_stable;
    r->duty_last_min    = (c->duty_n > 0) ? ((float)on / c->duty_n) : 0.0f;
    r->episode_ms       = c->episode_ms;
    r->accel_amp_mg     = accel_amp_mg;
    r->accel_gyro_ratio = accel_gyro_ratio;
    r->rotational       = rotational;
    r->ang_disp_deg     = ang_disp;

    /* --- resultado del detector V2 (se reporta siempre, decida o no) --- */
    for (int a = 0; a < 3; a++) {
        r->sigma_f_hz[a]    = sigma_f_axis[a];
        r->prel_pct[a]      = prel_axis[a];
        r->rms_axis_dps[a]  = rms_axis[a];
        r->fpeak_axis_hz[a] = fpk_axis[a];
    }
    r->votes         = (uint8_t)votos;
    r->active_axes   = (uint8_t)n_activos;
    r->gate_ok       = gate_ok;
    r->detect_v2     = detect_v2;
    r->theta_cal_deg = theta_cal_deg;
    r->tremor_detected = tremor_detected;
    r->vol110_pct    = vol110_pct;
    r->ruta          = ruta_num;
    r->lin           = lin;
    r->prel3         = prel3;
    r->rms_pc        = rms_pc;
    r->desplaz_cm    = desplaz_cm;

    /* --- perfilado del calculo por hop (V2): reportado cada ~10 s --- */
    int64_t t_hop_end_us = esp_timer_get_time();
    uint32_t dt_us = (uint32_t)(t_hop_end_us - t_hop_start_us);
    c->perf_sum_us += dt_us;
    c->perf_count++;
    if (dt_us > c->perf_max_us) {
        c->perf_max_us = dt_us;
    }
    if (c->perf_last_log_us == 0 || (t_hop_end_us - c->perf_last_log_us) >= 10000000LL) {
        ESP_LOGI(TAG, "perf V2: %u hop(s) en ~10s, media=%.2f ms, max=%.2f ms (este hop=%.2f ms)",
                 (unsigned)c->perf_count,
                 (double)c->perf_sum_us / (double)c->perf_count / 1000.0,
                 (double)c->perf_max_us / 1000.0,
                 (double)dt_us / 1000.0);
        c->perf_last_log_us = t_hop_end_us;
        c->perf_sum_us = 0;
        c->perf_max_us = 0;
        c->perf_count = 0;
    }

    return true;
}

#ifdef EMK_SELFTEST
/* ------------------------------------------------------------------ */
/* Autoprueba de los filtros (solo si EMK_SELFTEST esta definido)     */
/* ------------------------------------------------------------------ */

/* Ruido gaussiano estandar (media 0, sigma 1) por Box-Muller, a partir de
 * rand()/RAND_MAX. Solo para la autoprueba: no se usa en produccion. */
static float randn(void)
{
    float u1 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 2.0f);
    float u2 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 2.0f);
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * TR_PI * u2);
}

/* Caso A: seno de 5 Hz, 20 dps RMS, SOLO en gy (eje 1); ruido gaussiano de
 * 0.5 dps RMS en los 3 ejes. */
static float gen_case_a(int i, int axis)
{
    float t = (float)i / (float)EMK_FS_HZ;
    float noise = 0.5f * randn();
    if (axis == 1) {
        return 20.0f * 1.41421356f * sinf(2.0f * TR_PI * 5.0f * t) + noise;
    }
    return noise;
}

/* Caso B: ruido gaussiano de banda ancha, 30 dps RMS, en los 3 ejes. */
static float gen_case_b(int i, int axis)
{
    (void)i;
    (void)axis;
    return 30.0f * randn();
}

/* Caso G: seno de 1.5 Hz, 80 dps RMS, SOLO en gy (movimiento voluntario
 * rapido, p.ej. agitar la mano o gesticular). Debe rechazarse por la
 * compuerta de energia voluntaria (vol110_pct alto), no por falta de señal. */
static float gen_case_g(int i, int axis)
{
    float t = (float)i / (float)EMK_FS_HZ;
    if (axis == 1) {
        return 80.0f * 1.41421356f * sinf(2.0f * TR_PI * 1.5f * t);
    }
    return 0.0f;
}

/* Caso C: seno de 4 Hz, 70 dps RMS, SOLO en gy -> theta_cal ~3.7° (MODERADO). */
static float gen_case_c(int i, int axis)
{
    float t = (float)i / (float)EMK_FS_HZ;
    float noise = 0.5f * randn();
    if (axis == 1) {
        return 70.0f * 1.41421356f * sinf(2.0f * TR_PI * 4.0f * t) + noise;
    }
    return noise;
}

/* Caso D: seno de 3 Hz, 85 dps RMS, SOLO en gy -> theta_cal ~5.9° (GRAVE). */
static float gen_case_d(int i, int axis)
{
    float t = (float)i / (float)EMK_FS_HZ;
    float noise = 0.5f * randn();
    if (axis == 1) {
        return 85.0f * 1.41421356f * sinf(2.0f * TR_PI * 3.0f * t) + noise;
    }
    return noise;
}

/* Caso H: seno de 4 Hz, 300 dps RMS, SOLO en gy (imitacion de temblor MUY
 * fuerte, un solo eje). Regresion del bug "las imitaciones de temblor salen
 * como VOLUNTARIO": a esta amplitud, el pasa-bajos de 2 Hz (vol_lp) deja
 * pasar lo bastante del temblor como para que 'movement' crezca; si la
 * clasificacion mirara 'movement' antes que 'tremor_detected', esto saldria
 * VOLUNTARIO en vez de GRAVE (theta_cal ~15.8°). */
static float gen_case_h(int i, int axis)
{
    float t = (float)i / (float)EMK_FS_HZ;
    float noise = 0.5f * randn();
    if (axis == 1) {
        return 300.0f * 1.41421356f * sinf(2.0f * TR_PI * 4.0f * t) + noise;
    }
    return noise;
}

/* Corre 10 s de (gx,gy,gz) sinteticas (generadas por 'gen') por un
 * analizador nuevo y desechable, y devuelve por 'out' el ULTIMO resultado
 * valido (la ventana final, en regimen estacionario). El acelerometro se
 * deja fijo en "reloj plano y quieto" (gravedad en Z): el detector V2 no
 * usa el acelerometro, asi que no afecta a lo que se esta probando. */
static bool run_selftest_case(float (*gen)(int i, int axis), tremor_result_t *out)
{
    tremor_ctx_t *tc = tremor_create((float)EMK_FS_HZ);
    if (!tc) {
        return false;
    }
    bool have_any = false;
    tremor_result_t last = { 0 };
    const int n_samples = (int)(10.0f * EMK_FS_HZ + 0.5f);

    for (int i = 0; i < n_samples; i++) {
        float gx = gen(i, 0);
        float gy = gen(i, 1);
        float gz = gen(i, 2);
        tremor_result_t r;
        if (tremor_process(tc, gx, gy, gz, 0.0f, 0.0f, 1.0f, &r)) {
            last = r;
            have_any = true;
        }
    }
    tremor_destroy(tc);
    if (have_any) {
        *out = last;
    }
    return have_any;
}

/* Genera 10 s de seno a 50 Hz (RMS 20 dps) a cada frecuencia de referencia,
 * lo pasa por hp03+bp_g con el estado en cero, y registra el RMS de los
 * ultimos 5 s. No usa tremor_ctx_t: es una prueba aislada del filtro, sin
 * ventana ni logica de decision. Amplia con dos casos completos (A y B) que
 * si usan tremor_process(), para validar el detector V2 de punta a punta. */
void tremor_selftest_filters(void)
{
    static const float freqs_hz[]    = { 1.0f, 1.5f, 5.0f, 12.0f };
    static const float expected_dps[] = { 0.52f, 2.1f, 20.0f, 1.9f };
    const float fs      = (float)EMK_FS_HZ;
    const float amp_pk  = 20.0f * 1.41421356f;         /* RMS 20 dps -> amplitud de pico */
    const int   n_total = (int)(10.0f * fs + 0.5f);    /* 10 s */
    const int   n_tail  = (int)(5.0f  * fs + 0.5f);    /* ultimos 5 s */
    const int   n_head  = n_total - n_tail;

    ESP_LOGI(TAG, "==== autoprueba de filtros (hp03 + bp_g), fs=%d Hz ====", EMK_FS_HZ);

    for (int f = 0; f < 4; f++) {
        sos_cascade_t hp03, bp_g;
        sos_init(&hp03, EMK_HP_SOS, EMK_HP_N_BIQUADS);
        sos_init(&bp_g, EMK_BP_SOS, EMK_BP_N_BIQUADS);

        double sumsq = 0.0;
        for (int i = 0; i < n_total; i++) {
            float t = (float)i / fs;
            float x = amp_pk * sinf(2.0f * TR_PI * freqs_hz[f] * t);
            float h = sos_process(&hp03, x);
            float y = sos_process(&bp_g, h);
            if (i >= n_head) {
                sumsq += (double)y * (double)y;
            }
        }
        float rms     = sqrtf((float)(sumsq / n_tail));
        float exp_v   = expected_dps[f];
        float err_pct = (exp_v > 0.0f) ? (fabsf(rms - exp_v) / exp_v * 100.0f) : 0.0f;
        const char *veredicto = (err_pct <= 5.0f) ? "OK" : "FUERA DE TOLERANCIA";

        ESP_LOGI(TAG, "  %5.1f Hz -> RMS=%.3f dps (esperado %.2f dps, error %.1f%%) [%s]",
                 freqs_hz[f], rms, exp_v, err_pct, veredicto);
    }

    /* --- Casos A/B: deteccion completa (tremor_process), reloj simulado ---
     * Semilla fija -> autoprueba reproducible (no depende del azar real). */
    ESP_LOGI(TAG, "==== autoprueba detector V2 ====");
    srand(1234u);

    {
        tremor_result_t r;
        bool ok_run = run_selftest_case(gen_case_a, &r);
        /* Tolerancias: fpk debe caer justo en el bin de 5.0 Hz (exacto, el
         * barrido tiene paso 0.125 Hz y 5.0 cae en un bin). sigma_f y
         * theta_cal llevan una tolerancia generosa (+-20% o piso absoluto)
         * porque dependen de una realizacion concreta de ruido aleatorio;
         * ver el resumen de la respuesta para el detalle de este supuesto. */
        bool pass = ok_run &&
                    (r.active_axes == 1) && (r.votes == 1) && r.detect_v2 &&
                    r.tremor_detected && (r.category == TREMOR_CLASS_LEVE) &&
                    (fabsf(r.fpeak_axis_hz[1] - 5.0f) < 0.01f) &&
                    (r.prel_pct[1] >= 90.0f) &&
                    (fabsf(r.sigma_f_hz[1] - 0.29f) <= (0.29f * 0.20f + 0.05f)) &&
                    (fabsf(r.theta_cal_deg - 0.84f) <= (0.84f * 0.20f + 0.05f)) &&
                    (fabsf(r.desplaz_cm - 0.44f) <= (0.44f * 0.20f + 0.05f)) &&
                    (r.vol110_pct < 5.0f);
        ESP_LOGI(TAG,
                 "  Caso A (5 Hz, 20 dps RMS en gy): activos=%u votos=%u v2=%d "
                 "detectado=%d categoria=%s sigma_gy=%.3f prel_gy=%.1f%% fpk_gy=%.3f "
                 "theta_cal=%.3f desplaz_cm=%.3f vol110=%.2f%% -> [%s]",
                 ok_run ? r.active_axes : 0u, ok_run ? r.votes : 0u,
                 ok_run && r.detect_v2, ok_run && r.tremor_detected,
                 ok_run ? tremor_class_name(r.category) : "?",
                 ok_run ? r.sigma_f_hz[1] : -1.0f, ok_run ? r.prel_pct[1] : -1.0f,
                 ok_run ? r.fpeak_axis_hz[1] : -1.0f, ok_run ? r.theta_cal_deg : -1.0f,
                 ok_run ? r.desplaz_cm : -1.0f, ok_run ? r.vol110_pct : -1.0f,
                 pass ? "PASA" : "FALLA");
    }

    {
        tremor_result_t r;
        bool ok_run = run_selftest_case(gen_case_c, &r);
        bool pass = ok_run && r.tremor_detected &&
                    (r.category == TREMOR_CLASS_MODERADO) &&
                    (fabsf(r.theta_cal_deg - 3.7f) <= (3.7f * 0.20f + 0.05f));
        ESP_LOGI(TAG,
                 "  Caso C (4 Hz, 70 dps RMS en gy): detectado=%d categoria=%s "
                 "theta_cal=%.3f desplaz_cm=%.3f -> [%s]",
                 ok_run && r.tremor_detected, ok_run ? tremor_class_name(r.category) : "?",
                 ok_run ? r.theta_cal_deg : -1.0f, ok_run ? r.desplaz_cm : -1.0f,
                 pass ? "PASA" : "FALLA");
    }

    {
        tremor_result_t r;
        bool ok_run = run_selftest_case(gen_case_d, &r);
        bool pass = ok_run && r.tremor_detected &&
                    (r.category == TREMOR_CLASS_GRAVE) &&
                    (fabsf(r.theta_cal_deg - 5.9f) <= (5.9f * 0.20f + 0.05f));
        ESP_LOGI(TAG,
                 "  Caso D (3 Hz, 85 dps RMS en gy): detectado=%d categoria=%s "
                 "theta_cal=%.3f desplaz_cm=%.3f -> [%s]",
                 ok_run && r.tremor_detected, ok_run ? tremor_class_name(r.category) : "?",
                 ok_run ? r.theta_cal_deg : -1.0f, ok_run ? r.desplaz_cm : -1.0f,
                 pass ? "PASA" : "FALLA");
    }

    {
        tremor_result_t r;
        bool ok_run = run_selftest_case(gen_case_h, &r);
        /* 4 Hz/300 dps da theta_cal ~15.8° -> GRAVE; lo que importa para la
         * regresion es que NO salga VOLUNTARIO. */
        bool pass = ok_run && (r.ruta == 1 || r.ruta == 2) && r.detect_v2 &&
                    r.tremor_detected && (r.category == TREMOR_CLASS_GRAVE);
        ESP_LOGI(TAG,
                 "  Caso H (4 Hz, 300 dps RMS en gy, imitacion MUY fuerte): "
                 "ruta=%u v2=%d detectado=%d categoria=%s lin=%.3f prel3=%.1f%% "
                 "rms_pc=%.1f movement=%.1f desplaz_cm=%.3f -> [%s]",
                 ok_run ? r.ruta : 0u, ok_run && r.detect_v2, ok_run && r.tremor_detected,
                 ok_run ? tremor_class_name(r.category) : "?",
                 ok_run ? r.lin : -1.0f, ok_run ? r.prel3 : -1.0f,
                 ok_run ? r.rms_pc : -1.0f, ok_run ? r.movement_dps : -1.0f,
                 ok_run ? r.desplaz_cm : -1.0f,
                 pass ? "PASA" : "FALLA");
    }

    {
        tremor_result_t r;
        bool ok_run = run_selftest_case(gen_case_b, &r);
        bool pass = ok_run && !r.detect_v2 && (r.category == TREMOR_CLASS_SIN_TEMBLOR);
        ESP_LOGI(TAG, "  Caso B (ruido 30 dps banda ancha, 3 ejes): v2=%d categoria=%s -> [%s]",
                 ok_run && r.detect_v2, ok_run ? tremor_class_name(r.category) : "?",
                 pass ? "PASA" : "FALLA");
    }

    {
        tremor_result_t r;
        bool ok_run = run_selftest_case(gen_case_g, &r);
        bool pass = ok_run && !r.detect_v2 && (r.vol110_pct > 60.0f);
        ESP_LOGI(TAG,
                 "  Caso G (1.5 Hz, 80 dps RMS en gy, mov. voluntario rapido): "
                 "v2=%d vol110=%.2f%% categoria=%s -> [%s]",
                 ok_run && r.detect_v2, ok_run ? r.vol110_pct : -1.0f,
                 ok_run ? tremor_class_name(r.category) : "?",
                 pass ? "PASA" : "FALLA");
    }
}
#endif /* EMK_SELFTEST */
