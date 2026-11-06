#ifndef SIGNAL_PROC_H
#define SIGNAL_PROC_H

/*
 * signal_proc.h
 * ---------------------------------------------------------------------------
 * Bloques de procesamiento de señal reutilizables para la cadena de la IMU:
 *   - Biquad IIR (con diseño de pasa-bajo / pasa-alto tipo Butterworth)
 *   - Filtro complementario para estimar roll/pitch
 *   - Estimacion del bias (offset) del giroscopio con la IMU quieta
 *
 * Todo trabaja en 'float'. La frecuencia de muestreo (fs) se pasa en el init
 * de cada bloque; aqui se asume muestreo uniforme (dt = 1/fs constante).
 * ---------------------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Q de un Butterworth de 2º orden (un solo biquad). */
#define BUTTERWORTH_Q 0.70710678f

/* ------------------------------------------------------------------ */
/* Biquad IIR (Direct Form II transposed)                            */
/* ------------------------------------------------------------------ */
typedef struct {
    float b0, b1, b2;   /* numerador  (a0 ya normalizado a 1) */
    float a1, a2;       /* denominador */
    float z1, z2;       /* estado interno */
} biquad_t;

/* Diseña el biquad como pasa-bajo / pasa-alto (bilinear transform, RBJ). */
void  biquad_design_lowpass (biquad_t *bq, float fc_hz, float fs_hz, float q);
void  biquad_design_highpass(biquad_t *bq, float fc_hz, float fs_hz, float q);

/* Pone el estado a cero (los coeficientes se conservan). */
void  biquad_reset(biquad_t *bq);

/* Procesa una muestra y devuelve la salida filtrada. */
float biquad_process(biquad_t *bq, float x);

/* Copia coeficientes {b0,b1,b2,a1,a2} (a0=1, ya normalizado) directamente en
 * el biquad -- para filtros diseñados fuera (p.ej. en MATLAB/SciPy) en vez
 * de con biquad_design_lowpass/highpass. Pone z1=z2=0. */
void biquad_set_coeffs(biquad_t *bq, const float c[5]);

/* ------------------------------------------------------------------ */
/* Cascada de secciones de segundo orden (SOS)                       */
/* ------------------------------------------------------------------ */
#define SOS_MAX_SECTIONS 4

typedef struct {
    biquad_t s[SOS_MAX_SECTIONS];
    int      n;    /* secciones realmente usadas (<= SOS_MAX_SECTIONS) */
} sos_cascade_t;

/* Inicializa la cascada con 'n' secciones (coef[i] = {b0,b1,b2,a1,a2},
 * a0=1) y pone el estado a cero. n se recorta a SOS_MAX_SECTIONS si hiciera
 * falta (no deberia ocurrir con los filtros del proyecto). */
void sos_init(sos_cascade_t *c, const float coef[][5], int n);

/* Pone el estado de todas las secciones a cero (coeficientes intactos). */
void sos_reset(sos_cascade_t *c);

/* Procesa una muestra por las n secciones EN ORDEN (la salida de una es la
 * entrada de la siguiente) y devuelve el resultado final. */
float sos_process(sos_cascade_t *c, float x);

/* ------------------------------------------------------------------ */
/* Filtro complementario: roll / pitch en grados                     */
/* ------------------------------------------------------------------ */
typedef struct {
    float roll_deg;
    float pitch_deg;
    float alpha;        /* peso del giroscopio: alpha = tau / (tau + dt) */
    float fs_hz;
    bool  initialised;
} comp_filter_t;

/* tau_s: constante de tiempo del cruce giro/acelerometro (~0.5 .. 1 s). */
void comp_filter_init(comp_filter_t *cf, float fs_hz, float tau_s);

/* ax..az en g ; gx..gz en grados/s (ya con el bias restado). */
void comp_filter_update(comp_filter_t *cf,
                        float ax_g, float ay_g, float az_g,
                        float gx_dps, float gy_dps, float gz_dps);

/* ------------------------------------------------------------------ */
/* Estimacion del bias del giroscopio (IMU quieta)                   */
/* ------------------------------------------------------------------ */
typedef struct {
    double   sx, sy, sz;   /* acumuladores */
    uint16_t count;
    uint16_t target;       /* nº de muestras a promediar */
    float    bx, by, bz;   /* bias resultante (grados/s) */
    bool     done;
} gyro_bias_t;

void gyro_bias_start(gyro_bias_t *gb, uint16_t n_samples);

/* Alimenta una muestra cruda del giroscopio. Devuelve true cuando ya hay
 * bias calculado (a partir de ese momento gb->bx/by/bz son validos). */
bool gyro_bias_feed(gyro_bias_t *gb, float gx_dps, float gy_dps, float gz_dps);

#ifdef __cplusplus
}
#endif

#endif /* SIGNAL_PROC_H */
