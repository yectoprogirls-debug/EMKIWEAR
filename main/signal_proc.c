/*
 * signal_proc.c  -- ver signal_proc.h
 */

#include "signal_proc.h"
#include <math.h>

#define SP_PI     3.14159265358979323846f
#define SP_RAD2DEG 57.2957795130823f
#define SP_DEG2RAD 0.01745329251994f

/* ================================================================== */
/* Biquad                                                             */
/* ================================================================== */

static void biquad_set(biquad_t *bq,
                       float b0, float b1, float b2,
                       float a0, float a1, float a2)
{
    /* normaliza por a0 */
    bq->b0 = b0 / a0;
    bq->b1 = b1 / a0;
    bq->b2 = b2 / a0;
    bq->a1 = a1 / a0;
    bq->a2 = a2 / a0;
    bq->z1 = 0.0f;
    bq->z2 = 0.0f;
}

void biquad_design_lowpass(biquad_t *bq, float fc_hz, float fs_hz, float q)
{
    float w0    = 2.0f * SP_PI * fc_hz / fs_hz;
    float cw    = cosf(w0);
    float sw    = sinf(w0);
    float alpha = sw / (2.0f * q);

    float b0 = (1.0f - cw) * 0.5f;
    float b1 = (1.0f - cw);
    float b2 = (1.0f - cw) * 0.5f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cw;
    float a2 = 1.0f - alpha;

    biquad_set(bq, b0, b1, b2, a0, a1, a2);
}

void biquad_design_highpass(biquad_t *bq, float fc_hz, float fs_hz, float q)
{
    float w0    = 2.0f * SP_PI * fc_hz / fs_hz;
    float cw    = cosf(w0);
    float sw    = sinf(w0);
    float alpha = sw / (2.0f * q);

    float b0 = (1.0f + cw) * 0.5f;
    float b1 = -(1.0f + cw);
    float b2 = (1.0f + cw) * 0.5f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cw;
    float a2 = 1.0f - alpha;

    biquad_set(bq, b0, b1, b2, a0, a1, a2);
}

void biquad_reset(biquad_t *bq)
{
    bq->z1 = 0.0f;
    bq->z2 = 0.0f;
}

float biquad_process(biquad_t *bq, float x)
{
    /* Direct Form II transposed: estable numericamente y barato */
    float y  = bq->b0 * x + bq->z1;
    bq->z1   = bq->b1 * x - bq->a1 * y + bq->z2;
    bq->z2   = bq->b2 * x - bq->a2 * y;
    return y;
}

void biquad_set_coeffs(biquad_t *bq, const float c[5])
{
    bq->b0 = c[0];
    bq->b1 = c[1];
    bq->b2 = c[2];
    bq->a1 = c[3];
    bq->a2 = c[4];
    bq->z1 = 0.0f;
    bq->z2 = 0.0f;
}

/* ================================================================== */
/* Cascada de secciones de segundo orden (SOS)                       */
/* ================================================================== */

void sos_init(sos_cascade_t *c, const float coef[][5], int n)
{
    if (n > SOS_MAX_SECTIONS) {
        n = SOS_MAX_SECTIONS;   /* proteccion: no deberia pasar en este proyecto */
    }
    c->n = n;
    for (int i = 0; i < n; i++) {
        biquad_set_coeffs(&c->s[i], coef[i]);
    }
}

void sos_reset(sos_cascade_t *c)
{
    for (int i = 0; i < c->n; i++) {
        biquad_reset(&c->s[i]);
    }
}

float sos_process(sos_cascade_t *c, float x)
{
    float y = x;
    for (int i = 0; i < c->n; i++) {
        y = biquad_process(&c->s[i], y);
    }
    return y;
}

/* ================================================================== */
/* Filtro complementario                                              */
/* ================================================================== */

void comp_filter_init(comp_filter_t *cf, float fs_hz, float tau_s)
{
    float dt = 1.0f / fs_hz;
    cf->fs_hz       = fs_hz;
    cf->alpha       = tau_s / (tau_s + dt);
    cf->roll_deg    = 0.0f;
    cf->pitch_deg   = 0.0f;
    cf->initialised = false;
}

void comp_filter_update(comp_filter_t *cf,
                        float ax_g, float ay_g, float az_g,
                        float gx_dps, float gy_dps, float gz_dps)
{
    (void)gz_dps;   /* yaw no observable sin magnetometro */

    float dt = 1.0f / cf->fs_hz;

    /* Angulos a partir del vector gravedad medido por el acelerometro */
    float roll_acc  = atan2f(ay_g, az_g) * SP_RAD2DEG;
    float pitch_acc = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * SP_RAD2DEG;

    if (!cf->initialised) {
        cf->roll_deg    = roll_acc;
        cf->pitch_deg   = pitch_acc;
        cf->initialised = true;
        return;
    }

    /* Prediccion por integracion del giroscopio + correccion con el acelerometro */
    float roll_gyro  = cf->roll_deg  + gx_dps * dt;
    float pitch_gyro = cf->pitch_deg + gy_dps * dt;

    cf->roll_deg  = cf->alpha * roll_gyro  + (1.0f - cf->alpha) * roll_acc;
    cf->pitch_deg = cf->alpha * pitch_gyro + (1.0f - cf->alpha) * pitch_acc;
}

/* ================================================================== */
/* Bias del giroscopio                                                */
/* ================================================================== */

void gyro_bias_start(gyro_bias_t *gb, uint16_t n_samples)
{
    gb->sx = gb->sy = gb->sz = 0.0;
    gb->count  = 0;
    gb->target = (n_samples > 0) ? n_samples : 200;
    gb->bx = gb->by = gb->bz = 0.0f;
    gb->done = false;
}

bool gyro_bias_feed(gyro_bias_t *gb, float gx_dps, float gy_dps, float gz_dps)
{
    if (gb->done) {
        return true;
    }

    gb->sx += gx_dps;
    gb->sy += gy_dps;
    gb->sz += gz_dps;
    gb->count++;

    if (gb->count >= gb->target) {
        gb->bx   = (float)(gb->sx / gb->count);
        gb->by   = (float)(gb->sy / gb->count);
        gb->bz   = (float)(gb->sz / gb->count);
        gb->done = true;
    }
    return gb->done;
}
