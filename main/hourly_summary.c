/*
 * hourly_summary.c  -- ver hourly_summary.h
 */

#include "hourly_summary.h"

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"

static const char *TAG = "HOURLY";

#define HS_PERIOD_S      300      /* cada 5 min se cierra y se guarda un resumen en NVS
                                  * (60 para pruebas rapidas; 3600 = por hora) */
#define HS_ACCUM_SAVE_S  300      /* persistir el acumulador en curso; con periodo de
                                  * 5 min coincide con el cierre, asi que no aporta,
                                  * pero protege si se sube HS_PERIOD_S */
#define HOP_S            0.5f     /* debe coincidir con el hop del detector */
#define NVS_NS           "pkwatch"

/* --- acumulador de la ventana en curso (se persiste en NVS periodicamente) --- */
typedef struct {
    uint32_t t_start_uptime;
    uint32_t hops_total;
    uint32_t hops_moderado;
    uint32_t hops_grave;
    uint32_t hops_voluntario;
    uint32_t hops_leve;
    uint32_t tremor_hops;
    double   amp_sum;
    double   freq_sum;
    float    amp_max;
    float    gyro_max;
    uint32_t episodes;
    uint8_t  prev_present;
} hs_accum_t;

/* --- metadatos persistentes --- */
typedef struct {
    uint32_t head;
    uint32_t count;
    uint32_t seq_next;
    uint32_t last_acked;
} hs_meta_t;

static hs_record_t s_ring[HS_RING_N];
static hs_meta_t   s_meta = { 0, 0, 1, 0 };
static hs_accum_t  s_acc;

static uint32_t s_epoch_at_boot;       /* epoch - uptime cuando se sincronizo */
static bool     s_time_synced;
static uint32_t s_last_accum_save;
static uint32_t s_iter_idx;
static uint32_t s_first_seq_this_boot;  /* primer seq generado en este arranque:
                                         * solo estos se pueden datar al sincronizar
                                         * (los de arranques previos tienen uptime
                                         *  de una sesion ya inexistente) */

/* ------------------------------------------------------------------ */

static uint32_t now_s(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000);
}

static uint16_t clamp_u16(uint32_t v)
{
    return (v > 65535u) ? 65535u : (uint16_t)v;
}

/* Escrituras a NVS separadas por clave para minimizar el desgaste de flash:
 *  - "ring2" (1.7 KB) solo cambia al cerrar un resumen  -> 1 vez/hora
 *  - "meta2" (16 B)   cambia al cerrar y al recibir ACK -> ~2 veces/hora
 *  - "acc2"  (~60 B)  se guarda periodicamente para no perder la hora en curso
 * NVS reparte las escrituras por toda la particion (wear leveling), asi que
 * incluso el caso peor (~1 reescritura completa cada 6 h) da decadas de vida.
 * Claves "...2" (no "ring"/"meta"/"acc"): el formato de hs_record_t cambio de
 * 34 a 36 bytes (taxonomia nueva, hops_leve al final); con las claves viejas
 * un blob guardado con el formato anterior se leeria mal (tamaño distinto). */
static void nvs_save_ring(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_blob(h, "ring2", s_ring, sizeof(s_ring));
    nvs_set_blob(h, "meta2", &s_meta, sizeof(s_meta));
    nvs_commit(h);
    nvs_close(h);
}

static void nvs_save_meta(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_blob(h, "meta2", &s_meta, sizeof(s_meta));
    nvs_commit(h);
    nvs_close(h);
}

static void nvs_save_acc(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_blob(h, "acc2", &s_acc, sizeof(s_acc));
    nvs_commit(h);
    nvs_close(h);
}

static void nvs_load_all(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;   /* primera vez: no hay nada guardado */
    }
    size_t sz;

    sz = sizeof(s_ring);
    nvs_get_blob(h, "ring2", s_ring, &sz);

    sz = sizeof(s_meta);
    nvs_get_blob(h, "meta2", &s_meta, &sz);
    if (s_meta.seq_next == 0) {
        s_meta.seq_next = 1;
    }

    sz = sizeof(s_acc);
    nvs_get_blob(h, "acc2", &s_acc, &sz);   /* continua la hora que estaba en curso */

    nvs_close(h);
}

static void accum_reset(void)
{
    memset(&s_acc, 0, sizeof(s_acc));
    s_acc.t_start_uptime = now_s();
}

/* ------------------------------------------------------------------ */

void hs_init(void)
{
    memset(s_ring, 0, sizeof(s_ring));
    accum_reset();
    nvs_load_all();
    s_last_accum_save = now_s();
    s_first_seq_this_boot = s_meta.seq_next;   /* lo que se genere a partir de ahora */
    ESP_LOGI(TAG, "init: %u resumenes en ring, seq_next=%u, last_acked=%u",
             (unsigned)s_meta.count, (unsigned)s_meta.seq_next,
             (unsigned)s_meta.last_acked);
}

void hs_set_epoch(uint32_t epoch_s)
{
    uint32_t up = now_s();
    s_epoch_at_boot = (epoch_s > up) ? (epoch_s - up) : 0;
    s_time_synced = true;

    /* Back-fill: datar los resumenes que se cerraron mientras el movil no
     * estaba (mismo arranque, todavia sin epoch). Asi la app los mete en el
     * bucket de hora correcto al reconectar. */
    int fixed = 0;
    for (uint32_t i = 0; i < HS_RING_N; i++) {
        hs_record_t *r = &s_ring[i];
        if (r->seq >= s_first_seq_this_boot && r->seq != 0 &&
            r->epoch_s == 0 && r->uptime_s > 0) {
            r->epoch_s = s_epoch_at_boot + r->uptime_s;
            r->flags  |= HS_FLAG_TIME_SYNCED;
            fixed++;
        }
    }
    if (fixed > 0) {
        nvs_save_ring();
    }
    ESP_LOGI(TAG, "hora sincronizada: epoch=%u (uptime=%u), %d resumenes datados",
             (unsigned)epoch_s, (unsigned)up, fixed);
}

static void finalize_window(void)
{
    hs_record_t *r = &s_ring[s_meta.head];
    memset(r, 0, sizeof(*r));

    uint32_t up = now_s();
    r->seq      = s_meta.seq_next++;
    r->uptime_s = up;
    r->epoch_s  = s_time_synced ? (s_epoch_at_boot + up) : 0;
    r->duration_s      = clamp_u16(up - s_acc.t_start_uptime);
    r->hops_total      = clamp_u16(s_acc.hops_total);
    r->hops_moderado   = clamp_u16(s_acc.hops_moderado);
    r->hops_grave      = clamp_u16(s_acc.hops_grave);
    r->hops_voluntario = clamp_u16(s_acc.hops_voluntario);
    r->episodes        = clamp_u16(s_acc.episodes);
    r->tremor_time_s   = clamp_u16((uint32_t)(s_acc.tremor_hops * HOP_S));
    if (s_acc.tremor_hops > 0) {
        r->amp_mean_ddps  = clamp_u16((uint32_t)((s_acc.amp_sum  / s_acc.tremor_hops) * 10.0));
        uint32_t fm       = (uint32_t)((s_acc.freq_sum / s_acc.tremor_hops) * 10.0);
        r->freq_mean_ddhz = (fm > 255u) ? 255u : (uint8_t)fm;
    }
    r->amp_max_ddps  = clamp_u16((uint32_t)(s_acc.amp_max  * 10.0f));
    r->gyro_max_ddps = clamp_u16((uint32_t)(s_acc.gyro_max * 10.0f));
    r->hops_leve     = clamp_u16(s_acc.hops_leve);
    r->flags         = (s_time_synced ? HS_FLAG_TIME_SYNCED : 0) | HS_FLAG_TAXONOMY_V2;

    s_meta.head = (s_meta.head + 1) % HS_RING_N;
    if (s_meta.count < HS_RING_N) {
        s_meta.count++;
    }

    ESP_LOGI(TAG,
             "resumen #%u: dur=%us  temblor=%us  episodios=%u  ampMedia=%.1f ampMax=%.1f dps",
             (unsigned)r->seq, r->duration_s, r->tremor_time_s, r->episodes,
             r->amp_mean_ddps / 10.0f, s_acc.amp_max);

    accum_reset();
    nvs_save_ring();          /* el ring solo cambia aqui: 1 escritura/hora */
    s_last_accum_save = now_s();
}

void hs_feed(const tremor_result_t *res)
{
    s_acc.hops_total++;
    switch (res->category) {
    case TREMOR_CLASS_LEVE:       s_acc.hops_leve++;       break;
    case TREMOR_CLASS_MODERADO:   s_acc.hops_moderado++;   break;
    case TREMOR_CLASS_GRAVE:      s_acc.hops_grave++;      break;
    case TREMOR_CLASS_VOLUNTARIO: s_acc.hops_voluntario++; break;
    default: break;
    }

    if (res->tremor_present) {
        s_acc.tremor_hops++;
        s_acc.amp_sum  += res->tremor_amp_dps;
        s_acc.freq_sum += res->dominant_freq_hz;
        if (res->tremor_amp_dps > s_acc.amp_max) {
            s_acc.amp_max = res->tremor_amp_dps;
        }
        if (!s_acc.prev_present) {
            s_acc.episodes++;
        }
    }
    if (res->gyro_total_dps > s_acc.gyro_max) {
        s_acc.gyro_max = res->gyro_total_dps;
    }
    s_acc.prev_present = res->tremor_present ? 1 : 0;

    uint32_t up = now_s();
    if (up - s_acc.t_start_uptime >= HS_PERIOD_S) {
        finalize_window();
    } else if (up - s_last_accum_save >= HS_ACCUM_SAVE_S) {
        s_last_accum_save = up;
        nvs_save_acc();   /* ~60 B, por si hay reinicio a mitad de la hora */
    }
}

/* ------------------------------------------------------------------ */

void hs_iter_reset(void)
{
    s_iter_idx = 0;
}

bool hs_iter_next(hs_record_t *out)
{
    uint32_t start = (s_meta.head - s_meta.count + HS_RING_N) % HS_RING_N;
    while (s_iter_idx < s_meta.count) {
        const hs_record_t *r = &s_ring[(start + s_iter_idx) % HS_RING_N];
        s_iter_idx++;
        if (r->seq > s_meta.last_acked) {
            memcpy(out, r, sizeof(*out));
            return true;
        }
    }
    return false;
}

void hs_ack(uint32_t seq)
{
    if (seq > s_meta.last_acked && seq < s_meta.seq_next) {
        s_meta.last_acked = seq;
        nvs_save_meta();   /* 16 B */
        ESP_LOGI(TAG, "ack: resumenes confirmados hasta seq=%u", (unsigned)seq);
    }
}

uint32_t hs_pending_count(void)
{
    uint32_t n = 0;
    uint32_t start = (s_meta.head - s_meta.count + HS_RING_N) % HS_RING_N;
    for (uint32_t i = 0; i < s_meta.count; i++) {
        if (s_ring[(start + i) % HS_RING_N].seq > s_meta.last_acked) {
            n++;
        }
    }
    return n;
}

bool hs_get_epoch(uint32_t *epoch_s)
{
    if (!s_time_synced) {
        return false;
    }
    *epoch_s = s_epoch_at_boot + now_s();
    return true;
}
