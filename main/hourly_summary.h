#ifndef HOURLY_SUMMARY_H
#define HOURLY_SUMMARY_H

/*
 * hourly_summary.h
 * ---------------------------------------------------------------------------
 * Agregador de resumenes horarios del temblor, con almacenamiento local:
 *
 *   - Se alimenta con cada resultado del detector (~2 Hz) via hs_feed().
 *   - Cada HS_PERIOD_S (300 s = 5 min) cierra un resumen (promedios, maximos,
 *     nº de episodios, tiempo con temblor, reparto por clase) y lo guarda
 *     en un buffer circular en RAM (HS_RING_N registros) y en NVS (flash).
 *   - Cada registro lleva 'seq' (contador monotono) y un timestamp:
 *       * epoch_s  si la app sincronizo la hora (CMD 0x10 + uint32 epoch LE)
 *       * si no, epoch_s = 0 y la app usa uptime_s para situarlo en el tiempo.
 *   - Entrega: mientras hay BLE, el firmware va enviando por la caracteristica
 *     SUMMARY los registros con seq > last_acked. La app confirma con
 *     CMD 0x20 + uint32 seq LE; ese 'last_acked' se persiste en NVS, asi que
 *     al reconectar (o tras un reinicio) el wearable "vacia" solo lo pendiente.
 *
 * Para PRUEBAS: bajar HS_PERIOD_S (p.ej. a 60) en hourly_summary.c.
 * ---------------------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>
#include "tremor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HS_RING_N       48      /* resumenes retenidos: 48 x 5 min = 4 horas */
#define HS_RECORD_SIZE  36      /* tamaño en el aire (BLE), little-endian */

/* bits del campo 'flags' del registro */
#define HS_FLAG_TIME_SYNCED   0x01
#define HS_FLAG_TAXONOMY_V2   0x02   /* usa la taxonomia SIN_TEMBLOR/LEVE/MODERADO/
                                      * GRAVE/VOLUNTARIO (con hops_leve al final) */

typedef struct __attribute__((packed)) {
    uint32_t seq;              /* contador monotono, 1..N */
    uint32_t epoch_s;          /* hora UNIX del cierre (0 si no sincronizada) */
    uint32_t uptime_s;         /* segundos desde arranque al cierre */
    uint16_t duration_s;       /* duracion real acumulada (~300) */
    uint16_t hops_total;       /* nº de ventanas de 0.5 s agregadas */
    uint16_t hops_moderado;    /* ventanas clasificadas MODERADO */
    uint16_t hops_grave;       /* ventanas clasificadas GRAVE */
    uint16_t hops_voluntario;  /* ventanas clasificadas ACCION VOLUNTARIA */
    uint16_t episodes;         /* nº de episodios de temblor (flancos de subida) */
    uint16_t tremor_time_s;    /* segundos con temblor confirmado */
    uint16_t amp_mean_ddps;    /* amplitud media durante el temblor, dps * 10 */
    uint16_t amp_max_ddps;     /* amplitud maxima, dps * 10 */
    uint16_t gyro_max_ddps;    /* velocidad angular total maxima, dps * 10 */
    uint8_t  freq_mean_ddhz;   /* frecuencia media durante el temblor, Hz * 10 */
    uint8_t  flags;            /* bit0 = hora sincronizada, bit1 = taxonomia v2 */
    uint16_t hops_leve;        /* NUEVO (al final): ventanas clasificadas LEVE */
} hs_record_t;

_Static_assert(sizeof(hs_record_t) == HS_RECORD_SIZE, "hs_record_t debe medir 36 bytes");

/* Carga el estado desde NVS. Llamar una vez, despues de nvs_flash_init(). */
void hs_init(void);

/* Sincroniza la hora UNIX (segundos). La manda la app al conectar. */
void hs_set_epoch(uint32_t epoch_s);

/* Alimenta el agregador con un resultado del detector. Llamar una vez por hop. */
void hs_feed(const tremor_result_t *res);

/* --- Entrega de pendientes (los usa el modulo BLE) --- */
void     hs_iter_reset(void);              /* reinicia el recorrido de pendientes */
bool     hs_iter_next(hs_record_t *out);   /* siguiente registro con seq > last_acked */
void     hs_ack(uint32_t seq);             /* la app confirma recepcion hasta 'seq' */
uint32_t hs_pending_count(void);           /* nº de resumenes sin confirmar */

/* Hora UNIX actual (segundos), si la app ya la sincronizo (CMD 0x10).
 * Devuelve false (y no toca *epoch_s) si todavia no hay hora sincronizada. */
bool hs_get_epoch(uint32_t *epoch_s);

#ifdef __cplusplus
}
#endif

#endif /* HOURLY_SUMMARY_H */
