/*
 * ble_stream.c  -- ver ble_stream.h
 *
 * Servidor GATT con NimBLE. Mantiene la ultima muestra y el ultimo estado en
 * buffers estaticos (para responder a READ) y envia NOTIFY al cliente
 * suscrito.
 */

#include "ble_stream.h"
#include "hourly_summary.h"
#include "bmi270_esp32.h"
#include "charge.h"

#include <string.h>
#include <math.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_bt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* lo aporta el componente 'bt' (store por defecto en RAM) */
extern void ble_store_config_init(void);

static const char *TAG = "BLE_STREAM";
#define DEVICE_NAME "ParkinsonWatch"

/* ---- UUIDs 7A9B000x-2D4E-4E6F-8A1B-0C2D3E4F5061 (LSB primero) ---- */
#define WATCH_UUID128(b12) BLE_UUID128_DECLARE( \
    0x61, 0x50, 0x4F, 0x3E, 0x2D, 0x0C, 0x1B, 0x8A, \
    0x6F, 0x4E, 0x4E, 0x2D, (b12), 0x00, 0x9B, 0x7A)

static const ble_uuid128_t s_svc_uuid = BLE_UUID128_INIT(
    0x61, 0x50, 0x4F, 0x3E, 0x2D, 0x0C, 0x1B, 0x8A,
    0x6F, 0x4E, 0x4E, 0x2D, 0x01, 0x00, 0x9B, 0x7A);

/* ---- Estructuras de datos en el aire (little-endian, empaquetadas) ---- */
typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    int16_t  roll_cdeg;
    int16_t  pitch_cdeg;
    int16_t  lin_acc_mg;
    int16_t  gyro_mag_ddps;
    int16_t  tremor_amp_ddps;
} ble_sample_t;

typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    uint8_t  flags;
    uint8_t  dom_freq_ddhz;
    int16_t  tremor_amp_ddps;
    int16_t  movement_ddps;
    uint8_t  band_frac_pct;
    uint8_t  duty_pct;
    uint16_t episode_s;
} ble_status_t;

typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    uint16_t accel_amp_mg;
    uint16_t accel_gyro_x100;
    uint16_t ang_disp_cdeg;
    uint8_t  flags2;
    uint8_t  category;   /* 0 SIN_TEMBLOR, 1 LEVE, 2 MODERADO, 3 GRAVE, 4 VOLUNTARIO */
} ble_ext_t;

typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    int16_t  ax_mg;      /* accel X, g * 1000 (crudo) */
    int16_t  ay_mg;
    int16_t  az_mg;
    int16_t  gx_ddps;    /* giro X, grados/s * 10 (crudo, con bias de fabrica) */
    int16_t  gy_ddps;
    int16_t  gz_ddps;
} ble_raw_t;

typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    uint16_t theta_cdeg;   /* theta_cal_deg, grados * 100 */
    uint16_t desplaz_mm;   /* desplaz_cm, milimetros */
} ble_sev_t;

_Static_assert(sizeof(ble_sample_t) == 14, "ble_sample_t debe medir 14 bytes");
_Static_assert(sizeof(ble_status_t) == 14, "ble_status_t debe medir 14 bytes");
_Static_assert(sizeof(ble_ext_t)    == 12, "ble_ext_t debe medir 12 bytes");
_Static_assert(sizeof(ble_raw_t)    == 16, "ble_raw_t debe medir 16 bytes");
_Static_assert(sizeof(ble_sev_t)    == 8,  "ble_sev_t debe medir 8 bytes");

/* ---- Caracteristica 0x08: lotes de FIFO con cabecera (para MATLAB) ---- */
typedef struct __attribute__((packed)) {
    uint16_t seq_primera;
    uint32_t t0_us;
    uint8_t  n_muestras;
} ble_rawfifo_hdr_t;

_Static_assert(sizeof(ble_rawfifo_hdr_t)   == 7,  "cabecera 0x08 debe medir 7 bytes");
_Static_assert(sizeof(bmi270_raw_sample_t) == 12, "muestra 0x08 debe medir 12 bytes");

/* ODR fijo del proyecto (ver bmi270_sensor_init): necesario aqui solo para
 * calcular el t0_us de cada sub-paquete cuando un lote de la FIFO hay que
 * partirlo en varias notificaciones por el MTU. Si cambia el ODR en
 * bmi270_esp32.c, cambiar tambien esta constante. */
#define BLE_RAWFIFO_ODR_HZ            50
#define BLE_RAWFIFO_PERIOD_US         (1000000 / BLE_RAWFIFO_ODR_HZ)
#define BLE_RAWFIFO_MAX_SAMPLES_PKT   42   /* holgado para el MTU maximo de la especificacion BLE (517) */

/* ---- estado ---- */
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_sample_handle;
static uint16_t s_status_handle;
static uint16_t s_ext_handle;
static uint16_t s_summary_handle;
static uint16_t s_raw_handle;
static uint16_t s_rawfifo_handle;       /* 0x08 */
static uint16_t s_cmd_handle;           /* 0x04 (ahora tambien NOTIFY+READ, para INFO) */
static uint16_t s_sev_handle;           /* 0x0A (severidad, escala MDS-UPDRS) */
static uint16_t s_batt_handle;          /* 0x09 (bateria / aviso de modo carga) */
static bool     s_sample_subscribed;
static bool     s_status_subscribed;
static bool     s_ext_subscribed;
static bool     s_summary_subscribed;
static bool     s_raw_subscribed;
static bool     s_rawfifo_subscribed;
static bool     s_sev_subscribed;
static bool     s_batt_subscribed;
static uint32_t s_summary_sent_upto;   /* mayor seq enviado en esta conexion */
static bool     s_streaming = true;     /* stream SAMPLE */
static bool     s_raw_streaming = true; /* stream RAW (0x07) */
static uint8_t  s_own_addr_type;
static uint16_t s_att_payload = 20;     /* MTU-3 negociado; 20 = por defecto (MTU 23) */

/* ---- false solo mientras ble_stream_deinit() apaga el stack (link_down) ---- */
static volatile bool    s_enabled = true;
static bool             s_running;      /* stack NimBLE arrancado */
static struct ble_npl_event s_link_down_ev;

/* ---- robustez de notify() ---- */
#define BLE_DROP_LOG_PERIOD_US   (5 * 1000000LL)  /* resumen de descartes cada 5 s */
#define BLE_ENOMEM_HOLD_US       (2 * 1000000LL)  /* SAMPLE/RAW a la mitad 2 s tras un ENOMEM */
static uint32_t s_notify_drops;         /* notificaciones fallidas desde el ultimo resumen */
static uint32_t s_notify_enomem;        /* de esas, por falta de buffers (BLE_HS_ENOMEM) */
static int64_t  s_last_enomem_us = -BLE_ENOMEM_HOLD_US;
static int64_t  s_last_drop_log_us;
static uint32_t s_sample_thin_cnt;      /* contador para mandar SAMPLE 1 de cada 2 */
static uint32_t s_raw_thin_cnt;         /* contador para mandar RAW 1 de cada 2 (ENOMEM) */
static uint32_t s_raw_mtu_cnt;          /* contador para RAW a 25 Hz con MTU 23 */

/* ---- STATUS nunca se pierde ---- */
#define BLE_STATUS_TIMER_US      (250 * 1000)     /* periodo del timer de reenvio */
#define BLE_STATUS_MAX_GAP_US    (1000 * 1000LL)  /* STATUS al menos 1 vez por segundo */
#define BLE_DEINIT_WAIT_MS       2000             /* espera maxima a que cierre la conexion */
static volatile bool s_status_pending = false;
static int64_t       s_status_last_us = 0;
static uint32_t      s_status_retries;          /* reenvios por fallo desde el ultimo resumen */
static esp_timer_handle_t s_status_timer;

static ble_sample_t s_last_sample;
static ble_status_t s_last_status;
static ble_ext_t    s_last_ext;
static ble_raw_t    s_last_raw;
static ble_sev_t    s_last_sev;
/* BATT (0x09), 1 byte. No hay medicion de bateria en el hardware, asi que
 * solo existen dos valores:
 *   BLE_BATT_UNKNOWN     (0xFE): nivel desconocido; es lo que devuelve READ
 *                                siempre en funcionamiento normal.
 *   BLE_BATT_CHARGE_MODE (0xFF): se notifica una sola vez, justo antes de
 *                                entrar en modo carga. */
static uint8_t      s_last_batt = BLE_BATT_UNKNOWN;
static uint8_t       s_rawfifo_pkt[sizeof(ble_rawfifo_hdr_t) +
                                   BLE_RAWFIFO_MAX_SAMPLES_PKT * sizeof(bmi270_raw_sample_t)];

/* ---- INFO (comando 0x30 por CMD): version, ODR, rangos y factores de escala ---- */
#define BLE_FW_VERSION        "2.4.0"
#define BLE_INFO_ACC_RANGE_G  4
#define BLE_INFO_GYR_RANGE_DPS 500

static char     s_info_text[128];
static uint16_t s_info_len;

static void flush_summaries(void);
static int  notify(uint16_t handle, const void *data, uint16_t len);
static void send_status(void);
static void status_timer_cb(void *arg);
static void start_advertising(void);
static void link_down_ev(struct ble_npl_event *ev);
static void link_down(void);

/* ------------------------------------------------------------------ */
/* Acceso a caracteristicas                                          */
/* ------------------------------------------------------------------ */
static int gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)arg;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        if (attr_handle == s_sample_handle) {
            return os_mbuf_append(ctxt->om, &s_last_sample, sizeof(s_last_sample))
                       ? BLE_ATT_ERR_INSUFFICIENT_RES : 0;
        }
        if (attr_handle == s_status_handle) {
            return os_mbuf_append(ctxt->om, &s_last_status, sizeof(s_last_status))
                       ? BLE_ATT_ERR_INSUFFICIENT_RES : 0;
        }
        if (attr_handle == s_ext_handle) {
            return os_mbuf_append(ctxt->om, &s_last_ext, sizeof(s_last_ext))
                       ? BLE_ATT_ERR_INSUFFICIENT_RES : 0;
        }
        if (attr_handle == s_summary_handle) {
            hs_record_t rec;
            hs_iter_reset();
            if (hs_iter_next(&rec)) {   /* el mas antiguo pendiente */
                return os_mbuf_append(ctxt->om, &rec, sizeof(rec))
                           ? BLE_ATT_ERR_INSUFFICIENT_RES : 0;
            }
            return 0;   /* nada pendiente -> longitud 0 */
        }
        if (attr_handle == s_raw_handle) {
            return os_mbuf_append(ctxt->om, &s_last_raw, sizeof(s_last_raw))
                       ? BLE_ATT_ERR_INSUFFICIENT_RES : 0;
        }
        if (attr_handle == s_sev_handle) {
            return os_mbuf_append(ctxt->om, &s_last_sev, sizeof(s_last_sev))
                       ? BLE_ATT_ERR_INSUFFICIENT_RES : 0;
        }
        if (attr_handle == s_batt_handle) {
            return os_mbuf_append(ctxt->om, &s_last_batt, sizeof(s_last_batt))
                       ? BLE_ATT_ERR_INSUFFICIENT_RES : 0;
        }
        if (attr_handle == s_cmd_handle) {
            /* respuesta a INFO por READ ademas de por NOTIFY: si el cliente
             * aun no nego un MTU grande, READ fragmenta solo (ATT_READ_BLOB),
             * a diferencia de un NOTIFY que se trunca al MTU actual. */
            return os_mbuf_append(ctxt->om, s_info_text, s_info_len)
                       ? BLE_ATT_ERR_INSUFFICIENT_RES : 0;
        }
        return BLE_ATT_ERR_UNLIKELY;

    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        uint8_t buf[5] = { 0xFF, 0, 0, 0, 0 };
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len > sizeof(buf)) {
            len = sizeof(buf);
        }
        if (len >= 1) {
            ble_hs_mbuf_to_flat(ctxt->om, buf, len, NULL);
        }
        uint8_t cmd = buf[0];
        uint32_t arg = (uint32_t)buf[1] | ((uint32_t)buf[2] << 8) |
                       ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 24);

        if (cmd == 0x00) {
            s_streaming = false;
            ESP_LOGI(TAG, "stream SAMPLE pausado por el cliente");
        } else if (cmd == 0x01) {
            s_streaming = true;
            ESP_LOGI(TAG, "stream SAMPLE reanudado por el cliente");
        } else if (cmd == 0x02) {
            s_raw_streaming = false;
            ESP_LOGI(TAG, "stream RAW pausado por el cliente");
        } else if (cmd == 0x03) {
            s_raw_streaming = true;
            ESP_LOGI(TAG, "stream RAW reanudado por el cliente");
        } else if (cmd == 0x10 && len >= 5) {
            hs_set_epoch(arg);                 /* sincronizacion de hora */
            /* los resumenes que estaban sin datar ya tienen epoch: reenviarlos */
            s_summary_sent_upto = 0;
            flush_summaries();
        } else if (cmd == 0x20 && len >= 5) {
            hs_ack(arg);                       /* ack de resumenes hasta 'arg' */
        } else if (cmd == 0x30) {
            /* INFO: version, ODR, rangos y factores de escala. Se manda por
             * NOTIFY (si hay suscripcion) y ademas queda listo para READ. */
            ESP_LOGI(TAG, "INFO solicitada por el cliente");
            notify(s_cmd_handle, s_info_text, s_info_len);
        } else if (cmd == BLE_CMD_CHARGE_MODE) {
            /* la secuencia NO se ejecuta aqui (callback de NimBLE): solo se
             * avisa a la tarea "charge", que es la que la hace */
            charge_mode_request();
        }
        return 0;
    }

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = WATCH_UUID128(0x01),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid       = WATCH_UUID128(0x02),   /* SAMPLE */
                .access_cb  = gatt_access,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_sample_handle,
            },
            {
                .uuid       = WATCH_UUID128(0x03),   /* STATUS */
                .access_cb  = gatt_access,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_status_handle,
            },
            {
                .uuid       = WATCH_UUID128(0x04),   /* CMD (+ respuesta a INFO) */
                .access_cb  = gatt_access,
                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_cmd_handle,
            },
            {
                .uuid       = WATCH_UUID128(0x05),   /* EXT (rama acelerometro) */
                .access_cb  = gatt_access,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_ext_handle,
            },
            {
                .uuid       = WATCH_UUID128(0x06),   /* SUMMARY (resumenes horarios) */
                .access_cb  = gatt_access,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_summary_handle,
            },
            {
                .uuid       = WATCH_UUID128(0x07),   /* RAW (IMU en crudo -> Firebase) */
                .access_cb  = gatt_access,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_raw_handle,
            },
            {
                .uuid       = WATCH_UUID128(0x08),   /* RAWFIFO: lotes con cabecera, LSB crudos */
                .access_cb  = gatt_access,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_rawfifo_handle,
            },
            {
                .uuid       = WATCH_UUID128(0x0A),   /* SEV (severidad, escala MDS-UPDRS) */
                .access_cb  = gatt_access,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_sev_handle,
            },
            {
                .uuid       = WATCH_UUID128(0x09),   /* BATT (bateria / aviso de modo carga) */
                .access_cb  = gatt_access,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_batt_handle,
            },
            { 0 }
        },
    },
    { 0 }
};

/* ------------------------------------------------------------------ */
/* GAP                                                               */
/* ------------------------------------------------------------------ */
/* significado de los motivos de desconexion mas comunes (HCI + 0x200) */
static const char *disconnect_reason_str(int reason)
{
    switch (reason) {
    case 0x208: return "supervision timeout (senal perdida)";
    case 0x213: return "el celular cerro la conexion";
    case 0x216: return "la cerro el reloj";
    case 0x23E: return "fallo al establecer la conexion";
    default:    return "otro motivo";
    }
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "cliente conectado (handle %d)", s_conn_handle);

            /* conexion que ya venia en camino cuando se empezo a apagar el BLE */
            if (!s_enabled) {
                ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                return 0;
            }

            /* parametros de conexion: 30-50 ms, sin latencia, supervision 6 s */
            struct ble_gap_upd_params up = {
                .itvl_min            = 24,    /* 30 ms  (x1.25 ms) */
                .itvl_max            = 40,    /* 50 ms  (x1.25 ms) */
                .latency             = 0,
                .supervision_timeout = 600,   /* 6 s    (x10 ms) */
                .min_ce_len          = 0,
                .max_ce_len          = 0,
            };
            int urc = ble_gap_update_params(s_conn_handle, &up);
            if (urc == 0) {
                ESP_LOGI(TAG, "pedidos parametros de conexion 30-50 ms / lat 0 / timeout 6 s");
            } else {
                ESP_LOGW(TAG, "ble_gap_update_params fallo (rc=%d), se sigue con los del celular", urc);
            }

            /* intercambio de MTU iniciado por el reloj */
            int mrc = ble_gattc_exchange_mtu(s_conn_handle, NULL, NULL);
            if (mrc != 0) {
                ESP_LOGW(TAG, "ble_gattc_exchange_mtu fallo (rc=%d)", mrc);
            }
        } else {
            ESP_LOGW(TAG, "conexion fallida (status %d), re-publicitando",
                     event->connect.status);
            start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "cliente desconectado (reason 0x%03X: %s)",
                 (unsigned)event->disconnect.reason,
                 disconnect_reason_str(event->disconnect.reason));
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_sample_subscribed = false;
        s_status_subscribed = false;
        s_ext_subscribed = false;
        s_summary_subscribed = false;
        s_raw_subscribed = false;
        s_rawfifo_subscribed = false;
        s_sev_subscribed = false;
        s_batt_subscribed = false;
        s_summary_sent_upto = 0;
        s_att_payload = 20;   /* vuelve al MTU por defecto hasta la proxima negociacion */
        start_advertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_sample_handle) {
            s_sample_subscribed = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == s_status_handle) {
            s_status_subscribed = event->subscribe.cur_notify;
            if (s_status_subscribed) {
                send_status();   /* la app recibe el ultimo STATUS de inmediato */
            }
        } else if (event->subscribe.attr_handle == s_ext_handle) {
            s_ext_subscribed = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == s_summary_handle) {
            s_summary_subscribed = event->subscribe.cur_notify;
            if (s_summary_subscribed) {
                flush_summaries();   /* vacia lo pendiente al suscribirse */
            }
        } else if (event->subscribe.attr_handle == s_raw_handle) {
            s_raw_subscribed = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == s_rawfifo_handle) {
            s_rawfifo_subscribed = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == s_sev_handle) {
            s_sev_subscribed = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == s_batt_handle) {
            s_batt_subscribed = event->subscribe.cur_notify;
        }
        ESP_LOGI(TAG, "subscribe: sample=%d status=%d ext=%d summary=%d raw=%d rawfifo=%d sev=%d",
                 s_sample_subscribed, s_status_subscribed, s_ext_subscribed,
                 s_summary_subscribed, s_raw_subscribed, s_rawfifo_subscribed,
                 s_sev_subscribed);
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE: {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) == 0) {
            ESP_LOGI(TAG, "conn update (status %d): intervalo %.2f ms, latencia %d, timeout %d ms",
                     event->conn_update.status,
                     desc.conn_itvl * 1.25f, desc.conn_latency,
                     desc.supervision_timeout * 10);
        } else {
            ESP_LOGW(TAG, "conn update (status %d): conexion no encontrada",
                     event->conn_update.status);
        }
        return 0;
    }

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU = %d", event->mtu.value);
        if (event->mtu.value > 3) {
            s_att_payload = (uint16_t)(event->mtu.value - 3);
        }
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        return 0;

    default:
        return 0;
    }
}

static void start_advertising(void)
{
    struct ble_hs_adv_fields fields;
    struct ble_hs_adv_fields rsp;
    struct ble_gap_adv_params advp;

    if (!s_enabled) {
        return;
    }

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    if (ble_gap_adv_set_fields(&fields) != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields fallo");
        return;
    }

    /* El UUID de 128 bits no cabe junto al nombre: va en el scan response */
    memset(&rsp, 0, sizeof(rsp));
    rsp.uuids128 = (ble_uuid128_t *)&s_svc_uuid;
    rsp.num_uuids128 = 1;
    rsp.uuids128_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&rsp);

    memset(&advp, 0, sizeof(advp));
    advp.conn_mode = BLE_GAP_CONN_MODE_UND;
    advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
    advp.itvl_min  = 160;   /* 100 ms (x0.625 ms): reconexion rapida */
    advp.itvl_max  = 160;

    int rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                               &advp, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start fallo (rc=%d)", rc);
    } else {
        ESP_LOGI(TAG, "publicitando como '%s'", DEVICE_NAME);
    }
}

/* ------------------------------------------------------------------ */
/* Arranque del host                                                 */
/* ------------------------------------------------------------------ */
static void on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto fallo (rc=%d)", rc);
        return;
    }

    /* potencia de TX al maximo del ESP32-C6 (ESP_PWR_LVL_P20 = +20 dBm) */
    const esp_ble_power_type_t pwr_types[] = {
        ESP_BLE_PWR_TYPE_DEFAULT, ESP_BLE_PWR_TYPE_ADV, ESP_BLE_PWR_TYPE_CONN_HDL0,
    };
    const char *pwr_names[] = { "DEFAULT", "ADV", "CONN_HDL0" };
    for (size_t i = 0; i < sizeof(pwr_types) / sizeof(pwr_types[0]); i++) {
        esp_err_t perr = esp_ble_tx_power_set(pwr_types[i], ESP_PWR_LVL_P20);
        if (perr == ESP_OK) {
            ESP_LOGI(TAG, "TX power %s = +20 dBm (nivel %d)", pwr_names[i],
                     (int)esp_ble_tx_power_get(pwr_types[i]));
        } else {
            ESP_LOGW(TAG, "TX power %s: %s", pwr_names[i], esp_err_to_name(perr));
        }
    }

    start_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "stack BLE reiniciado, reason=%d", reason);
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();                 /* bloquea hasta nimble_port_stop() */
    nimble_port_freertos_deinit();
}

void ble_stream_init(void)
{
    if (s_running) {
        return;
    }
    s_enabled = true;

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init fallo: %s", esp_err_to_name(err));
        return;
    }

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    /* Aceptamos un MTU grande por si la app lo negocia (permite subir el
     * ritmo de SAMPLE sin fragmentar). Con el MTU por defecto (23) tambien
     * funciona: los frames son de 14 bytes. */
    ble_att_set_preferred_mtu(247);

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc == 0) {
        rc = ble_gatts_add_svcs(gatt_svcs);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "registro GATT fallo (rc=%d)", rc);
        return;
    }

    ble_svc_gap_device_name_set(DEVICE_NAME);
    ble_store_config_init();

    /* INFO (respuesta al comando 0x30): se calcula una vez, con los mismos
     * valores fijos que configura bmi270_sensor_init(). Formato clave=valor
     * separado por ';' en texto plano, como pidio el equipo de la app. */
    float lsb2ms2 = BLE_INFO_ACC_RANGE_G * 9.80665f / 32768.0f;
    float lsb2dps = (float)BLE_INFO_GYR_RANGE_DPS / 32768.0f;
    int ilen = snprintf(s_info_text, sizeof(s_info_text),
                        "FW=%s;ODR=%d;ACC_RANGE_G=%d;GYR_RANGE_DPS=%d;LSB2MS2=%.8f;LSB2DPS=%.8f",
                        BLE_FW_VERSION, BLE_RAWFIFO_ODR_HZ, BLE_INFO_ACC_RANGE_G,
                        BLE_INFO_GYR_RANGE_DPS, lsb2ms2, lsb2dps);
    s_info_len = (ilen > 0 && ilen < (int)sizeof(s_info_text)) ? (uint16_t)ilen : 0;
    ESP_LOGI(TAG, "INFO: %s", s_info_text);

    /* reenvio de STATUS: si fallo el ultimo o pasa >1 s sin mandarlo */
    esp_err_t terr = ESP_OK;
    if (!s_status_timer) {     /* se crea una sola vez; ble_stream_deinit() solo lo para */
        const esp_timer_create_args_t targs = {
            .callback = status_timer_cb,
            .name     = "ble_status",
        };
        terr = esp_timer_create(&targs, &s_status_timer);
    }
    if (terr == ESP_OK && !esp_timer_is_active(s_status_timer)) {
        terr = esp_timer_start_periodic(s_status_timer, BLE_STATUS_TIMER_US);
    }
    if (terr != ESP_OK) {
        ESP_LOGE(TAG, "timer de STATUS fallo: %s", esp_err_to_name(terr));
    }

    ble_npl_event_init(&s_link_down_ev, link_down_ev, NULL);

    s_last_batt = BLE_BATT_UNKNOWN;
    s_running = true;
    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "BLE inicializado");
}

void ble_stream_stop_data(void)
{
    if (s_status_timer) {
        esp_timer_stop(s_status_timer);   /* sin reenvios de STATUS */
    }
    s_status_pending = false;
}

void ble_stream_deinit(void)
{
    if (!s_running) {
        return;
    }

    /* 1) desconexion ordenada y fin del advertising, en la tarea de NimBLE */
    link_down();
    for (int t = 0; t < BLE_DEINIT_WAIT_MS &&
                    (s_conn_handle != BLE_HS_CONN_HANDLE_NONE || ble_gap_adv_active());
         t += 20) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "la conexion no cerro en %d ms: se apaga el stack igual",
                 BLE_DEINIT_WAIT_MS);
    }
    ble_stream_stop_data();

    /* 2) host y controlador fuera: la radio queda apagada del todo y nada
     *    del BLE queda pendiente durante el light sleep */
    int rc = nimble_port_stop();
    if (rc != 0) {
        ESP_LOGE(TAG, "nimble_port_stop fallo (rc=%d): el stack BLE sigue arriba", rc);
        return;
    }
    ble_npl_event_deinit(&s_link_down_ev);
    esp_err_t err = nimble_port_deinit();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_deinit fallo: %s", esp_err_to_name(err));
    }

    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    s_sample_subscribed = false;
    s_status_subscribed = false;
    s_ext_subscribed = false;
    s_summary_subscribed = false;
    s_raw_subscribed = false;
    s_rawfifo_subscribed = false;
    s_sev_subscribed = false;
    s_batt_subscribed = false;
    s_summary_sent_upto = 0;
    s_att_payload = 20;
    s_running = false;
    ESP_LOGI(TAG, "BLE apagado (host + controlador)");
}

void ble_stream_push_battery(uint8_t value)
{
    s_last_batt = value;
    if (s_batt_subscribed) {
        notify(s_batt_handle, &s_last_batt, sizeof(s_last_batt));
    }
}

/* Corre en la tarea de NimBLE, igual que gap_event: asi no compite con el
 * re-publicitado que hace gap_event al desconectarse. */
static void link_down_ev(struct ble_npl_event *ev)
{
    (void)ev;
    if (!ble_hs_synced()) {
        return;                 /* on_sync ya respeta s_enabled */
    }
    if (ble_gap_adv_active()) {
        ble_gap_adv_stop();
    }
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

/* Desconecta al cliente y deja de publicitar (no se vuelve a publicitar
 * hasta el proximo ble_stream_init()). */
static void link_down(void)
{
    s_enabled = false;
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_link_down_ev);
}

/* ------------------------------------------------------------------ */
/* API de publicacion                                                */
/* ------------------------------------------------------------------ */
bool ble_stream_connected(void)
{
    return s_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}

bool ble_stream_streaming(void)
{
    return s_streaming;
}

static inline int16_t clamp16(float v)
{
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)lrintf(v);
}

static inline uint8_t clamp_pct(float v)
{
    if (v < 0.0f)   return 0;
    if (v > 100.0f) return 100;
    return (uint8_t)lrintf(v);
}

static inline uint8_t clamp_u8(float v)
{
    if (v < 0.0f)   return 0;
    if (v > 255.0f) return 255;
    return (uint8_t)lrintf(v);
}

static inline uint16_t clamp_u16(float v)
{
    if (v < 0.0f)     return 0;
    if (v > 65535.0f) return 65535;
    return (uint16_t)lrintf(v);
}

/* registra un fallo de notificacion (ENOMEM activa el modo "mitad") */
static void note_notify_fail(int rc)
{
    s_notify_drops++;
    if (rc == BLE_HS_ENOMEM) {
        s_notify_enomem++;
        s_last_enomem_us = esp_timer_get_time();
    }
}

/* Devuelve 0 si se encolo la notificacion, o el codigo de NimBLE si no.
 * (ble_gatts_notify_custom consume el mbuf aunque falle.) */
static int notify(uint16_t handle, const void *data, uint16_t len)
{
    if (!s_enabled || s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return BLE_HS_ENOTCONN;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) {
        note_notify_fail(BLE_HS_ENOMEM);   /* sin mbufs: mismo caso que ENOMEM */
        return BLE_HS_ENOMEM;
    }
    int rc = ble_gatts_notify_custom(s_conn_handle, handle, om);
    if (rc != 0) {
        note_notify_fail(rc);
    }
    return rc;
}

/* Envia s_last_status si hay conexion y suscripcion. Si se encola, marca la
 * hora y limpia el pendiente; si falla, queda pendiente para el timer. */
static void send_status(void)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_status_subscribed) {
        return;
    }
    if (notify(s_status_handle, &s_last_status, sizeof(s_last_status)) == 0) {
        s_status_last_us = esp_timer_get_time();
        s_status_pending = false;
    } else {
        s_status_pending = true;
    }
}

/* timer de 250 ms: reenvia STATUS si quedo pendiente o si paso >1 s */
static void status_timer_cb(void *arg)
{
    (void)arg;
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_status_subscribed) {
        return;
    }
    bool retry = s_status_pending;
    if (retry || (esp_timer_get_time() - s_status_last_us) > BLE_STATUS_MAX_GAP_US) {
        if (retry) {
            s_status_retries++;
        }
        send_status();
    }
}

/* true si en los ultimos 2 s hubo ENOMEM: SAMPLE y RAW van a la mitad */
static bool congested(void)
{
    return (esp_timer_get_time() - s_last_enomem_us) < BLE_ENOMEM_HOLD_US;
}

/* resumen de descartes cada 5 s, solo si los hubo (se llama desde el tick
 * de 2 Hz de ble_stream_push_status) */
static void report_drops(void)
{
    int64_t now = esp_timer_get_time();
    if (now - s_last_drop_log_us < BLE_DROP_LOG_PERIOD_US) {
        return;
    }
    s_last_drop_log_us = now;
    if (s_notify_drops > 0) {
        ESP_LOGW(TAG, "BLE: %u notificaciones descartadas (STATUS reintentado %u veces)",
                 (unsigned)s_notify_drops, (unsigned)s_status_retries);
        s_notify_drops   = 0;
        s_notify_enomem  = 0;
        s_status_retries = 0;
    }
}

/* Envia por SUMMARY los resumenes pendientes que aun no se mandaron en esta
 * conexion. Se reintenta cada 0.5 s (desde ble_stream_push_status) hasta que
 * la app confirma con CMD 0x20; al reconectar se reenvia lo no confirmado. */
static void flush_summaries(void)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_summary_subscribed) {
        return;
    }
    hs_record_t rec;
    hs_iter_reset();
    int sent = 0;
    while (sent < 8 && hs_iter_next(&rec)) {   /* como mucho 8 por tick */
        if (rec.seq > s_summary_sent_upto) {
            notify(s_summary_handle, &rec, sizeof(rec));
            s_summary_sent_upto = rec.seq;
            sent++;
        }
    }
}

void ble_stream_push_sample(uint32_t t_ms,
                            float roll_deg, float pitch_deg,
                            float lin_acc_g, float gyro_mag_dps,
                            float tremor_amp_dps)
{
    s_last_sample.t_ms            = t_ms;
    s_last_sample.roll_cdeg       = clamp16(roll_deg  * 100.0f);
    s_last_sample.pitch_cdeg      = clamp16(pitch_deg * 100.0f);
    s_last_sample.lin_acc_mg      = clamp16(lin_acc_g * 1000.0f);
    s_last_sample.gyro_mag_ddps   = clamp16(gyro_mag_dps * 10.0f);
    s_last_sample.tremor_amp_ddps = clamp16(tremor_amp_dps * 10.0f);

    if (s_streaming && s_sample_subscribed) {
        /* con ENOMEM reciente, solo 1 de cada 2 */
        if (!congested() || (s_sample_thin_cnt++ & 1u) == 0u) {
            notify(s_sample_handle, &s_last_sample, sizeof(s_last_sample));
        }
    }
}

bool ble_stream_raw_streaming(void)
{
    return s_raw_streaming;
}

void ble_stream_push_raw(uint32_t t_ms,
                         float ax_g, float ay_g, float az_g,
                         float gx_dps, float gy_dps, float gz_dps)
{
    s_last_raw.t_ms    = t_ms;
    s_last_raw.ax_mg   = clamp16(ax_g * 1000.0f);
    s_last_raw.ay_mg   = clamp16(ay_g * 1000.0f);
    s_last_raw.az_mg   = clamp16(az_g * 1000.0f);
    s_last_raw.gx_ddps = clamp16(gx_dps * 10.0f);
    s_last_raw.gy_ddps = clamp16(gy_dps * 10.0f);
    s_last_raw.gz_ddps = clamp16(gz_dps * 10.0f);

    if (s_raw_streaming && s_raw_subscribed) {
        /* MTU 23 (payload 20): RAW a 25 Hz en vez de 50 Hz */
        if (s_att_payload == 20 && (s_raw_mtu_cnt++ & 1u) != 0u) {
            return;
        }
        /* con ENOMEM reciente, solo 1 de cada 2 */
        if (!congested() || (s_raw_thin_cnt++ & 1u) == 0u) {
            notify(s_raw_handle, &s_last_raw, sizeof(s_last_raw));
        }
    }
}

/* Trocea un lote de muestras crudas (LSB, ya sin perdidas/duplicados gracias
 * a la FIFO) en tantas notificaciones de 0x08 como haga falta segun el MTU
 * negociado, cada una con su propia cabecera (seq_primera, t0_us, n_muestras).
 * 't0_us' es el instante (esp_timer_get_time) de samples[0]; como el ODR es
 * fijo (BLE_RAWFIFO_ODR_HZ), el resto de sub-paquetes calculan su propio t0
 * sumando 'offset * periodo' sin necesitar mas informacion de temporizacion.
 * Devuelve cuantas notificaciones se enviaron de verdad (0 si no hay cliente
 * suscrito a 0x08: en ese caso no se manda nada, tal como se pidio). */
int ble_stream_push_raw_batch(uint16_t seq_primera, uint32_t t0_us,
                              const bmi270_raw_sample_t *samples, int n)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_rawfifo_subscribed || n <= 0) {
        return 0;
    }

    int max_samples = ((int)s_att_payload - (int)sizeof(ble_rawfifo_hdr_t)) /
                      (int)sizeof(bmi270_raw_sample_t);
    if (max_samples < 1) {
        max_samples = 1;   /* con MTU 23 (payload 20) ya caben 1 muestra + cabecera */
    }
    if (max_samples > BLE_RAWFIFO_MAX_SAMPLES_PKT) {
        max_samples = BLE_RAWFIFO_MAX_SAMPLES_PKT;
    }

    int sent = 0, i = 0;
    while (i < n) {
        int k = (n - i < max_samples) ? (n - i) : max_samples;

        ble_rawfifo_hdr_t hdr;
        hdr.seq_primera = (uint16_t)(seq_primera + i);
        hdr.t0_us       = (uint32_t)(t0_us + (uint32_t)i * BLE_RAWFIFO_PERIOD_US);
        hdr.n_muestras  = (uint8_t)k;

        memcpy(s_rawfifo_pkt, &hdr, sizeof(hdr));
        memcpy(s_rawfifo_pkt + sizeof(hdr), &samples[i], (size_t)k * sizeof(bmi270_raw_sample_t));

        notify(s_rawfifo_handle, s_rawfifo_pkt, (uint16_t)(sizeof(hdr) +
              (size_t)k * sizeof(bmi270_raw_sample_t)));
        sent++;
        i += k;
    }
    return sent;
}

void ble_stream_push_status(uint32_t t_ms, const tremor_result_t *res)
{
    uint8_t flags = 0;
    if (res->tremor_present)  flags |= BLE_STATUS_FLAG_TREMOR;
    if (res->freq_stable)     flags |= BLE_STATUS_FLAG_FREQ_OK;
    if (res->tremor_detected) flags |= BLE_STATUS_FLAG_DETECTED;
    flags |= ((uint8_t)res->category << BLE_STATUS_CLASS_SHIFT) & BLE_STATUS_CLASS_MASK;
    if (res->analizando)      flags |= BLE_STATUS_FLAG_ANALIZANDO;

    s_last_status.t_ms            = t_ms;
    s_last_status.flags           = flags;
    s_last_status.dom_freq_ddhz   = clamp_u8(res->dominant_freq_hz * 10.0f);
    s_last_status.tremor_amp_ddps = clamp16(res->tremor_amp_dps * 10.0f);
    s_last_status.movement_ddps   = clamp16(res->movement_dps * 10.0f);
    s_last_status.band_frac_pct   = clamp_pct(res->band_fraction * 100.0f);
    s_last_status.duty_pct        = clamp_pct(res->duty_last_min * 100.0f);
    s_last_status.episode_s       = (uint16_t)(res->episode_ms / 1000u);

    send_status();   /* si falla queda pendiente y lo reenvia el timer */

    /* rama del acelerometro (caracteristica EXT, aditiva) */
    s_last_ext.t_ms            = t_ms;
    s_last_ext.accel_amp_mg    = clamp_u16(res->accel_amp_mg);
    s_last_ext.accel_gyro_x100 = clamp_u16(res->accel_gyro_ratio * 100.0f);
    s_last_ext.ang_disp_cdeg   = clamp_u16(res->ang_disp_deg * 100.0f);
    s_last_ext.flags2          = res->rotational ? BLE_EXT_FLAG_ROTATIONAL : 0;
    s_last_ext.category        = (uint8_t)res->category;

    if (s_ext_subscribed) {
        notify(s_ext_handle, &s_last_ext, sizeof(s_last_ext));
    }

    /* severidad (caracteristica SEV, aditiva, escala MDS-UPDRS): no se metio
     * en EXT para no cambiarle el tamaño */
    s_last_sev.t_ms       = t_ms;
    s_last_sev.theta_cdeg = clamp_u16(res->theta_cal_deg * 100.0f);
    s_last_sev.desplaz_mm = clamp_u16(res->desplaz_cm * 10.0f);

    if (s_sev_subscribed) {
        notify(s_sev_handle, &s_last_sev, sizeof(s_last_sev));
    }

    /* aprovecha el tick de 2 Hz para ir vaciando resumenes horarios pendientes */
    flush_summaries();

    report_drops();
}
