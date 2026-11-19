/*
 * bmi270_esp32.c
 * -----------------------------------------------------------------------
 * "Pegamento" entre el driver oficial de Bosch (bmi2.c / bmi270.c) y el
 * bus I2C del ESP32-C6, usando el driver nuevo de ESP-IDF (driver/i2c_master.h).
 *
 * El driver de Bosch no sabe nada de ESP32: solo necesita que le demos
 * 3 funciones (leer registro, escribir registro, esperar microsegundos)
 * con la firma exacta que él espera. Eso es lo que hace este archivo.
 * -----------------------------------------------------------------------
 */

#include <string.h>
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#include "bmi2.h"
#include "bmi270.h"
#include "bmi270_esp32.h"

static const char *TAG = "BMI270_ESP32";

// ---- TODO: cambia estos pines por los que realmente usaste ----
#define BMI270_SDA_GPIO   6
#define BMI270_SCL_GPIO   7
// -----------------------------------------------------------------

// TODO: si el pin SDO del BMI270 está a VDD en vez de GND, cambia esto
// a BMI2_I2C_SEC_ADDR (0x69).
#define BMI270_I2C_ADDR   BMI2_I2C_PRIM_ADDR // 0x68

// Tamano de bloque para lectura/escritura. La subida del archivo de config
// del BMI270 (~8 KB) se hace en trozos de este tamano; mas grande = menos
// transacciones = mas fiable. Debe ser par.
#define BMI270_MAX_WRITE_LEN 128

// Velocidad del bus I2C. Empezamos conservador (100 kHz) porque con los
// pull-ups internos del ESP32 (~45 kohm) 400 kHz suele corromper la subida
// del firmware. Si con pull-ups externos de 4.7 kohm va bien, se puede subir.
#define BMI270_I2C_SPEED_HZ  100000

static struct bmi2_dev s_bmi;
static i2c_master_dev_handle_t s_i2c_dev;
static bool    s_ready;        // bmi270_sensor_init() termino bien
static bool    s_suspended;    // en suspend (modo carga)
static uint8_t s_aps_before;   // advanced power save antes de suspender

// ------- Funciones "glue" que el driver de Bosch necesita -------

// OJO: en el driver i2c_master de ESP-IDF >=5.2 el ultimo parametro es el
// timeout EN MILISEGUNDOS (no en ticks). Usar pdMS_TO_TICKS() aqui da un
// timeout diminuto y la subida del firmware del BMI270 (~8 KB) falla con
// COM_FAIL. 1000 ms es holgado para cualquier transaccion.
#define BMI270_I2C_TIMEOUT_MS 1000

static BMI2_INTF_RETURN_TYPE bmi270_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
  uint32_t len, void *intf_ptr)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)intf_ptr;
    esp_err_t err = i2c_master_transmit_receive(dev, &reg_addr, 1, reg_data, len,
                                                 BMI270_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "READ  reg=0x%02X len=%u -> %s", reg_addr, (unsigned)len,
                 esp_err_to_name(err));
        return BMI2_E_COM_FAIL;
    }
    return BMI2_INTF_RET_SUCCESS;
}

static BMI2_INTF_RETURN_TYPE bmi270_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                               uint32_t len, void *intf_ptr)
{
    if (len > BMI270_MAX_WRITE_LEN) {
        ESP_LOGE(TAG, "WRITE reg=0x%02X len=%u excede el buffer (%d)", reg_addr,
                 (unsigned)len, BMI270_MAX_WRITE_LEN);
        return BMI2_E_OUT_OF_RANGE;
    }
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)intf_ptr;
    uint8_t buf[BMI270_MAX_WRITE_LEN + 1];
    buf[0] = reg_addr;
    memcpy(&buf[1], reg_data, len);

    esp_err_t err = i2c_master_transmit(dev, buf, len + 1, BMI270_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WRITE reg=0x%02X len=%u -> %s", reg_addr, (unsigned)len,
                 esp_err_to_name(err));
        return BMI2_E_COM_FAIL;
    }
    return BMI2_INTF_RET_SUCCESS;
}

static void bmi270_delay_us(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;
    // CONFIG_FREERTOS_HZ=100 => 1 tick = 10 ms, asi que vTaskDelay()/
    // pdMS_TO_TICKS() redondean a 0 cualquier espera < 10 ms y NO esperan
    // nada. El arranque del BMI270 necesita esperas de ~2 ms (settle tras
    // el soft-reset) y de pocos ms durante la carga del firmware; si no se
    // respetan, las siguientes transacciones dan NACK -> COM_FAIL (rslt=-2).
    // esp_rom_delay_us() es busy-wait y preciso: lo usamos para todo lo
    // corto (el driver no pide esperas largas en el uso normal).
    if (period_us <= 10000) {
        esp_rom_delay_us(period_us);
    } else {
        vTaskDelay((period_us / 1000) / portTICK_PERIOD_MS + 1);
    }
}

// ------- Conversión de cuentas crudas (LSB) a unidades físicas -------
// Misma fórmula que usa Bosch en sus ejemplos oficiales.

static float lsb_to_g(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = (float)(1 << (bit_width - 1));
    return (val * g_range) / half_scale;
}

static float lsb_to_dps(int16_t val, float dps_range, uint8_t bit_width)
{
    float half_scale = (float)(1 << (bit_width - 1));
    return (val * dps_range) / half_scale;
}

// ------- Inicialización -------

esp_err_t bmi270_sensor_init(void)
{
    ESP_LOGW(TAG, "==== glue BMI270 build v5 -- FIFO (timeout=%d ms, rw_len=%d, %d Hz I2C) ====",
             BMI270_I2C_TIMEOUT_MS, BMI270_MAX_WRITE_LEN, BMI270_I2C_SPEED_HZ);

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = BMI270_SDA_GPIO,
        .scl_io_num = BMI270_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus fallo: %s", esp_err_to_name(err));
        return err;
    }

    // ---- Escaneo del bus I2C: ayuda a diagnosticar cableado/direccion ----
    ESP_LOGI(TAG, "Escaneando bus I2C (SDA=%d, SCL=%d)...", BMI270_SDA_GPIO, BMI270_SCL_GPIO);
    int encontrados = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(bus_handle, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "  -> dispositivo en 0x%02X", addr);
            encontrados++;
        }
    }
    if (encontrados == 0) {
        ESP_LOGE(TAG, "  Ningun dispositivo I2C responde. Revisa VDD/VDDIO=3V3, CS a 3V3, "
                      "SDA/SCL (no cruzados) y GND comun.");
    }
    // ---------------------------------------------------------------------

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMI270_I2C_ADDR,
        .scl_speed_hz = BMI270_I2C_SPEED_HZ,
    };
    err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &s_i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device fallo: %s", esp_err_to_name(err));
        return err;
    }

    // ---- Lectura directa del CHIP_ID (reg 0x00): debe devolver 0x24 ----
    // Nos dice si las transacciones basicas de registros funcionan, antes
    // de intentar la subida del firmware en bmi270_init().
    uint8_t chip_id_reg = 0x00;
    uint8_t chip_id = 0xFF;
    esp_err_t idr = i2c_master_transmit_receive(s_i2c_dev, &chip_id_reg, 1,
                                                &chip_id, 1, BMI270_I2C_TIMEOUT_MS);
    ESP_LOGI(TAG, "CHIP_ID directo: err=%s, valor=0x%02X (esperado 0x24)",
             esp_err_to_name(idr), chip_id);
    // ------------------------------------------------------------------

    memset(&s_bmi, 0, sizeof(s_bmi));
    s_bmi.intf = BMI2_I2C_INTF;
    s_bmi.read = bmi270_i2c_read;
    s_bmi.write = bmi270_i2c_write;
    s_bmi.delay_us = bmi270_delay_us;
    s_bmi.intf_ptr = s_i2c_dev;
    s_bmi.read_write_len = BMI270_MAX_WRITE_LEN;
    s_bmi.config_file_ptr = NULL; // usa el archivo de config por defecto

    int8_t rslt = bmi270_init(&s_bmi);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "bmi270_init fallo, rslt=%d (revisa cableado/direccion I2C)", rslt);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BMI270 inicializado correctamente");

    struct bmi2_sens_config config[2];
    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;

    rslt = bmi2_get_sensor_config(config, 2, &s_bmi);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "bmi2_get_sensor_config fallo, rslt=%d", rslt);
        return ESP_FAIL;
    }

    // ODR REAL del sensor: 50 Hz, fijo, en accel y gyro (requisito del
    // contrato BLE: frecuencia fija para poder reconstruir el tiempo de
    // cada muestra en la app a partir de seq + t0 + periodo).
    //
    // bwp/filter_perf se dejan en modo "Normal" (el mismo que ya usaba el
    // proyecto): es el modo que Bosch recomienda para uso general porque el
    // ancho de banda del filtro escala con el ODR -3dB en torno a ODR/4,
    // muy por debajo del Nyquist de 25 Hz -> margen de sobra contra aliasing
    // para el temblor (<=12 Hz) y para el resto de la señal de movimiento.
    config[0].cfg.acc.odr = BMI2_ACC_ODR_50HZ;
    config[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
    config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;         // promediado normal (4 muestras)
    config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;   // alto rendimiento

    config[1].cfg.gyr.odr = BMI2_GYR_ODR_50HZ;
    config[1].cfg.gyr.range = BMI2_GYR_RANGE_500;
    config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;         // modo normal
    config[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;   // alto rendimiento

    rslt = bmi2_set_sensor_config(config, 2, &s_bmi);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "bmi2_set_sensor_config fallo, rslt=%d", rslt);
        return ESP_FAIL;
    }

    uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi2_sensor_enable(sensor_list, 2, &s_bmi);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "bmi2_sensor_enable fallo, rslt=%d", rslt);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Accel+Gyro configurados y habilitados a 50Hz (ODR): "
                  "accel bwp=NORMAL_AVG4 filter_perf=OPT, gyro bwp=NORMAL noise/filter_perf=OPT");
    s_ready = true;
    return ESP_OK;
}

// ------- Suspend / resume (modo carga) -------
// Suspend del BMI270 = accel y gyro deshabilitados (PWR_CTRL) + advanced
// power save activo (PWR_CONF): unos pocos uA. Los registros de configuracion
// (ODR, rangos, FIFO) se conservan, asi que al volver basta con rehabilitar.

esp_err_t bmi270_sensor_suspend(void)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_suspended) {
        return ESP_OK;
    }

    s_aps_before = s_bmi.aps_status;

    uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    int8_t rslt = bmi2_sensor_disable(sensor_list, 2, &s_bmi);
    if (rslt == BMI2_OK) {
        rslt = bmi2_set_adv_power_save(BMI2_ENABLE, &s_bmi);
    }
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "suspend fallo, rslt=%d", rslt);
        return ESP_FAIL;
    }
    s_suspended = true;
    ESP_LOGI(TAG, "BMI270 en suspend");
    return ESP_OK;
}

esp_err_t bmi270_sensor_resume(void)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_suspended) {
        return ESP_OK;
    }

    int8_t rslt = bmi2_set_adv_power_save(s_aps_before, &s_bmi);
    if (rslt == BMI2_OK) {
        uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };
        rslt = bmi2_sensor_enable(sensor_list, 2, &s_bmi);
    }
    if (rslt == BMI2_OK) {
        // descarta lo que hubiera quedado en la FIFO: se arranca en limpio
        rslt = bmi2_set_command_register(BMI2_FIFO_FLUSH_CMD, &s_bmi);
    }
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "resume fallo, rslt=%d", rslt);
        return ESP_FAIL;
    }
    s_suspended = false;
    ESP_LOGI(TAG, "BMI270 reanudado (accel+gyro, FIFO vaciada)");
    return ESP_OK;
}

bool bmi270_sensor_read(bmi270_data_t *out)
{
    struct bmi2_sens_data sensor_data = { 0 };
    int8_t rslt = bmi2_get_sensor_data(&sensor_data, &s_bmi);

    if (rslt != BMI2_OK) {
        return false;   // solo fallamos ante error real de comunicacion
    }
    // No comprobamos DRDY. OJO: ahora que el ODR bajo a 50 Hz (igual que el
    // sondeo de esta funcion) ya NO hay margen de sobremuestreo como cuando
    // el ODR era 100 Hz; si el scheduler se retrasa un poco (jitter de
    // FreeRTOS/BLE/SPI de la pantalla) esta ruta puede devolver el mismo
    // dato dos veces de vez en cuando. Es tolerable para SAMPLE/STATUS/EXT/
    // temblor (no cambia la deteccion, solo un valor repetido ocasional) y
    // NO afecta a la característica 0x08: esa va por la FIFO del sensor
    // (bmi270_fifo_read), que cuenta cada muestra real sin duplicar ni
    // perder ninguna, independientemente del sondeo por software.

    out->ax = lsb_to_g(sensor_data.acc.x, 4.0f, s_bmi.resolution);
    out->ay = lsb_to_g(sensor_data.acc.y, 4.0f, s_bmi.resolution);
    out->az = lsb_to_g(sensor_data.acc.z, 4.0f, s_bmi.resolution);

    out->gx = lsb_to_dps(sensor_data.gyr.x, 500.0f, s_bmi.resolution);
    out->gy = lsb_to_dps(sensor_data.gyr.y, 500.0f, s_bmi.resolution);
    out->gz = lsb_to_dps(sensor_data.gyr.z, 500.0f, s_bmi.resolution);

    return true;
}

/* ===================================================================
 * FIFO: lectura sin perdidas ni duplicados para la caracteristica 0x08.
 *
 * A diferencia de bmi270_sensor_read() (que lee "el dato de ahora mismo"
 * de los registros directos y puede repetirse si se sondea muy seguido),
 * la FIFO acumula dentro del propio sensor CADA muestra real tomada al
 * ODR configurado, en orden, hasta que el firmware la retira. Si el
 * firmware tarda en venir a leerla y la FIFO se llena, el BMI270 descarta
 * las muestras mas antiguas y lleva la cuenta exacta de cuantas descarto
 * (skipped_frame_count) -> por eso esta via no pierde informacion sobre
 * la perdida, aunque pierda las muestras en si.
 * =================================================================== */

// ~40 frames (13 B/frame: 1 cabecera + 6 accel + 6 gyro) de margen; a 50 Hz
// y sondeando cada 20-30 ms se usan normalmente 1-2 frames por lectura.
#define BMI270_FIFO_BUF_BYTES   512
#define BMI270_FIFO_MAX_FRAMES  40

static uint8_t s_fifo_buf[BMI270_FIFO_BUF_BYTES];

esp_err_t bmi270_fifo_enable(void)
{
    // Vacia cualquier resto y arranca en limpio.
    int8_t rslt = bmi2_set_command_register(BMI2_FIFO_FLUSH_CMD, &s_bmi);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "FIFO flush fallo, rslt=%d", rslt);
        return ESP_FAIL;
    }

    // Header-mode + accel + gyro: cada frame de la FIFO trae un byte de
    // cabecera que dice que contiene, y accel+gyro llegan juntos en el
    // mismo frame porque comparten ODR. NO activamos BMI2_FIFO_STOP_ON_FULL:
    // queremos el modo "stream" (circular) de fabrica, que es el que hace
    // que el sensor lleve la cuenta de frames descartados en vez de
    // simplemente dejar de aceptar datos nuevos.
    rslt = bmi2_set_fifo_config(BMI2_FIFO_HEADER_EN | BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN,
                                BMI2_ENABLE, &s_bmi);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "FIFO config fallo, rslt=%d", rslt);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "FIFO habilitada (header+accel+gyro, buffer local=%d B, ~%d frames)",
             BMI270_FIFO_BUF_BYTES, BMI270_FIFO_MAX_FRAMES);
    return ESP_OK;
}

int bmi270_fifo_read(bmi270_raw_sample_t *out, int max_out, uint16_t *skipped_out)
{
    if (skipped_out) {
        *skipped_out = 0;
    }

    uint16_t avail = 0;
    int8_t rslt = bmi2_get_fifo_length(&avail, &s_bmi);
    if (rslt != BMI2_OK) {
        return -1;
    }
    if (avail == 0) {
        return 0;   // nada nuevo todavia, no es un error
    }
    if (avail > BMI270_FIFO_BUF_BYTES) {
        avail = BMI270_FIFO_BUF_BYTES;   // se completa en la siguiente lectura
    }

    struct bmi2_fifo_frame fifo = { 0 };
    fifo.data = s_fifo_buf;
    fifo.length = avail;

    rslt = bmi2_read_fifo_data(&fifo, &s_bmi);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "FIFO read fallo, rslt=%d", rslt);
        return -1;
    }

    struct bmi2_sens_axes_data acc[BMI270_FIFO_MAX_FRAMES];
    struct bmi2_sens_axes_data gyr[BMI270_FIFO_MAX_FRAMES];
    uint16_t acc_len = BMI270_FIFO_MAX_FRAMES;
    uint16_t gyr_len = BMI270_FIFO_MAX_FRAMES;

    // OJO: en la API de Bosch, rslt < 0 es error real (BMI2_E_*), pero
    // rslt > 0 es un AVISO, no un fallo: BMI2_W_FIFO_EMPTY(1) = no habia mas
    // frames de ese tipo, BMI2_W_PARTIAL_READ(2) = se extrajeron los frames
    // completos y el ultimo quedo a medias en el buffer (normal, se completa
    // solo en la siguiente lectura), BMI2_W_DUMMY_BYTE(3) = idem para SPI.
    // Tratar estos avisos como error (como hacia antes) abortaba la lectura
    // en casi todas las pasadas sin motivo.
    rslt = bmi2_extract_accel(acc, &acc_len, &fifo, &s_bmi);
    if (rslt < BMI2_OK) {
        ESP_LOGE(TAG, "FIFO extract_accel fallo, rslt=%d", rslt);
        return -1;
    }
    rslt = bmi2_extract_gyro(gyr, &gyr_len, &fifo, &s_bmi);
    if (rslt < BMI2_OK) {
        ESP_LOGE(TAG, "FIFO extract_gyro fallo, rslt=%d", rslt);
        return -1;
    }

    if (skipped_out) {
        *skipped_out = fifo.skipped_frame_count;   // desborde detectado en ESTA lectura
    }

    int n = (acc_len < gyr_len) ? acc_len : gyr_len;
    if (n > max_out) {
        n = max_out;   // no deberia pasar con MAX_FRAMES bien dimensionado, pero por si acaso
    }
    for (int i = 0; i < n; i++) {
        out[i].ax = acc[i].x;
        out[i].ay = acc[i].y;
        out[i].az = acc[i].z;
        out[i].gx = gyr[i].x;
        out[i].gy = gyr[i].y;
        out[i].gz = gyr[i].z;
    }
    return n;
}
