#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"

/* =========================================================
 *  Utility
 * ========================================================= */
#define NOW_MS()  ((uint32_t)(esp_timer_get_time() / 1000ULL))

/* =========================================================
 *  Pin definitions
 * ========================================================= */
#define LED_RED_PIN      GPIO_NUM_12
#define LED_GREEN_PIN    GPIO_NUM_14
#define LED_BLUE_PIN     GPIO_NUM_13

#define GPS_UART         UART_NUM_2
#define GPS_RX_PIN       GPIO_NUM_16
#define GPS_TX_PIN       GPIO_NUM_17
#define GPS_BAUD         9600
#define GPS_BUF_SIZE     1024

#define SOIL_UART        UART_NUM_1
#define SOIL_TX_PIN      GPIO_NUM_18
#define SOIL_RX_PIN      GPIO_NUM_19
#define SOIL_DE_RE_PIN   GPIO_NUM_4
#define SOIL_BAUD        9600

#define LDR_ADC_CHANNEL  ADC_CHANNEL_6
#define BUTTON_PIN       GPIO_NUM_32

/* =========================================================
 *  Timing
 * ========================================================= */
#define POWERON_RED_MS       1000
#define BLE_SEARCH_BLINK_MS  200
#define BLE_CONN_BURST_MS    3000
#define GPS_FIX_LED_MS       3000
#define BTN_POLL_MS          50
#define BTN_LONG_MS          2000
#define BLE_RECONNECT_DELAY_MS 500

#define SENSOR_POLL_MS       1000

#define RS485_DE_PRE_US          500
#define RS485_DE_POST_US         500
/*
 * OPT-1: Reduced from 500 ms to 10 ms.
 * Modbus spec requires only 3.5 char-times of silence between frames.
 * At 9600 baud that is ~3.6 ms; 10 ms leaves ample margin.
 * Previously this added 3.5 s of dead-wait per full sensor cycle.
 */
#define MODBUS_FRAME_GAP_MS      10
#define MODBUS_RESP_WAIT_MS      2000
#define MODBUS_RETRIES           3
#define MODBUS_RETRY_GAP_MS      500
#define MODBUS_BUS_RECOVERY_MS   2000

/* =========================================================
 *  Binary packet constants
 * ========================================================= */
#define PKT_START_BYTE   0xAA
#define PKT_END_BYTE     0x55
#define PKT_VERSION      0x01
#define PKT_TOTAL_SIZE   31
#define PKT_PAYLOAD_LEN  25

/* =========================================================
 *  FreeRTOS events
 * ========================================================= */
#define EVT_SENSOR_READY  BIT0
#define EVT_BLE_SEND_NOW  BIT1
static EventGroupHandle_t g_events = NULL;

/* =========================================================
 *  Shared data
 * ========================================================= */
typedef struct {
    float latitude, longitude, altitude, speed_kmh;
    int   satellites;
    char  utc_time[12];
    char  date[8];
    bool  fix_valid;
} gps_data_t;

typedef struct {
    float    temperature, moisture, ph, ldr_pct;
    uint16_t conductivity, nitrogen, phosphorus, potassium;
} sensor_data_t;

static gps_data_t        g_gps    = {0};
static sensor_data_t     g_sensor = {0};
static SemaphoreHandle_t g_mutex  = NULL;

/* =========================================================
 *  BLE / GATTS state
 * ========================================================= */
#define SOILKIT_SERVICE_UUID   0x00FF
#define SOILKIT_CHAR_DATA_UUID 0xFF01
#define SOILKIT_CHAR_CMD_UUID  0xFF02
#define GATTS_APP_ID           0
#define GATTS_TAG              "BLE"
#define DEVICE_NAME            "SoilTestKit"
#define BLE_MTU                500

enum {
    IDX_SVC,
    IDX_CHAR_DATA,
    IDX_CHAR_DATA_VAL,
    IDX_CHAR_DATA_CCCD,
    IDX_CHAR_CMD,
    IDX_CHAR_CMD_VAL,
    IDX_NB
};

static uint16_t      s_gatts_if       = ESP_GATT_IF_NONE;
static uint16_t      s_conn_id        = 0xFFFF;
static bool          s_ble_connected  = false;
static bool          s_notify_enabled = false;
static uint16_t      s_handle_table[IDX_NB];
static uint8_t       s_packet_id      = 0;
static esp_bd_addr_t s_peer_bda       = {0};

typedef enum {
    BLE_SEARCHING,
    BLE_CONN_BURST,
    BLE_CONNECTED,
    BLE_DISCONNECTING
} ble_state_t;

/*
 * OPT-2: Guard s_ble_state with a spinlock.
 * It is written from the BT-stack task (GATTS/GAP callbacks) and read/written
 * from bt_led_task on a different core. volatile alone does not guarantee
 * atomicity on a dual-core ESP32; portENTER_CRITICAL uses a per-core spinlock
 * that is safe from task context.
 */
static volatile ble_state_t s_ble_state      = BLE_SEARCHING;
static volatile uint32_t    s_state_change_tick = 0;
static portMUX_TYPE         s_ble_state_mux  = portMUX_INITIALIZER_UNLOCKED;

static inline ble_state_t ble_state_get(void)
{
    portENTER_CRITICAL(&s_ble_state_mux);
    ble_state_t s = s_ble_state;
    portEXIT_CRITICAL(&s_ble_state_mux);
    return s;
}

static inline void ble_state_set(ble_state_t s)
{
    portENTER_CRITICAL(&s_ble_state_mux);
    s_ble_state = s;
    portEXIT_CRITICAL(&s_ble_state_mux);
}

static volatile bool     s_gps_led_shown  = false;
static volatile bool     s_gps_led_active = false;
static volatile uint32_t s_gps_led_tick   = 0;

static const char *T_GPS  = "GPS";
static const char *T_SOIL = "SOIL";
static const char *T_LDR  = "LDR";
static const char *T_MAIN = "MAIN";

static inline void led(gpio_num_t pin, int lvl) { gpio_set_level(pin, lvl); }

/* =========================================================
 *  BLE advertising
 * ========================================================= */
static uint8_t s_adv_service_uuid[] = {
    0xfb,0x34,0x9b,0x5f,0x80,0x00,0x00,0x80,
    0x00,0x10,0x00,0x00,0xFF,0x00,0x00,0x00,
};

static esp_ble_adv_data_t s_adv_data = {
    .set_scan_rsp=false, .include_name=true, .include_txpower=false,
    .min_interval=0x0006, .max_interval=0x0010, .appearance=0x00,
    .manufacturer_len=0, .p_manufacturer_data=NULL,
    .service_data_len=0, .p_service_data=NULL,
    .service_uuid_len=sizeof(s_adv_service_uuid),
    .p_service_uuid=s_adv_service_uuid,
    .flag=ESP_BLE_ADV_FLAG_GEN_DISC|ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
};

static esp_ble_adv_data_t s_scan_rsp = {
    .set_scan_rsp=true, .include_name=true, .include_txpower=true,
    .appearance=0x00, .manufacturer_len=0, .p_manufacturer_data=NULL,
    .service_data_len=0, .p_service_data=NULL,
    .service_uuid_len=sizeof(s_adv_service_uuid),
    .p_service_uuid=s_adv_service_uuid,
    .flag=ESP_BLE_ADV_FLAG_GEN_DISC|ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
};

static esp_ble_adv_params_t s_adv_params = {
    .adv_int_min=0x20, .adv_int_max=0x40,
    .adv_type=ADV_TYPE_IND, .own_addr_type=BLE_ADDR_TYPE_PUBLIC,
    .channel_map=ADV_CHNL_ALL,
    .adv_filter_policy=ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static uint8_t s_adv_config_done = 0;
#define ADV_CONFIG_FLAG      BIT0
#define SCAN_RSP_CONFIG_FLAG BIT1

/* =========================================================
 *  GATT attribute table
 * ========================================================= */
static const uint16_t UUID_PRI_SVC      = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t UUID_CHAR_DECLARE = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t UUID_CCCD         = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t UUID_SVC_VAL      = SOILKIT_SERVICE_UUID;
static const uint16_t UUID_DATA_CHAR    = SOILKIT_CHAR_DATA_UUID;
static const uint16_t UUID_CMD_CHAR     = SOILKIT_CHAR_CMD_UUID;
static const uint8_t  DATA_PROP = ESP_GATT_CHAR_PROP_BIT_NOTIFY |
                                   ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t  CMD_PROP  = ESP_GATT_CHAR_PROP_BIT_WRITE_NR;

static uint8_t s_cccd_val[2]              = {0x00, 0x00};
static uint8_t s_data_val[PKT_TOTAL_SIZE] = {0};
static uint8_t s_cmd_val[32]              = {0};

static const esp_gatts_attr_db_t s_attr_table[IDX_NB] = {
    [IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16,(uint8_t*)&UUID_PRI_SVC,ESP_GATT_PERM_READ,
         sizeof(uint16_t),sizeof(uint16_t),(uint8_t*)&UUID_SVC_VAL}
    },
    [IDX_CHAR_DATA] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16,(uint8_t*)&UUID_CHAR_DECLARE,ESP_GATT_PERM_READ,
         sizeof(uint8_t),sizeof(uint8_t),(uint8_t*)&DATA_PROP}
    },
    [IDX_CHAR_DATA_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16,(uint8_t*)&UUID_DATA_CHAR,ESP_GATT_PERM_READ,
         PKT_TOTAL_SIZE,PKT_TOTAL_SIZE,s_data_val}
    },
    [IDX_CHAR_DATA_CCCD] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16,(uint8_t*)&UUID_CCCD,
         ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
         sizeof(s_cccd_val),sizeof(s_cccd_val),s_cccd_val}
    },
    [IDX_CHAR_CMD] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16,(uint8_t*)&UUID_CHAR_DECLARE,ESP_GATT_PERM_READ,
         sizeof(uint8_t),sizeof(uint8_t),(uint8_t*)&CMD_PROP}
    },
    [IDX_CHAR_CMD_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16,(uint8_t*)&UUID_CMD_CHAR,ESP_GATT_PERM_WRITE,
         sizeof(s_cmd_val),1,s_cmd_val}
    },
};

/* =========================================================
 *  ble_send_packet()
 * ========================================================= */
static void ble_send_packet(void)
{
    if (!s_ble_connected || !s_notify_enabled) return;
    if (s_gatts_if == ESP_GATT_IF_NONE) return;

    gps_data_t    gps;
    sensor_data_t sen;
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    gps = g_gps;
    sen = g_sensor;
    xSemaphoreGive(g_mutex);

    uint16_t ldr_x100   = (uint16_t)(sen.ldr_pct     * 100.0f + 0.5f);
    uint16_t temp_x100  = (uint16_t)(sen.temperature  * 100.0f + 0.5f);
    uint16_t ph_x100    = (uint16_t)(sen.ph            * 100.0f + 0.5f);
    uint16_t moist_x100 = (uint16_t)(sen.moisture     * 100.0f + 0.5f);
    int32_t  lat_x1e6   = (int32_t) (gps.latitude  * 1000000.0f);
    int32_t  lon_x1e6   = (int32_t) (gps.longitude * 1000000.0f);

    uint8_t pkt[PKT_TOTAL_SIZE];
    memset(pkt, 0, sizeof(pkt));

    pkt[0]  = PKT_START_BYTE;
    pkt[1]  = PKT_VERSION;
    pkt[2]  = s_packet_id++;
    pkt[3]  = PKT_PAYLOAD_LEN;

    pkt[4]  = (uint8_t)(ldr_x100        & 0xFF);
    pkt[5]  = (uint8_t)(ldr_x100        >> 8);
    pkt[6]  = (uint8_t)(temp_x100       & 0xFF);
    pkt[7]  = (uint8_t)(temp_x100       >> 8);
    pkt[8]  = (uint8_t)(ph_x100         & 0xFF);
    pkt[9]  = (uint8_t)(ph_x100         >> 8);
    pkt[10] = (uint8_t)(moist_x100      & 0xFF);
    pkt[11] = (uint8_t)(moist_x100      >> 8);
    pkt[12] = (uint8_t)(sen.conductivity & 0xFF);
    pkt[13] = (uint8_t)(sen.conductivity >> 8);
    pkt[14] = (uint8_t)(sen.nitrogen     & 0xFF);
    pkt[15] = (uint8_t)(sen.nitrogen     >> 8);
    pkt[16] = (uint8_t)(sen.phosphorus   & 0xFF);
    pkt[17] = (uint8_t)(sen.phosphorus   >> 8);
    pkt[18] = (uint8_t)(sen.potassium    & 0xFF);
    pkt[19] = (uint8_t)(sen.potassium    >> 8);

    pkt[20] = (uint8_t)( lat_x1e6        & 0xFF);
    pkt[21] = (uint8_t)((lat_x1e6 >>  8) & 0xFF);
    pkt[22] = (uint8_t)((lat_x1e6 >> 16) & 0xFF);
    pkt[23] = (uint8_t)((lat_x1e6 >> 24) & 0xFF);
    pkt[24] = (uint8_t)( lon_x1e6        & 0xFF);
    pkt[25] = (uint8_t)((lon_x1e6 >>  8) & 0xFF);
    pkt[26] = (uint8_t)((lon_x1e6 >> 16) & 0xFF);
    pkt[27] = (uint8_t)((lon_x1e6 >> 24) & 0xFF);

    pkt[28] = gps.fix_valid ? 0x01 : 0x00;

    uint8_t chk = 0;
    for (int i = 0; i < 29; i++) chk ^= pkt[i];
    pkt[29] = chk;
    pkt[30] = PKT_END_BYTE;

    esp_err_t err = esp_ble_gatts_send_indicate(
        s_gatts_if, s_conn_id,
        s_handle_table[IDX_CHAR_DATA_VAL],
        PKT_TOTAL_SIZE, pkt,
        false);

    if (err == ESP_OK)
        ESP_LOGI(GATTS_TAG,
            "Pkt#%u sent: ldr=%.1f%% temp=%.2fC pH=%.2f moist=%.1f%% "
            "EC=%u N=%u P=%u K=%u gps=%s",
            (unsigned)(s_packet_id-1),
            (double)sen.ldr_pct, (double)sen.temperature,
            (double)sen.ph, (double)sen.moisture,
            sen.conductivity, sen.nitrogen, sen.phosphorus, sen.potassium,
            gps.fix_valid ? "FIX" : "no-fix");
    else
        ESP_LOGW(GATTS_TAG, "Notify err 0x%x", err);
}

/* =========================================================
 *  GAP callback
 * ========================================================= */
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                               esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        s_adv_config_done &= ~ADV_CONFIG_FLAG;
        if (s_adv_config_done == 0) esp_ble_gap_start_advertising(&s_adv_params);
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        s_adv_config_done &= ~SCAN_RSP_CONFIG_FLAG;
        if (s_adv_config_done == 0) esp_ble_gap_start_advertising(&s_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
            ESP_LOGE(GATTS_TAG, "Adv start failed");
        else
            ESP_LOGI(GATTS_TAG, "BLE advertising started  name=\"%s\"", DEVICE_NAME);
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "Advertising stopped"); break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TAG,
            "Conn params: interval=%d latency=%d timeout=%d",
            param->update_conn_params.conn_int,
            param->update_conn_params.latency,
            param->update_conn_params.timeout);
        break;
    default: break;
    }
}

/* =========================================================
 *  GATTS callback
 * ========================================================= */
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                 esp_gatt_if_t gatts_if,
                                 esp_ble_gatts_cb_param_t *param)
{
    switch (event) {

    case ESP_GATTS_REG_EVT:
        if (param->reg.status == ESP_GATT_OK) {
            s_gatts_if = gatts_if;
            ESP_LOGI(GATTS_TAG, "GATTS registered if=%d", gatts_if);
            esp_ble_gap_set_device_name(DEVICE_NAME);
            esp_ble_gap_config_adv_data(&s_adv_data);
            s_adv_config_done |= ADV_CONFIG_FLAG;
            esp_ble_gap_config_adv_data(&s_scan_rsp);
            s_adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            esp_ble_gatts_create_attr_tab(s_attr_table, gatts_if, IDX_NB, 0);
        } else {
            ESP_LOGE(GATTS_TAG, "GATTS reg failed status=%d", param->reg.status);
        }
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK) {
            ESP_LOGE(GATTS_TAG, "Attr table failed 0x%x",
                     param->add_attr_tab.status);
        } else if (param->add_attr_tab.num_handle != IDX_NB) {
            ESP_LOGE(GATTS_TAG, "Handle count %d != %d",
                     param->add_attr_tab.num_handle, IDX_NB);
        } else {
            memcpy(s_handle_table, param->add_attr_tab.handles,
                   sizeof(s_handle_table));
            esp_ble_gatts_start_service(s_handle_table[IDX_SVC]);
            ESP_LOGI(GATTS_TAG,
                "Service started SVC=%d DATA=%d CCCD=%d CMD=%d",
                s_handle_table[IDX_SVC],
                s_handle_table[IDX_CHAR_DATA_VAL],
                s_handle_table[IDX_CHAR_DATA_CCCD],
                s_handle_table[IDX_CHAR_CMD_VAL]);
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TAG,
            "Connected conn_id=%d MAC=%02x:%02x:%02x:%02x:%02x:%02x",
            param->connect.conn_id,
            param->connect.remote_bda[0], param->connect.remote_bda[1],
            param->connect.remote_bda[2], param->connect.remote_bda[3],
            param->connect.remote_bda[4], param->connect.remote_bda[5]);
        s_conn_id        = param->connect.conn_id;
        s_ble_connected  = true;
        s_notify_enabled = false;
        memcpy(s_peer_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* OPT-2: use spinlock-guarded setter for cross-core safety */
        portENTER_CRITICAL(&s_ble_state_mux);
        s_state_change_tick = NOW_MS();
        s_ble_state = BLE_CONN_BURST;
        portEXIT_CRITICAL(&s_ble_state_mux);
        {
            esp_ble_conn_update_params_t cp = {0};
            memcpy(cp.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            cp.min_int=0x10; cp.max_int=0x20; cp.latency=0; cp.timeout=400;
            esp_ble_gap_update_conn_params(&cp);
        }
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Disconnected reason=0x%x",
                 param->disconnect.reason);
        s_ble_connected  = false;
        s_notify_enabled = false;
        s_conn_id        = 0xFFFF;
        /* OPT-2: use spinlock-guarded setter for cross-core safety */
        portENTER_CRITICAL(&s_ble_state_mux);
        s_state_change_tick = NOW_MS();
        s_ble_state = BLE_DISCONNECTING;
        portEXIT_CRITICAL(&s_ble_state_mux);
        break;

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "MTU=%d", param->mtu.mtu); break;

    case ESP_GATTS_READ_EVT:
        ESP_LOGI(GATTS_TAG, "READ handle=%d", param->read.handle); break;

    case ESP_GATTS_WRITE_EVT:
        if (!param->write.is_prep) {
            ESP_LOGI(GATTS_TAG, "WRITE handle=%d len=%d",
                     param->write.handle, param->write.len);

            if (param->write.handle == s_handle_table[IDX_CHAR_DATA_CCCD]
                && param->write.len == 2) {
                uint16_t cccd = (uint16_t)(param->write.value[1] << 8)
                              |            param->write.value[0];
                if (cccd == 0x0001) {
                    s_notify_enabled = true;
                    ESP_LOGI(GATTS_TAG, "Notifications ENABLED");
                    xEventGroupSetBits(g_events, EVT_BLE_SEND_NOW);
                } else if (cccd == 0x0000) {
                    s_notify_enabled = false;
                    ESP_LOGI(GATTS_TAG, "Notifications DISABLED");
                }
            }

            if (param->write.handle == s_handle_table[IDX_CHAR_CMD_VAL]) {
                if (param->write.len >= 3 &&
                    memcmp(param->write.value, "GET", 3) == 0) {
                    ESP_LOGI(GATTS_TAG, "GET command received");
                    xEventGroupSetBits(g_events, EVT_BLE_SEND_NOW);
                }
            }
        }
        if (param->write.need_rsp)
            esp_ble_gatts_send_response(gatts_if,
                param->write.conn_id, param->write.trans_id,
                ESP_GATT_OK, NULL);
        break;

    case ESP_GATTS_EXEC_WRITE_EVT:
        esp_ble_gatts_send_response(gatts_if,
            param->write.conn_id, param->write.trans_id,
            ESP_GATT_OK, NULL);
        break;

    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "START EVT status=%d handle=%d",
                 param->start.status, param->start.service_handle);
        break;

    case ESP_GATTS_CONF_EVT:
        if (param->conf.status != ESP_GATT_OK)
            ESP_LOGW(GATTS_TAG, "Conf fail status=%d", param->conf.status);
        break;

    default: break;
    }
}

/* =========================================================
 *  NMEA helpers
 * ========================================================= */
static double nmea_to_dec(const char *v, char dir)
{
    if (!v || strlen(v) < 4) return 0.0;
    double raw = atof(v);
    int    deg = (int)(raw / 100);
    double res = deg + (raw - deg * 100.0) / 60.0;
    if (dir == 'S' || dir == 'W') res = -res;
    return res;
}

static bool nmea_cksum_ok(const char *s)
{
    if (s[0] != '$') return false;
    const char *star = strchr(s, '*');
    if (!star || strlen(star) < 3) return false;
    uint8_t c = 0;
    for (const char *p = s + 1; p < star; p++) c ^= (uint8_t)*p;
    return c == (uint8_t)strtol(star + 1, NULL, 16);
}

static bool nmea_parse(const char *sentence, gps_data_t *out)
{
    if (!nmea_cksum_ok(sentence)) return false;
    char buf[128];
    strncpy(buf, sentence, sizeof(buf)-1); buf[sizeof(buf)-1] = '\0';
    char *star = strchr(buf, '*'); if (star) *star = '\0';
    char *f[20] = {0}; int nf = 0;
    for (char *t = strtok(buf, ","); t && nf < 20; t = strtok(NULL, ","))
        f[nf++] = t;
    if (nf >= 10 && f[0] && strcmp(f[0], "$GPRMC") == 0) {
        out->fix_valid = (f[2] && f[2][0] == 'A');
        if (f[1] && strlen(f[1]) >= 6) strncpy(out->utc_time, f[1], sizeof(out->utc_time)-1);
        if (f[9] && strlen(f[9]) == 6) strncpy(out->date, f[9], sizeof(out->date)-1);
        if (out->fix_valid) {
            out->latitude  = (float)nmea_to_dec(f[3], f[4]?f[4][0]:'N');
            out->longitude = (float)nmea_to_dec(f[5], f[6]?f[6][0]:'E');
            out->speed_kmh = f[7] ? atof(f[7]) * 1.852f : 0.0f;
        }
        return true;
    }
    if (nf >= 10 && f[0] && strcmp(f[0], "$GPGGA") == 0) {
        int fq = f[6] ? atoi(f[6]) : 0;
        out->fix_valid  = (fq > 0);
        out->satellites = f[7] ? atoi(f[7]) : 0;
        out->altitude   = f[9] ? atof(f[9]) : 0.0f;
        if (out->fix_valid) {
            out->latitude  = (float)nmea_to_dec(f[2], f[3]?f[3][0]:'N');
            out->longitude = (float)nmea_to_dec(f[4], f[5]?f[5][0]:'E');
        }
        return true;
    }
    return false;
}

/* =========================================================
 *  Modbus RTU helpers
 * ========================================================= */
static const uint8_t REQ_TEMP[]  = {0x01,0x03,0x00,0x13,0x00,0x01,0x75,0xCF};
static const uint8_t REQ_MOIST[] = {0x01,0x03,0x00,0x12,0x00,0x01,0x24,0x0F};
static const uint8_t REQ_EC[]    = {0x01,0x03,0x00,0x15,0x00,0x01,0x95,0xCE};
static const uint8_t REQ_PH[]    = {0x01,0x03,0x00,0x06,0x00,0x01,0x64,0x0B};
static const uint8_t REQ_N[]     = {0x01,0x03,0x00,0x1E,0x00,0x01,0xE4,0x0C};
static const uint8_t REQ_P[]     = {0x01,0x03,0x00,0x1F,0x00,0x01,0xB5,0xCC};
static const uint8_t REQ_K[]     = {0x01,0x03,0x00,0x20,0x00,0x01,0x85,0xC0};

static uint16_t modbus_crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

/*
 * OPT-3: soil_val() now returns the register value directly (>=0) or -1 on
 * error. Previously it returned bool and soil_req_once() discarded the value,
 * forcing a second call in the SOIL_READ macro. Now the value flows through
 * soil_req_once → soil_req → SOIL_READ in a single pass with no redundant CRC.
 */
static int soil_val(const uint8_t *r)
{
    if (r[0]!=0x01||r[1]!=0x03||r[2]!=0x02) return -1;
    uint16_t cc = modbus_crc16(r, 5);
    uint16_t cr = (uint16_t)(r[5]|(r[6]<<8));
    if (cc != cr) { ESP_LOGW(T_SOIL,"CRC fail cc=0x%04X cr=0x%04X",cc,cr); return -1; }
    return (r[3]<<8)|r[4];
}

static void rs485_tx(const uint8_t *data, size_t len)
{
    gpio_set_level(SOIL_DE_RE_PIN, 1);
    esp_rom_delay_us(RS485_DE_PRE_US);
    uart_write_bytes(SOIL_UART, (const char *)data, len);
    uart_wait_tx_done(SOIL_UART, pdMS_TO_TICKS(100));
    esp_rom_delay_us(RS485_DE_POST_US);
    gpio_set_level(SOIL_DE_RE_PIN, 0);
}

static void rs485_flush_rx(void)
{
    uart_flush_input(SOIL_UART);
    uint8_t tmp[64]; int n;
    do {
        n = uart_read_bytes(SOIL_UART, tmp, sizeof(tmp), pdMS_TO_TICKS(10));
    } while (n > 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    uart_flush_input(SOIL_UART);
}

/* OPT-3: Returns register value (>=0) on success, -1 on failure. */
static int soil_req_once(const uint8_t *req, size_t rlen, uint8_t *resp)
{
    rs485_flush_rx();
    rs485_tx(req, rlen);

    uint32_t t0 = xTaskGetTickCount();
    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(MODBUS_RESP_WAIT_MS)) {
        size_t av = 0;
        uart_get_buffered_data_len(SOIL_UART, &av);
        if (av >= 7) break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    int got = uart_read_bytes(SOIL_UART, resp, 7, pdMS_TO_TICKS(30));

    uint8_t drain[16]; int extra;
    do {
        extra = uart_read_bytes(SOIL_UART, drain, sizeof(drain),
                                pdMS_TO_TICKS(5));
    } while (extra > 0);

    if (got < 7) return -1;
    return soil_val(resp);   /* single CRC call; value propagated upward */
}

/* OPT-3: Returns register value (>=0) on success, -1 after all retries. */
static int soil_req(const uint8_t *req, size_t rlen,
                    uint8_t *resp, const char *name)
{
    for (int a = 1; a <= MODBUS_RETRIES; a++) {
        int v = soil_req_once(req, rlen, resp);
        if (v >= 0) {
            if (a > 1)
                ESP_LOGI(T_SOIL, "%s OK on attempt %d", name, a);
            return v;
        }
        ESP_LOGW(T_SOIL, "%s attempt %d/%d failed", name, a, MODBUS_RETRIES);
        if (a < MODBUS_RETRIES)
            vTaskDelay(pdMS_TO_TICKS(MODBUS_RETRY_GAP_MS));
    }
    ESP_LOGE(T_SOIL, "%s: all %d attempts failed – bus recovery %d ms",
             name, MODBUS_RETRIES, MODBUS_BUS_RECOVERY_MS);
    gpio_set_level(SOIL_DE_RE_PIN, 0);
    rs485_flush_rx();
    vTaskDelay(pdMS_TO_TICKS(MODBUS_BUS_RECOVERY_MS));
    rs485_flush_rx();
    return -1;
}

/* OPT-3: SOIL_READ macro no longer calls soil_val(); value comes from soil_req(). */
#define SOIL_READ(req_arr, buf, raw_var, name, on_success)              \
    do {                                                                 \
        int _v = soil_req((req_arr), sizeof(req_arr), (buf), (name));   \
        if (_v >= 0) { (raw_var) = _v; on_success; }                    \
        else         { any_fail = true; }                                \
        vTaskDelay(pdMS_TO_TICKS(MODBUS_FRAME_GAP_MS));                 \
    } while (0)

/* =========================================================
 *  TASK 1 - sensor_task
 * ========================================================= */
static void sensor_task(void *pv)
{
    adc_oneshot_unit_handle_t adc;
    adc_oneshot_unit_init_cfg_t ainit = {
        .unit_id = ADC_UNIT_1, .ulp_mode = ADC_ULP_MODE_DISABLE };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ainit, &adc));
    adc_oneshot_chan_cfg_t ach = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, LDR_ADC_CHANNEL, &ach));
    ESP_LOGI(T_LDR, "ADC1 CH6 (GPIO34) ready");

    uint8_t buf[7];
    int     raw;
    sensor_data_t working = {0};

    ESP_LOGI(T_SOIL, "Sensor task started – retries=%d recovery=%dms",
             MODBUS_RETRIES, MODBUS_BUS_RECOVERY_MS);

    while (1) {
        bool any_fail = false;
        raw = 0;

        xSemaphoreTake(g_mutex, portMAX_DELAY);
        working = g_sensor;
        xSemaphoreGive(g_mutex);

        SOIL_READ(REQ_TEMP,  buf, raw, "TEMP",  working.temperature  = raw / 10.0f);
        SOIL_READ(REQ_MOIST, buf, raw, "MOIST", working.moisture     = raw / 10.0f);
        SOIL_READ(REQ_EC,    buf, raw, "EC",    working.conductivity = (uint16_t)raw);
        SOIL_READ(REQ_PH,    buf, raw, "PH",    working.ph           = raw / 100.0f);
        SOIL_READ(REQ_N,     buf, raw, "N",     working.nitrogen     = (uint16_t)raw);
        SOIL_READ(REQ_P,     buf, raw, "P",     working.phosphorus   = (uint16_t)raw);
        SOIL_READ(REQ_K,     buf, raw, "K",     working.potassium    = (uint16_t)raw);

        {
            int araw = 0;
            adc_oneshot_read(adc, LDR_ADC_CHANNEL, &araw);
            working.ldr_pct = (araw / 4095.0f) * 100.0f;
            ESP_LOGI(T_LDR, "ADC=%d  Light=%.1f%%", araw, (double)working.ldr_pct);
        }

        xSemaphoreTake(g_mutex, portMAX_DELAY);
        g_sensor = working;
        xSemaphoreGive(g_mutex);

        if (any_fail) {
            ESP_LOGW(T_SOIL,
                "Cycle partial: Temp=%.1fC Moist=%.1f%% pH=%.2f "
                "EC=%u N=%u P=%u K=%u  (stale fields retained)",
                (double)working.temperature, (double)working.moisture,
                (double)working.ph,
                working.conductivity, working.nitrogen,
                working.phosphorus, working.potassium);
        } else {
            ESP_LOGI(T_SOIL,
                "Cycle OK: Temp=%.1fC Moist=%.1f%% pH=%.2f "
                "EC=%u N=%u P=%u K=%u",
                (double)working.temperature, (double)working.moisture,
                (double)working.ph,
                working.conductivity, working.nitrogen,
                working.phosphorus, working.potassium);
        }

        xEventGroupSetBits(g_events, EVT_SENSOR_READY);
        vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_MS));
    }
}

/* =========================================================
 *  TASK 2 - gps_task
 * ========================================================= */
static void gps_task(void *pv)
{
    /*
     * OPT-4: Static buffer instead of heap allocation.
     * The buffer is permanent and never freed, so a static array avoids
     * heap fragmentation and removes the null-check at startup.
     */
    static uint8_t raw[GPS_BUF_SIZE];
    char line[128]; int lpos = 0;
    ESP_LOGI(T_GPS, "GPS task started");
    while (1) {
        int len = uart_read_bytes(GPS_UART, raw, GPS_BUF_SIZE-1, pdMS_TO_TICKS(100));
        if (len <= 0) continue;
        for (int i = 0; i < len; i++) {
            char c = (char)raw[i];
            if (c == '$') lpos = 0;
            if (lpos < (int)sizeof(line)-1) line[lpos++] = c;
            if (c == '\n' && lpos > 0) {
                line[lpos] = '\0';
                for (int j = lpos-1; j >= 0; j--) {
                    if (line[j]=='\r'||line[j]=='\n') line[j]='\0'; else break;
                }
                gps_data_t tmp;
                xSemaphoreTake(g_mutex, portMAX_DELAY);
                tmp = g_gps;
                xSemaphoreGive(g_mutex);
                if (nmea_parse(line, &tmp)) {
                    xSemaphoreTake(g_mutex, portMAX_DELAY);
                    g_gps = tmp;
                    xSemaphoreGive(g_mutex);
                    if (tmp.fix_valid)
                        ESP_LOGI(T_GPS, "FIX lat=%.6f lon=%.6f alt=%.1fm sats=%d",
                            (double)tmp.latitude, (double)tmp.longitude,
                            (double)tmp.altitude, tmp.satellites);
                }
                lpos = 0;
            }
        }
    }
    vTaskDelete(NULL);
}

/* =========================================================
 *  TASK 3 - bt_led_task
 * ========================================================= */
static void bt_led_task(void *pv)
{
    gpio_config_t leds = {
        .pin_bit_mask=(1ULL<<LED_RED_PIN)|(1ULL<<LED_GREEN_PIN)|(1ULL<<LED_BLUE_PIN),
        .mode=GPIO_MODE_OUTPUT,
        .pull_up_en=GPIO_PULLUP_DISABLE,.pull_down_en=GPIO_PULLDOWN_DISABLE,
        .intr_type=GPIO_INTR_DISABLE,
    };
    gpio_config(&leds);
    led(LED_RED_PIN,0); led(LED_GREEN_PIN,0); led(LED_BLUE_PIN,0);
    led(LED_RED_PIN,1); vTaskDelay(pdMS_TO_TICKS(POWERON_RED_MS)); led(LED_RED_PIN,0);
    ESP_LOGI(GATTS_TAG, "Power-on blink done");

    gpio_config_t btn_cfg = {
        .pin_bit_mask=(1ULL<<BUTTON_PIN), .mode=GPIO_MODE_INPUT,
        .pull_up_en=GPIO_PULLUP_ENABLE,
        .pull_down_en=GPIO_PULLDOWN_DISABLE,
        .intr_type=GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_cfg);

    int last_btn = 1;
    bool btn_held = false;
    uint32_t btn_press_tick = 0, blue_timer = NOW_MS();
    bool blue_state = false;

    for (;;) {
        /*
         * OPT-5: Block on event group instead of polling with xEventGroupGetBits.
         * xEventGroupWaitBits sleeps until EVT_SENSOR_READY or EVT_BLE_SEND_NOW
         * fires OR BTN_POLL_MS elapses — whichever comes first.
         * This replaces the unconditional vTaskDelay(BTN_POLL_MS) at the loop
         * bottom, eliminating idle wakeups and cutting BLE send latency to <1 ms.
         * Button responsiveness is unchanged: the loop still runs at least every
         * BTN_POLL_MS ms.
         */
        EventBits_t ev = xEventGroupWaitBits(
            g_events,
            EVT_SENSOR_READY | EVT_BLE_SEND_NOW,
            pdTRUE,                       /* auto-clear matched bits */
            pdFALSE,                      /* any bit suffices */
            pdMS_TO_TICKS(BTN_POLL_MS));  /* timeout = button poll period */

        if (ev & (EVT_SENSOR_READY | EVT_BLE_SEND_NOW)) {
            ble_send_packet();
        }

        uint32_t now = NOW_MS();

        /* --- Button handling (active-low with pull-up) --- */
        int b = gpio_get_level(BUTTON_PIN);

        if (b == 0 && last_btn == 1) {
            btn_press_tick = now;
            btn_held = true;
            ESP_LOGI(GATTS_TAG, "Button pressed");
        }

        if (b == 1 && last_btn == 0 && btn_held) {
            ESP_LOGI(GATTS_TAG, "Button released (short press)");
            btn_held = false;
        }

        if (btn_held && (now - btn_press_tick) >= BTN_LONG_MS) {
            ESP_LOGI(GATTS_TAG, "Long press detected - disconnecting");
            btn_held = false;

            if (s_ble_connected) {
                esp_ble_gap_disconnect(s_peer_bda);
            }
            /* OPT-2: spinlock-guarded state write */
            portENTER_CRITICAL(&s_ble_state_mux);
            s_ble_state = BLE_DISCONNECTING;
            s_state_change_tick = now;
            portEXIT_CRITICAL(&s_ble_state_mux);

            while (gpio_get_level(BUTTON_PIN) == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            vTaskDelay(pdMS_TO_TICKS(BTN_POLL_MS));
        }
        last_btn = b;

        /* --- BLE state machine --- */
        /* OPT-2: read state through spinlock-guarded getter */
        ble_state_t current_state = ble_state_get();

        switch (current_state) {
            case BLE_DISCONNECTING:
                if ((now - s_state_change_tick) >= BLE_RECONNECT_DELAY_MS) {
                    portENTER_CRITICAL(&s_ble_state_mux);
                    s_ble_state = BLE_SEARCHING;
                    s_state_change_tick = now;
                    portEXIT_CRITICAL(&s_ble_state_mux);
                    esp_ble_gap_start_advertising(&s_adv_params);
                    ESP_LOGI(GATTS_TAG, "Restarting advertising after cooldown");
                }
                break;

            case BLE_CONN_BURST:
                if ((now - s_state_change_tick) >= BLE_CONN_BURST_MS) {
                    portENTER_CRITICAL(&s_ble_state_mux);
                    s_ble_state = BLE_CONNECTED;
                    portEXIT_CRITICAL(&s_ble_state_mux);
                    ESP_LOGI(GATTS_TAG, "Connection burst complete - steady state");
                }
                break;

            default:
                break;
        }

        /* --- Blue LED --- */
        ble_state_t bls = ble_state_get();
        if (bls == BLE_SEARCHING) {
            if ((now - blue_timer) >= BLE_SEARCH_BLINK_MS) {
                blue_timer = now;
                blue_state = !blue_state;
                led(LED_BLUE_PIN, blue_state);
            }
        } else if (bls == BLE_CONN_BURST || bls == BLE_CONNECTED) {
            led(LED_BLUE_PIN, 1);
        } else if (bls == BLE_DISCONNECTING) {
            if ((now - blue_timer) >= 100) {
                blue_timer = now;
                blue_state = !blue_state;
                led(LED_BLUE_PIN, blue_state);
            }
        }

        /* --- Green LED – GPS first fix ---
         * OPT-6: Skip mutex entirely once the first-fix LED has been shown.
         * s_gps_led_shown is only ever written from this task so the read is safe.
         */
        if (!s_gps_led_shown) {
            bool fix;
            xSemaphoreTake(g_mutex, portMAX_DELAY);
            fix = g_gps.fix_valid;
            xSemaphoreGive(g_mutex);

            if (fix) {
                s_gps_led_shown  = true;
                s_gps_led_active = true;
                s_gps_led_tick   = now;
                led(LED_GREEN_PIN, 1);
                ESP_LOGI(T_GPS, "GPS fix acquired - green LED on");
            }
        }
        if (s_gps_led_active && (now - s_gps_led_tick) >= GPS_FIX_LED_MS) {
            s_gps_led_active = false;
            led(LED_GREEN_PIN, 0);
            ESP_LOGI(T_GPS, "GPS fix LED off");
        }
        /* No vTaskDelay here — xEventGroupWaitBits above already provides the
         * BTN_POLL_MS sleep when no events are pending. */
    }
}

/* =========================================================
 *  app_main
 * ========================================================= */
void app_main(void)
{
    ESP_LOGI(T_MAIN, "=== Soil Testing Kit (BLE GATT binary) booting ===");

    esp_err_t ret = nvs_flash_init();
    if (ret==ESP_ERR_NVS_NO_FREE_PAGES||ret==ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    g_mutex  = xSemaphoreCreateMutex(); configASSERT(g_mutex);
    g_events = xEventGroupCreate();     configASSERT(g_events);

    /* UART2: GPS */
    const uart_config_t gps_cfg = {
        .baud_rate=GPS_BAUD, .data_bits=UART_DATA_8_BITS,
        .parity=UART_PARITY_DISABLE, .stop_bits=UART_STOP_BITS_1,
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE, .source_clk=UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(GPS_UART, &gps_cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART, GPS_TX_PIN, GPS_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART, GPS_BUF_SIZE*2, 0, 0, NULL, 0));
    ESP_LOGI(T_GPS, "UART2 RX=GPIO%d TX=GPIO%d @%d", GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);

    /* UART1: Soil sensor */
    const uart_config_t soil_cfg = {
        .baud_rate=SOIL_BAUD, .data_bits=UART_DATA_8_BITS,
        .parity=UART_PARITY_DISABLE, .stop_bits=UART_STOP_BITS_1,
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE, .source_clk=UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(SOIL_UART, 512, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(SOIL_UART, &soil_cfg));
    ESP_ERROR_CHECK(uart_set_pin(SOIL_UART, SOIL_TX_PIN, SOIL_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    {
        gpio_config_t de = {
            .pin_bit_mask=(1ULL<<SOIL_DE_RE_PIN), .mode=GPIO_MODE_OUTPUT,
            .pull_up_en=GPIO_PULLUP_DISABLE, .pull_down_en=GPIO_PULLDOWN_DISABLE,
            .intr_type=GPIO_INTR_DISABLE
        };
        ESP_ERROR_CHECK(gpio_config(&de));
        gpio_set_level(SOIL_DE_RE_PIN, 0);
    }
    ESP_LOGI(T_SOIL, "UART1/RS485 TX=GPIO%d RX=GPIO%d DE/RE=GPIO%d",
             SOIL_TX_PIN, SOIL_RX_PIN, SOIL_DE_RE_PIN);

    /* BLE Initialization */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(GATTS_APP_ID));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(BLE_MTU));

    ESP_LOGI(T_MAIN,
        "BLE ready name=\"%s\" SVC=0x%04X DATA=0x%04X CMD=0x%04X pkt=%dB",
        DEVICE_NAME, SOILKIT_SERVICE_UUID,
        SOILKIT_CHAR_DATA_UUID, SOILKIT_CHAR_CMD_UUID, PKT_TOTAL_SIZE);

    /*
     * OPT-7: sensor_task moved to core 1.
     * The ESP-IDF BT stack runs heavily on core 0. Keeping sensor_task on
     * core 0 forces it to compete with BT callbacks for CPU time.
     * Moving it to core 1 (alongside bt_led_task) distributes load evenly:
     *   Core 0 → BT stack + gps_task
     *   Core 1 → sensor_task + bt_led_task
     */
    xTaskCreatePinnedToCore(sensor_task, "sensor", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(gps_task,    "gps",    4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(bt_led_task, "bt_led", 4096, NULL, 3, NULL, 1);

    ESP_LOGI(T_MAIN, "All tasks running - waiting for Flutter BLE connection...");
}