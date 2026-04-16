#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mqtt_client.h"

#include "sensor.h"
#include "oled.h"
#include "health_monitor.h"

/* ══════════════════════════════════════════
   ⚙  CHỈNH 3 DÒNG NÀY TRƯỚC KHI FLASH
   ══════════════════════════════════════════ */
#define WIFI_SSID        "Viet"
#define WIFI_PASS        "Mottoichin"
#define MQTT_BROKER_URI  "mqtt://broker.hivemq.com:1883"
/* ══════════════════════════════════════════ */

#define TAG              "MAIN"
#define WIFI_CONNECTED_BIT  BIT0

static EventGroupHandle_t        s_wifi_eg;
static esp_mqtt_client_handle_t  s_mqtt_client    = NULL;
static bool                      s_mqtt_connected = false;

/* ════════════════════════════════════════
   MQTT EVENT HANDLER
   ════════════════════════════════════════ */
static void mqtt_event_handler(void *arg,
                                esp_event_base_t base,
                                int32_t event_id,
                                void *event_data)
{
    esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {

    case MQTT_EVENT_CONNECTED:
        s_mqtt_connected = true;
        ESP_LOGI(TAG, "MQTT connected");

        esp_mqtt_client_subscribe(s_mqtt_client, "esp32/control", 1);
        esp_mqtt_client_subscribe(s_mqtt_client, "esp32/led",     1);

        /* Báo online với retain */
        esp_mqtt_client_publish(s_mqtt_client,
                                "esp32/status", "online", 0, 1, true);
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_mqtt_connected = false;
        ESP_LOGW(TAG, "MQTT disconnected — auto retry...");
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "Topic: %.*s  Data: %.*s",
                 ev->topic_len, ev->topic,
                 ev->data_len,  ev->data);

        /* Điều khiển LED */
        if (strncmp(ev->topic, "esp32/led", ev->topic_len) == 0) {
            if      (strncmp(ev->data, "ON",  ev->data_len) == 0)
                ESP_LOGI(TAG, "LED -> ON");
            else if (strncmp(ev->data, "OFF", ev->data_len) == 0)
                ESP_LOGI(TAG, "LED -> OFF");
        }

        /* Lệnh điều khiển khác */
        if (strncmp(ev->topic, "esp32/control", ev->topic_len) == 0) {
            ESP_LOGI(TAG, "Control: %.*s", ev->data_len, ev->data);
        }
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error type=%d", ev->error_handle->error_type);
        break;

    default:
        break;
    }
}

/* ════════════════════════════════════════
   MQTT INIT
   ════════════════════════════════════════ */
static void mqtt_start(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .session = {
            .keepalive = 30,
            .last_will = {
                .topic  = "esp32/status",
                .msg    = "offline",
                .qos    = 1,
                .retain = true,
            },
        },
        .network.reconnect_timeout_ms = 3000,
    };

    s_mqtt_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_mqtt_client,
                                   ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);
    ESP_LOGI(TAG, "MQTT client started -> %s", MQTT_BROKER_URI);
}

/* ════════════════════════════════════════
   PUBLISH TASK  — mỗi 3 giây
   Lấy dữ liệu thực từ sensor, publish lên
   topic  esp32/sensor
   ════════════════════════════════════════ */
static void publish_task(void *pv)
{
    char payload[128];

    /* Chờ MQTT kết nối xong */
    while (!s_mqtt_connected) vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        SensorData d = sensor_get_data();

        if (d.finger && d.ready) {
            snprintf(payload, sizeof(payload),
                     "{\"bpm\":%d,\"spo2\":%d,\"finger\":true}",
                     d.bpm, d.spo2);
        } else if (!d.finger) {
            snprintf(payload, sizeof(payload),
                     "{\"bpm\":0,\"spo2\":0,\"finger\":false}");
        } else {
            /* Đang đo, chưa đủ buffer */
            snprintf(payload, sizeof(payload),
                     "{\"bpm\":0,\"spo2\":0,\"finger\":true,\"ready\":false}");
        }

        if (s_mqtt_connected) {
            esp_mqtt_client_publish(s_mqtt_client,
                                    "esp32/sensor",
                                    payload, 0, 1, 0);
            ESP_LOGI(TAG, "Publish -> %s", payload);
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

/* ════════════════════════════════════════
   WIFI EVENT HANDLER
   ════════════════════════════════════════ */
static void wifi_event_handler(void *arg,
                                esp_event_base_t base,
                                int32_t event_id,
                                void *data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi mat ket noi, thu lai...");
        s_mqtt_connected = false;
        esp_wifi_connect();
    }
    else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "WiFi OK — IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

/* ════════════════════════════════════════
   WIFI INIT
   ════════════════════════════════════════ */
static void wifi_init(void)
{
    s_wifi_eg = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                wifi_event_handler, NULL);

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    esp_wifi_start();

    ESP_LOGI(TAG, "Dang ket noi WiFi: %s ...", WIFI_SSID);
    xEventGroupWaitBits(s_wifi_eg, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
}

/* ════════════════════════════════════════
   APP MAIN
   ════════════════════════════════════════ */
void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32-C3 Health Monitor + MQTT ===");

    /* NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Phần cứng */
    sensor_init();   /* I2C + MAX30102 */
    oled_init();
    oled_clear();

    /* Mạng */
    wifi_init();
    mqtt_start();

    /* Health monitor (Sensor_Task + Health_Monitor_Task + Timer 500ms) */
    health_monitor_start(s_mqtt_client);

    /* Publish task: gửi dữ liệu sensor lên MQTT mỗi 3 giây */
    xTaskCreate(publish_task, "publish", 4096, NULL, 5, NULL);
}