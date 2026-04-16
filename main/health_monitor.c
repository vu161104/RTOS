#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "mqtt_client.h"

#include "sensor.h"
#include "oled.h"
#include "health_monitor.h"

#define TAG  "HEALTH"

/* ════════════════════════════════════════
   BIẾN NỘI BỘ
   ════════════════════════════════════════ */
static EventGroupHandle_t        s_events;
static esp_mqtt_client_handle_t  s_mqtt   = NULL;
static TaskHandle_t              s_sensor_task_handle = NULL;
static TimerHandle_t             s_timer;

/* ════════════════════════════════════════
   TIMER CALLBACK  — kích Sensor_Task mỗi 500 ms
   ════════════════════════════════════════ */
static void timer_cb(TimerHandle_t xTimer)
{
    xTaskNotifyGive(s_sensor_task_handle);
}

/* ════════════════════════════════════════
   TASK 1: Sensor_Task  (Priority 5)
   Đọc MAX30102, tính BPM/SpO2, hiển thị OLED,
   set Event Group khi vượt ngưỡng.
   ════════════════════════════════════════ */
static void Sensor_Task(void *pv)
{
    char line[20];

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensor_feed();                    /* Đọc FIFO 1 lần           */
        SensorData d = sensor_get_data(); /* Tính BPM/SpO2 + lọc MA   */

        /* ── Không có ngón tay ── */
        if (!d.finger) {
            oled_clear();
            oled_print_line(0, "NO FINGER");
            ESP_LOGW(TAG, "Khong co ngon tay");
            xEventGroupClearBits(s_events, BIT_BPM_ABNORMAL | BIT_SPO2_LOW);
            continue;
        }

        /* ── Đang thu thập đủ buffer ── */
        if (!d.ready) {
            oled_clear();
            oled_print_line(0, "MEASURING...");
            ESP_LOGI(TAG, "Dang do...");
            continue;
        }

        ESP_LOGI(TAG, "BPM: %d | SpO2: %d%%", d.bpm, d.spo2);

        /* ── Hiển thị OLED ── */
        oled_clear();
        snprintf(line, sizeof(line), "BPM: %d",   d.bpm);  oled_print_line(0, line);
        snprintf(line, sizeof(line), "SPO2:%d%%",  d.spo2); oled_print_line(2, line);

        /* ── Kiểm tra ngưỡng và set Event Group ── */
        EventBits_t bits = 0;

        if (d.bpm < BPM_LOW || d.bpm > BPM_HIGH) {
            bits |= BIT_BPM_ABNORMAL;
        } else {
            xEventGroupClearBits(s_events, BIT_BPM_ABNORMAL);
        }

        if (d.spo2 < SPO2_LOW) {
            bits |= BIT_SPO2_LOW;
        } else {
            xEventGroupClearBits(s_events, BIT_SPO2_LOW);
        }

        if (bits) xEventGroupSetBits(s_events, bits);
    }
}

/* ════════════════════════════════════════
   TASK 2: Health_Monitor_Task  (Priority 10)
   Chờ Event Group, phản hồi cảnh báo qua
   OLED và MQTT.
   ════════════════════════════════════════ */
static void Health_Monitor_Task(void *pv)
{
    while (1) {
        EventBits_t bits = xEventGroupWaitBits(
            s_events,
            BIT_BPM_ABNORMAL | BIT_SPO2_LOW,
            pdFALSE,      /* không tự clear */
            pdFALSE,      /* ANY bit         */
            portMAX_DELAY
        );

        /* ── BPM bất thường ── */
        if (bits & BIT_BPM_ABNORMAL) {
            oled_print_line(4, "WARN: BPM!");
            ESP_LOGW(TAG, "CANH BAO: Nhip tim bat thuong!");

            if (s_mqtt) {
                esp_mqtt_client_publish(s_mqtt,
                    "esp32/alert", "{\"alert\":\"BPM_ABNORMAL\"}", 0, 1, 0);
            }
        }

        /* ── SpO2 thấp → cảnh báo khẩn cấp ── */
        if (bits & BIT_SPO2_LOW) {
            oled_print_line(6, "STOP!SPO2!");
            ESP_LOGE(TAG, "KHAN CAP: SpO2 thap! Can dung xe!");

            if (s_mqtt) {
                esp_mqtt_client_publish(s_mqtt,
                    "esp32/alert", "{\"alert\":\"SPO2_LOW\"}", 0, 1, 0);
            }
        }

        xEventGroupClearBits(s_events, bits);
        vTaskDelay(pdMS_TO_TICKS(500)); /* tránh spam cảnh báo */
    }
}

/* ════════════════════════════════════════
   PUBLIC API
   ════════════════════════════════════════ */
void health_monitor_start(esp_mqtt_client_handle_t mqtt_client)
{
    s_mqtt   = mqtt_client;
    s_events = xEventGroupCreate();

    xTaskCreate(Sensor_Task,        "Sensor",    4096, NULL,  5, &s_sensor_task_handle);
    xTaskCreate(Health_Monitor_Task,"HealthMon", 2048, NULL, 10, NULL);

    /* Timer 500 ms kích Sensor_Task */
    s_timer = xTimerCreate("SensorTimer",
                            pdMS_TO_TICKS(500),
                            pdTRUE, NULL, timer_cb);
    xTimerStart(s_timer, 0);

    ESP_LOGI(TAG, "Health Monitor started | BPM[%d-%d] SpO2>%d%%",
             BPM_LOW, BPM_HIGH, SPO2_LOW);
}

EventGroupHandle_t health_monitor_get_events(void)
{
    return s_events;
}