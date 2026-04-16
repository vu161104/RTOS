#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "mqtt_client.h"

/* ════════════════════════════════════════
   EVENT GROUP BITS
   ════════════════════════════════════════ */
#define BIT_BPM_ABNORMAL   BIT0   /* Nhịp tim bất thường  */
#define BIT_SPO2_LOW       BIT1   /* SpO2 dưới ngưỡng     */

/* ════════════════════════════════════════
   API
   ════════════════════════════════════════ */

/**
 * Khởi tạo Event Group + tạo Sensor_Task và Health_Monitor_Task.
 * Gọi sau khi oled_init() và sensor_init() đã xong.
 *
 * @param mqtt_client  Handle MQTT để Health_Monitor_Task publish cảnh báo.
 *                     Truyền NULL nếu chưa cần publish.
 */
void health_monitor_start(esp_mqtt_client_handle_t mqtt_client);

/**
 * Trả về Event Group handle để main.c có thể đọc trạng thái nếu cần.
 */
EventGroupHandle_t health_monitor_get_events(void);