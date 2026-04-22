/*
 * ============================================================
 *  can.c  —  CAN bus (TWAI) cho ESP32-C3
 * ============================================================
 */

#include "can.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/twai.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "CAN";

/* ── State nội bộ ─────────────────────────────────────── */
static volatile bool       s_can_ok  = false;
static can_rx_frame_t      s_last_rx = {0};
static SemaphoreHandle_t   s_rx_mutex;

/* ════════════════════════════════════════════════════════
   TASK NHẬN — chờ frame từ bus, lưu vào s_last_rx
   ════════════════════════════════════════════════════════ */
static void can_rx_task(void *arg)
{
    twai_message_t msg;

    while (1) {
        /* Chờ tối đa 500 ms, nếu không có frame thì lặp lại */
        esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(500));

        if (err == ESP_OK) {
            if (msg.rtr) {
                /* Remote frame — bỏ qua hoặc xử lý tuỳ ứng dụng */
                ESP_LOGD(TAG, "RX remote frame ID:0x%03lX", msg.identifier);
                continue;
            }

            ESP_LOGI(TAG, "RX  ID:0x%03lX  DLC:%d  [%02X %02X %02X %02X]",
                     msg.identifier,
                     msg.data_length_code,
                     msg.data[0], msg.data[1],
                     msg.data[2], msg.data[3]);

            /* Lưu vào bộ đệm thread-safe */
            if (xSemaphoreTake(s_rx_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                s_last_rx.id    = msg.identifier;
                s_last_rx.dlc   = msg.data_length_code;
                s_last_rx.valid = true;
                memcpy(s_last_rx.data, msg.data,
                       msg.data_length_code);
                xSemaphoreGive(s_rx_mutex);
            }
        }
        else if (err == ESP_ERR_TIMEOUT) {
            /* Không có frame — bình thường */
        }
        else if (err == ESP_ERR_INVALID_STATE) {
            /* Bus-Off hoặc driver bị tắt */
            ESP_LOGW(TAG, "TWAI invalid state — thử recover...");
            twai_initiate_recovery();   /* ESP-IDF v5.x (thay thế twai_recover) */
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        else {
            ESP_LOGW(TAG, "RX lỗi: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/* ════════════════════════════════════════════════════════
   TASK GIÁM SÁT — kiểm tra trạng thái bus mỗi 2 giây
   ════════════════════════════════════════════════════════ */
static void can_monitor_task(void *arg)
{
    twai_status_info_t status;

    while (1) {
        if (twai_get_status_info(&status) == ESP_OK) {
            s_can_ok = (status.state == TWAI_STATE_RUNNING);

            if (status.bus_error_count > 0 ||
                status.tx_error_counter > 96 ||
                status.rx_error_counter > 96) {
                ESP_LOGW(TAG,
                    "Bus: TX_err=%lu  RX_err=%lu  arb_lost=%lu  bus_err=%lu",
                    status.tx_error_counter,
                    status.rx_error_counter,
                    status.arb_lost_count,
                    status.bus_error_count);
            }

            /* Tự recover nếu bị Bus-Off */
            if (status.state == TWAI_STATE_BUS_OFF) {
                ESP_LOGE(TAG, "Bus-Off! Đang recover...");
                twai_initiate_recovery();   /* ESP-IDF v5.x */
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ════════════════════════════════════════════════════════
   KHỞI TẠO
   ════════════════════════════════════════════════════════ */
void can_init(void)
{
    /* Mutex bảo vệ s_last_rx */
    s_rx_mutex = xSemaphoreCreateMutex();
    configASSERT(s_rx_mutex);

    /* ── Cấu hình TWAI driver ── */
    twai_general_config_t g_cfg =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO,
                                    TWAI_MODE_NORMAL);
    g_cfg.rx_queue_len = 16;
    g_cfg.tx_queue_len = 16;

    /*
     *  Đổi baudrate tại đây:
     *    TWAI_TIMING_CONFIG_125KBITS()
     *    TWAI_TIMING_CONFIG_250KBITS()
     *    TWAI_TIMING_CONFIG_500KBITS()   ← đang dùng
     *    TWAI_TIMING_CONFIG_1MBITS()
     *
     *  ⚠ Cả hai node phải cùng baudrate!
     */
    twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_500KBITS();

    /*
     *  Filter: chấp nhận tất cả frame.
     *  Để chỉ nhận ID 0x200 (node đối diện), thay bằng:
     *
     *    twai_filter_config_t f_cfg = {
     *        .acceptance_code = (CAN_ID_REMOTE << 21),
     *        .acceptance_mask = ~(0x7FFU << 21),
     *        .single_filter   = true
     *    };
     */
    twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_cfg, &t_cfg, &f_cfg));
    ESP_ERROR_CHECK(twai_start());

    s_can_ok = true;
    ESP_LOGI(TAG, "TWAI khởi động — 500 kbps | TX:GPIO%d  RX:GPIO%d",
             CAN_TX_GPIO, CAN_RX_GPIO);

    /* Tạo task RX và monitor */
    xTaskCreate(can_rx_task,      "can_rx",  3072, NULL, 6, NULL);
    xTaskCreate(can_monitor_task, "can_mon", 2048, NULL, 3, NULL);
}

/* ════════════════════════════════════════════════════════
   GỬI DỮ LIỆU CẢM BIẾN
   ════════════════════════════════════════════════════════
   Định dạng 8 byte:
     [0] bpm_H   — byte cao của BPM
     [1] bpm_L   — byte thấp của BPM
     [2] spo2    — SpO2 (%)
     [3] flags   — bit0: finger | bit1: ready
     [4..7]      — dự phòng (0x00)
   ════════════════════════════════════════════════════════ */
bool can_send_sensor(uint16_t bpm, uint8_t spo2,
                     bool finger, bool ready)
{
    uint8_t flags = ((finger ? 1U : 0U) << 0) |
                    ((ready  ? 1U : 0U) << 1);

    uint8_t data[8] = {
        (uint8_t)(bpm >> 8),   /* [0] BPM high byte  */
        (uint8_t)(bpm & 0xFF), /* [1] BPM low byte   */
        spo2,                  /* [2] SpO2           */
        flags,                 /* [3] flags          */
        0x00, 0x00, 0x00, 0x00 /* [4..7] dự phòng   */
    };

    return can_send_raw(CAN_ID_ESP32, data, 8);
}

/* ════════════════════════════════════════════════════════
   GỬI FRAME THÔ
   ════════════════════════════════════════════════════════ */
bool can_send_raw(uint32_t id, const uint8_t *data, uint8_t length)
{
    if (!s_can_ok || length > 8) return false;

    twai_message_t msg = {
        .identifier       = id,
        .extd             = 0,       /* Standard frame (11-bit) */
        .rtr              = 0,
        .ss               = 0,
        .self             = 0,
        .dlc_non_comp     = 0,
        .data_length_code = length,
    };
    memcpy(msg.data, data, length);

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TX lỗi ID:0x%03lX  %s",
                 id, esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "TX  ID:0x%03lX  [%02X %02X %02X %02X]",
             id, data[0], data[1], data[2], data[3]);
    return true;
}

/* ════════════════════════════════════════════════════════
   LẤY FRAME NHẬN GẦN NHẤT
   ════════════════════════════════════════════════════════ */
bool can_get_last_rx(can_rx_frame_t *out)
{
    if (!out) return false;

    bool ok = false;
    if (xSemaphoreTake(s_rx_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_last_rx.valid) {
            memcpy(out, &s_last_rx, sizeof(can_rx_frame_t));
            ok = true;
        }
        xSemaphoreGive(s_rx_mutex);
    }
    return ok;
}

/* ════════════════════════════════════════════════════════
   KIỂM TRA TRẠNG THÁI
   ════════════════════════════════════════════════════════ */
bool can_is_ok(void)
{
    return s_can_ok;
}