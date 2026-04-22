#pragma once

/*
 * ============================================================
 *  can.h  —  CAN bus (TWAI) cho ESP32-C3
 *  Transceiver : SN65HVD230 (VP230)
 *
 *  Kết nối phần cứng:
 *    GPIO4  →  VP230 CTX
 *    GPIO5  ←  VP230 CRX
 *    3V3    →  VP230 3V3
 *    GND    →  VP230 GND
 *
 *  Tích hợp vào main.c:
 *    #include "can.h"
 *    can_init();                    // trong app_main, sau sensor_init()
 *    can_send_sensor(bpm, spo2);    // trong publish_task hoặc health task
 * ============================================================
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Chân GPIO (đổi ở đây nếu cần) ─────────────────────── */
#define CAN_TX_GPIO     4    /* GPIO4 → VP230 CTX */
#define CAN_RX_GPIO     5    /* GPIO5 ← VP230 CRX */

/* ── CAN ID ─────────────────────────────────────────────── */
#define CAN_ID_ESP32    0x100U   /* ESP32-C3 gửi với ID này       */
#define CAN_ID_REMOTE   0x200U   /* ID của node đối diện (STM32…) */

/* ── Tốc độ bit ─────────────────────────────────────────── */
/* Đổi trong can.c: TWAI_TIMING_CONFIG_500KBITS()           */
/* Các lựa chọn: 125K / 250K / 500K / 1M                    */

/* ── Cấu trúc dữ liệu nhận được từ node khác ──────────── */
typedef struct {
    uint32_t id;            /* CAN ID của frame nhận          */
    uint8_t  data[8];       /* Payload thô (tối đa 8 byte)    */
    uint8_t  dlc;           /* Số byte thực tế                */
    bool     valid;         /* true nếu đã nhận ít nhất 1 frame */
} can_rx_frame_t;

/* ── API ────────────────────────────────────────────────── */

/**
 * @brief  Khởi tạo TWAI driver và bắt đầu 2 task TX/RX.
 *         Gọi một lần trong app_main().
 */
void can_init(void);

/**
 * @brief  Gửi dữ liệu cảm biến qua CAN ngay lập tức.
 *         Đóng gói: [bpm_H, bpm_L, spo2, flags, 0,0,0,0]
 *         flags bit0 = finger, bit1 = ready
 *
 * @param  bpm    Nhịp tim (0–300)
 * @param  spo2   SpO2 (0–100)
 * @param  finger Có ngón tay không
 * @param  ready  Dữ liệu hợp lệ chưa
 * @return true nếu gửi vào queue thành công
 */
bool can_send_sensor(uint16_t bpm, uint8_t spo2,
                     bool finger, bool ready);

/**
 * @brief  Gửi frame CAN thô (dùng khi muốn tự đóng gói).
 *
 * @param  id      CAN ID (11-bit standard)
 * @param  data    Con trỏ tới dữ liệu (tối đa 8 byte)
 * @param  length  Số byte cần gửi
 * @return true nếu gửi thành công
 */
bool can_send_raw(uint32_t id, const uint8_t *data, uint8_t length);

/**
 * @brief  Lấy frame nhận được gần nhất.
 *         Trả về false nếu chưa có frame nào.
 *
 * @param  out  Con trỏ để ghi kết quả ra
 */
bool can_get_last_rx(can_rx_frame_t *out);

/**
 * @brief  Kiểm tra CAN bus có đang hoạt động không.
 */
bool can_is_ok(void);

#ifdef __cplusplus
}
#endif