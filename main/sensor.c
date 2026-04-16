#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "sensor.h"

/* ════════════════════════════════════════
   I2C / HARDWARE
   ════════════════════════════════════════ */
#define I2C_MASTER_SCL_IO    7
#define I2C_MASTER_SDA_IO    6
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_FREQ_HZ          100000
#define MAX30102_ADDR        0x57

/* ════════════════════════════════════════
   BIẾN NỘI BỘ
   ════════════════════════════════════════ */
static uint32_t ir_buffer[BUFFER_SIZE];
static uint32_t red_buffer[BUFFER_SIZE];
static int      buf_idx   = 0;
static int      buf_count = 0;

static MovingAvg bpm_filter;
static MovingAvg spo2_filter;

/* ════════════════════════════════════════
   MOVING AVERAGE
   ════════════════════════════════════════ */
void ma_init(MovingAvg *f)
{
    memset(f, 0, sizeof(MovingAvg));
}

float ma_update(MovingAvg *f, float val)
{
    if (f->count == MA_SIZE) f->sum -= f->buf[f->idx];
    f->buf[f->idx] = val;
    f->sum += val;
    f->idx = (f->idx + 1) % MA_SIZE;
    if (f->count < MA_SIZE) f->count++;
    return f->sum / f->count;
}

/* ════════════════════════════════════════
   I2C HELPER
   ════════════════════════════════════════ */
static esp_err_t max_write(uint8_t reg, uint8_t val)
{
    uint8_t data[2] = {reg, val};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MAX30102_ADDR,
                                      data, 2, pdMS_TO_TICKS(1000));
}

static esp_err_t max_read_reg(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MAX30102_ADDR,
                                        &reg, 1, out, len,
                                        pdMS_TO_TICKS(1000));
}

/* ════════════════════════════════════════
   SENSOR INIT
   ════════════════════════════════════════ */
void sensor_init(void)
{
    /* I2C master */
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA_IO,
        .scl_io_num       = I2C_MASTER_SCL_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    /* MAX30102 */
    max_write(0x09, 0x03); /* SPO2 mode: RED + IR          */
    max_write(0x0A, 0x27); /* SPO2 config: 100 sps, 18-bit */
    max_write(0x0C, 0x24); /* LED1 RED amplitude           */
    max_write(0x0D, 0x24); /* LED2 IR  amplitude           */

    /* Filter */
    ma_init(&bpm_filter);
    ma_init(&spo2_filter);
}

/* ════════════════════════════════════════
   ĐỌC FIFO — gọi mỗi 500 ms từ timer
   ════════════════════════════════════════ */
void sensor_feed(void)
{
    uint8_t raw[6];
    max_read_reg(0x07, raw, 6);

    uint32_t red = (uint32_t)(raw[0] & 0x03) << 16 |
                   (uint32_t) raw[1]          <<  8 |
                   (uint32_t) raw[2];
    uint32_t ir  = (uint32_t)(raw[3] & 0x03) << 16 |
                   (uint32_t) raw[4]          <<  8 |
                   (uint32_t) raw[5];

    ir_buffer[buf_idx]  = ir;
    red_buffer[buf_idx] = red;
    buf_idx = (buf_idx + 1) % BUFFER_SIZE;
    if (buf_count < BUFFER_SIZE) buf_count++;
}

/* ════════════════════════════════════════
   TÍNH BPM  (zero-crossing trên IR)
   ════════════════════════════════════════ */
static int calc_bpm(void)
{
    if (buf_count < BUFFER_SIZE) return 0;

    uint32_t dc = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) dc += ir_buffer[i];
    dc /= BUFFER_SIZE;

    int peaks = 0, last_peak = -MIN_PEAK_DISTANCE, above = 0;
    for (int i = 1; i < BUFFER_SIZE; i++) {
        int prev = above;
        above = (ir_buffer[i] > dc) ? 1 : 0;
        if (!prev && above && (i - last_peak) >= MIN_PEAK_DISTANCE) {
            peaks++;
            last_peak = i;
        }
    }
    float dur = (float)BUFFER_SIZE / SAMPLE_RATE;
    return (int)((peaks / dur) * 60.0f);
}

/* ════════════════════════════════════════
   TÍNH SPO2  (AC/DC ratio RED vs IR)
   ════════════════════════════════════════ */
static int calc_spo2(void)
{
    if (buf_count < BUFFER_SIZE) return 0;

    uint32_t ir_dc = 0, red_dc = 0;
    uint32_t ir_max = 0,  ir_min  = UINT32_MAX;
    uint32_t red_max = 0, red_min = UINT32_MAX;

    for (int i = 0; i < BUFFER_SIZE; i++) {
        ir_dc  += ir_buffer[i];
        red_dc += red_buffer[i];
        if (ir_buffer[i]  > ir_max)  ir_max  = ir_buffer[i];
        if (ir_buffer[i]  < ir_min)  ir_min  = ir_buffer[i];
        if (red_buffer[i] > red_max) red_max = red_buffer[i];
        if (red_buffer[i] < red_min) red_min = red_buffer[i];
    }
    ir_dc  /= BUFFER_SIZE;
    red_dc /= BUFFER_SIZE;
    if (ir_dc == 0 || red_dc == 0) return 0;

    float ir_ac  = (float)(ir_max  - ir_min)  / 2.0f;
    float red_ac = (float)(red_max - red_min) / 2.0f;
    if (ir_ac == 0.0f) return 0;

    float R    = (red_ac / (float)red_dc) / (ir_ac / (float)ir_dc);
    int   spo2 = (int)(110.0f - 25.0f * R);
    if (spo2 > 100) spo2 = 100;
    if (spo2 < 0)   spo2 = 0;
    return spo2;
}

/* ════════════════════════════════════════
   LẤY KẾT QUẢ (có lọc Moving Average)
   ════════════════════════════════════════ */
SensorData sensor_get_data(void)
{
    SensorData d = {0};

    /* Kiểm tra ngón tay qua mẫu IR mới nhất */
    int last = (buf_idx - 1 + BUFFER_SIZE) % BUFFER_SIZE;
    d.finger = (ir_buffer[last] >= IR_FINGER_THRESHOLD);

    if (!d.finger || buf_count < BUFFER_SIZE) {
        d.ready = false;
        return d;
    }

    d.bpm  = (int)ma_update(&bpm_filter,  (float)calc_bpm());
    d.spo2 = (int)ma_update(&spo2_filter, (float)calc_spo2());
    d.ready = true;
    return d;
}