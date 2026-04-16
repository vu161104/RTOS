#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ════════════════════════════════════════
   NGƯỠNG
   ════════════════════════════════════════ */
#define IR_FINGER_THRESHOLD  50000
#define BPM_LOW              50
#define BPM_HIGH             120
#define SPO2_LOW             92

/* ════════════════════════════════════════
   BUFFER / SAMPLING
   ════════════════════════════════════════ */
#define SAMPLE_RATE          2
#define BUFFER_SIZE          20
#define MIN_PEAK_DISTANCE    4
#define MA_SIZE              5

/* ════════════════════════════════════════
   MOVING AVERAGE FILTER
   ════════════════════════════════════════ */
typedef struct {
    float buf[MA_SIZE];
    int   idx;
    int   count;
    float sum;
} MovingAvg;

void  ma_init(MovingAvg *f);
float ma_update(MovingAvg *f, float val);

/* ════════════════════════════════════════
   DỮ LIỆU ĐẦU RA
   ════════════════════════════════════════ */
typedef struct {
    int  bpm;
    int  spo2;
    bool finger;   /* true = có ngón tay            */
    bool ready;    /* true = buffer đủ, đã tính được */
} SensorData;

/* ════════════════════════════════════════
   API
   ════════════════════════════════════════ */
void       sensor_init(void);
void       sensor_feed(void);
SensorData sensor_get_data(void);