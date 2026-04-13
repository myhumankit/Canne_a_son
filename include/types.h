#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>

// ========== ÉNUMÉRATIONS ==========

typedef enum {
    MODE_RUN = 0,
    MODE_STANDBY = 1,
    MODE_LIGHT_SLEEP = 2,
    MODE_DEEP_SLEEP = 3,
    MODE_HIBERNATION = 4
} power_mode_t;

typedef enum {
    APP_STATE_INIT = 0,
    APP_STATE_IDLE = 1,
    APP_STATE_DETECTING = 2,
    APP_STATE_ALERT = 3,
    APP_STATE_SLEEPING = 4
} app_state_t;

typedef enum {
    EVENT_NONE = 0,
    EVENT_SENSOR_READY = 1,
    EVENT_OBSTACLE_DETECTED = 2,
    EVENT_OBSTACLE_CLEARED = 3,
    EVENT_IMU_MOVED = 4,
    EVENT_IMU_STILL = 5,
    EVENT_LOW_BATTERY = 6,
    EVENT_MODE_CHANGED = 7,
    EVENT_TIMER_TICK = 8
} event_type_t;

typedef enum {
    ALERT_NONE = 0,
    ALERT_CLOSE = 1,      // < 50 cm
    ALERT_MEDIUM = 2,     // 50-100 cm
    ALERT_FAR = 3         // > 100 cm
} alert_level_t;

// ========== STRUCTURES ==========

typedef struct {
    unsigned int distance_ch201[2];
    unsigned int amplitude_ch201[2];
    int16_t  distance_lidar;
    float    pitch;
    float    roll;
    float    accel_magnitude;
    uint8_t  confidence;
    uint32_t timestamp;
} sensor_data_t;

typedef struct {
    event_type_t type;
    uint32_t timestamp;
    union {
        sensor_data_t sensor_data;
        alert_level_t alert_level;
        power_mode_t new_mode;
    } data;
} event_t;

typedef struct {
    float esp32;
    float ch201;
    float mpu6050;
    float tfminiplus;
    float buzzer;
    float vibreur;
    float total;
} consumption_t;

#endif
