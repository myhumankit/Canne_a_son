/*
 *  Lecture et agrégation de tous les capteurs (CH201, MPU6050, TFMiniPlus)
 *
 *  @author BOUFALLOUS Amin
 *  @modified 2026-04-11
 */

#include "sensorManager.h"
#include "peripheralManager.h"
#include "CH201.h"
#include "MPU6050_custom.h"
#include "TFMiniPlus.h"
#include <Arduino.h>
#include <math.h>

extern MPU6050_custom IMU;
extern TFMiniPlus TFMP;

SensorManager::SensorManager()
    : data_ready(false), imu_available(false), active_sensors(0xFF) {
    memset(&sensor_data, 0, sizeof(sensor_data_t));
}

bool SensorManager::init() {
    Serial.println("[SENSOR] Initializing all sensors...");
    bool any_failure = false;

    if(!initCH201()) {
        Serial.println("  ❌ CH201 init failed");
        active_sensors &= ~(SENSOR_CH201_0 | SENSOR_CH201_1);
        any_failure = true;
    } else {
        Serial.println("  ✓ CH201 ready");
    }

    if(IMU.Init() != 0) {
        Serial.println("  ❌ MPU6050 init failed — motion detection disabled");
        active_sensors &= ~SENSOR_MPU6050;
        imu_available = false;
        any_failure = true;
    } else {
        Serial.println("  ✓ MPU6050 ready");
        imu_available = true;
    }

    if(!TFMP.begin(10)) {
        Serial.println("  ❌ LIDAR init failed");
        active_sensors &= ~SENSOR_LIDAR;
        any_failure = true;
    } else {
        Serial.println("  ✓ LIDAR ready");
    }

    if(any_failure) {
        PeripheralManager::getInstance().signalError();
    }

    return !any_failure;
}

void SensorManager::update() {
    sensor_data.timestamp = millis();
    
    if(active_sensors & SENSOR_CH201_0) readCH201();
    if(active_sensors & SENSOR_MPU6050) readMPU6050();
    if(active_sensors & SENSOR_LIDAR) readLIDAR();
    
    data_ready = true;
}

bool SensorManager::isDataReady() {
    bool ready = data_ready;
    data_ready = false;
    return ready;
}

void SensorManager::enableSensor(uint8_t sensor_mask) {
    active_sensors |= sensor_mask;
}

void SensorManager::disableSensor(uint8_t sensor_mask) {
    active_sensors &= ~sensor_mask;
}

void SensorManager::clearDistances() {
    sensor_data.distance_ch201[0] = 0;
    sensor_data.distance_ch201[1] = 0;
    sensor_data.distance_lidar    = 0;
}

void SensorManager::readCH201() {
    // Call twice per cycle: first handles TIMER_FLAG (trigger),
    // second handles DATA_READY_FLAG (read) if both are pending.
    int result = sendReceiveCH201(sensor_data.distance_ch201,
                                  sensor_data.amplitude_ch201);
    if(result == 1) {
        // Triggered a new measurement — check immediately if data from
        // previous measurement is also ready (DATA_READY_FLAG still set)
        sendReceiveCH201(sensor_data.distance_ch201,
                         sensor_data.amplitude_ch201);
    }
}

void SensorManager::readMPU6050() {
    IMU.getAttitude(&sensor_data.pitch, &sensor_data.roll);
    sensor_data.accel_magnitude = IMU.getAccelMagnitude();
}

void SensorManager::readLIDAR() {
    int16_t dist = TFMP.getDistance();
    if(dist >= 0) {
        sensor_data.distance_lidar = dist;
    }
}
