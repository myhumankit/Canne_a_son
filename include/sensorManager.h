#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "types.h"

class SensorManager {
public:
    static SensorManager& getInstance() {
        static SensorManager instance;
        return instance;
    }
    
    bool init();
    void update();
    const sensor_data_t& getSensorData() const { return sensor_data; }
    bool isDataReady();
    bool isImuAvailable() const { return imu_available; }

    void enableSensor(uint8_t sensor_mask);
    void disableSensor(uint8_t sensor_mask);
    void clearDistances();  // zero CH201 data when sensors are idled
    
    // Sensor bit masks
    static const uint8_t SENSOR_CH201_0 = (1 << 0);
    static const uint8_t SENSOR_CH201_1 = (1 << 1);
    static const uint8_t SENSOR_MPU6050 = (1 << 2);
    static const uint8_t SENSOR_LIDAR = (1 << 3);
    
private:
    SensorManager();
    
    sensor_data_t sensor_data;
    bool data_ready;
    bool imu_available;
    uint8_t active_sensors;
    
    void readCH201();
    void readMPU6050();
    void readLIDAR();
};

#endif
