#ifndef SENSOR_TEMPLATE_H
#define SENSOR_TEMPLATE_H

#include <cstddef>

// ========== TEMPLATE SENSOR BASE ==========
// Ce template permet de créer des drivers de capteurs réutilisables

template<typename DataType, uint8_t BufferSize = 10>
class SensorBase {
public:
    SensorBase() : buffer_index(0), is_initialized(false) {}
    
    virtual ~SensorBase() {}
    
    // Initialize sensor
    virtual bool init() = 0;
    
    // Read sensor data
    virtual bool read(DataType& data) = 0;
    
    // Get latest measurement
    const DataType& getLatest() const {
        return buffer[(buffer_index - 1) % BufferSize];
    }
    
    // Get average of last N measurements
    DataType getAverage(uint8_t count = BufferSize) {
        DataType sum = DataType();
        count = (count > BufferSize) ? BufferSize : count;
        
        for(uint8_t i = 0; i < count; i++) {
            sum += buffer[(buffer_index - i - 1) % BufferSize];
        }
        
        return sum / count;
    }
    
    // Check if sensor is ready
    bool isReady() const { return is_initialized; }
    
protected:
    // Store measurement in circular buffer
    void addMeasurement(const DataType& data) {
        buffer[buffer_index] = data;
        buffer_index = (buffer_index + 1) % BufferSize;
    }
    
    DataType buffer[BufferSize];
    uint8_t buffer_index;
    bool is_initialized;
};

#endif
