#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include "types.h"

class PowerManager {
public:
    static PowerManager& getInstance() {
        static PowerManager instance;
        return instance;
    }
    
    bool init();
    void setPowerMode(power_mode_t mode);
    power_mode_t getPowerMode() const { return current_mode; }
    
    void printStats();
    const consumption_t& getConsumption() const { return consumption; }
    
private:
    PowerManager();
    
    power_mode_t current_mode;
    consumption_t consumption;
    
    // Per-component configuration
    void configESP32(power_mode_t mode);
    void configCH201(power_mode_t mode);
    void configMPU6050(power_mode_t mode);
    void configTFMiniPlus(power_mode_t mode);
    void configBuzzer(power_mode_t mode);
    void configVibreur(power_mode_t mode);
};

#endif
