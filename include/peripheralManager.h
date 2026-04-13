#ifndef PERIPHERAL_MANAGER_H
#define PERIPHERAL_MANAGER_H

#include "types.h"

class PeripheralManager {
public:
    static PeripheralManager& getInstance() {
        static PeripheralManager instance;
        return instance;
    }
    
    bool init();
    void update();  // Call regularly in loop
    
    // Alert functions
    void triggerAlert(alert_level_t level, uint16_t duration_ms = 100);
    void stopAlert();
    void muteAll();
    void signalError();  // 3 short beeps — sensor init failure notification
    
private:
    PeripheralManager();
    
    struct {
        uint32_t buzzer_end_time;
        uint32_t vibreur_end_time;
        bool buzzer_active;
        bool vibreur_active;
    } state;
    
    void playBuzz(alert_level_t level);
    void playVibration(alert_level_t level);
};

#endif
