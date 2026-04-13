#include <Arduino.h>
#include "boardConfig.h"
#include "types.h"
#include "eventManager.h"
#include "sensorManager.h"
#include "peripheralManager.h"
#include "powerManager.h"
#include "appStateMachine.h"

// ========== ISR TIMER ==========
hw_timer_t *timer_sensor = NULL;
volatile bool sensor_ready_flag = false;

void IRAM_ATTR onSensorTimerISR() {
    sensor_ready_flag = true;
}

// ========== EVENT CALLBACKS ==========
void onObstacleDetected(const event_t* evt) {
    PeripheralManager::getInstance().triggerAlert(
        evt->data.alert_level, 100);
    
    AppStateMachine::getInstance().onEvent(*evt);
}

// ========== SETUP ==========
void setup() {
    Serial.begin(9600);
    delay(300);
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  ✓ CANNE À SON - Architecture V3       ║");
    Serial.println("║  ✓ Optimisé + Événementiel             ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    // Initialize managers
    SensorManager::getInstance().init();
    PeripheralManager::getInstance().init();
    PowerManager::getInstance().init();
    AppStateMachine::getInstance().init();
    
    // Register callbacks
    EventManager::getInstance().subscribe(
        EVENT_OBSTACLE_DETECTED, 
        onObstacleDetected);
    
    // Setup timer interrupt (100ms = 10 Hz)
    timer_sensor = timerBegin(0, 80, true);
    timerAttachInterrupt(timer_sensor, &onSensorTimerISR, true);
    timerAlarmWrite(timer_sensor, 100000, true);  // 100ms
    timerAlarmEnable(timer_sensor);
    
    Serial.println("✓ System ready\n");
}

// ========== LOOP - MAIN LOGIC ==========
void loop() {
    // 1. Read sensors when timer triggers
    if(sensor_ready_flag) {
        sensor_ready_flag = false;
        
        SensorManager::getInstance().update();
        
        if(SensorManager::getInstance().isDataReady()) {
            // Create sensor event
            event_t evt;
            evt.type = EVENT_SENSOR_READY;
            evt.timestamp = millis();
            evt.data.sensor_data = 
                SensorManager::getInstance().getSensorData();
            
            EventManager::getInstance().postEvent(evt);
        }
    }
    
    // 2. Process all pending events
    EventManager::getInstance().processEvents();
    
    // 3. Update state machine
    AppStateMachine::getInstance().update();
    
    // 4. Update peripherals (handle timeouts)
    PeripheralManager::getInstance().update();
    
    // 5. Let CPU sleep briefly
    delay(1);
}