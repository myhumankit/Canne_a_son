/*
 *  Machine d'états applicative : INIT → IDLE → DETECTING → ALERT → SLEEPING
 *
 *  @author BOUFALLOUS Amin
 *  @modified 2026-04-11
 */

#include "appStateMachine.h"
#include "eventManager.h"
#include "sensorManager.h"
#include "peripheralManager.h"
#include "powerManager.h"
#include <Arduino.h>

AppStateMachine::AppStateMachine()
    : current_state(APP_STATE_INIT),
      previous_state(APP_STATE_INIT),
      state_start_time(0),
      still_since_detecting(0) {}

void AppStateMachine::init() {
    if(!SensorManager::getInstance().isImuAvailable()) {
        // No IMU: motion detection impossible — stay in DETECTING permanently
        Serial.println("[STATE] IMU unavailable — starting in DETECTING (no power saving)");
        transitionTo(APP_STATE_DETECTING);
    } else {
        transitionTo(APP_STATE_IDLE);
    }
}

void AppStateMachine::update() {
    switch(current_state) {
        case APP_STATE_INIT:
            onStateInit();
            break;
        case APP_STATE_IDLE:
            onStateIdle();
            break;
        case APP_STATE_DETECTING:
            onStateDetecting();
            break;
        case APP_STATE_ALERT:
            onStateAlert();
            break;
        case APP_STATE_SLEEPING:
            onStateSleeping();
            break;
    }
}

void AppStateMachine::onEvent(const event_t& event) {
    if(event.type == EVENT_OBSTACLE_DETECTED) {
        transitionTo(APP_STATE_ALERT);
    } else if(event.type == EVENT_OBSTACLE_CLEARED) {
        transitionTo(APP_STATE_DETECTING);
    } else if(event.type == EVENT_IMU_MOVED) {
        transitionTo(APP_STATE_DETECTING);
    }
}

const char* AppStateMachine::getStateString() const {
    const char* states[] = {
        "INIT", "IDLE", "DETECTING", "ALERT", "SLEEPING"
    };
    return states[current_state];
}

void AppStateMachine::transitionTo(app_state_t new_state) {
    if(new_state == current_state) return;
    
    const char* states[] = {
        "INIT", "IDLE", "DETECTING", "ALERT", "SLEEPING"
    };
    
    Serial.printf("[STATE] %s → %s\n", 
                  states[current_state], states[new_state]);
    
    previous_state = current_state;
    current_state = new_state;
    state_start_time = millis();
    
    // Apply configuration for new state
    switch(new_state) {
        case APP_STATE_IDLE:
            PowerManager::getInstance().setPowerMode(MODE_STANDBY);
            PeripheralManager::getInstance().muteAll();
            break;
            
        case APP_STATE_DETECTING:
            PowerManager::getInstance().setPowerMode(MODE_RUN);
            PeripheralManager::getInstance().muteAll();
            still_since_detecting = 0;  // reset stillness timer on every entry
            break;
            
        case APP_STATE_ALERT:
            PowerManager::getInstance().setPowerMode(MODE_RUN);
            // Alert already triggered, don't mute
            break;
            
        case APP_STATE_SLEEPING:
            PowerManager::getInstance().setPowerMode(MODE_DEEP_SLEEP);
            PeripheralManager::getInstance().muteAll();
            break;
            
        case APP_STATE_INIT:
        default:
            break;
    }
}

void AppStateMachine::onStateInit() {
    transitionTo(APP_STATE_IDLE);
}

void AppStateMachine::onStateIdle() {
    // Safety: if IMU became unavailable, fall back to always detecting
    if(!SensorManager::getInstance().isImuAvailable()) {
        transitionTo(APP_STATE_DETECTING);
        return;
    }

    uint32_t idle_time = millis() - state_start_time;

    // Transition to sleep after 2 minutes without movement
    if(idle_time > 120000) {
        transitionTo(APP_STATE_SLEEPING);
        return;
    }

    // Wake up to detecting on significant movement (baseline at rest ≈ 1g)
    const sensor_data_t& data = SensorManager::getInstance().getSensorData();
    if(data.accel_magnitude > 1.2f) {
        event_t evt;
        evt.type = EVENT_IMU_MOVED;
        evt.timestamp = millis();
        EventManager::getInstance().postEvent(evt);
        transitionTo(APP_STATE_DETECTING);
    }
}

void AppStateMachine::onStateDetecting() {
    const sensor_data_t& data = SensorManager::getInstance().getSensorData();

    // Use the minimum of both CH201 sensors (ignore zero = no target)
    unsigned int d0 = data.distance_ch201[0];
    unsigned int d1 = data.distance_ch201[1];
    unsigned int dist = 0;
    if(d0 > 0 && d1 > 0)      dist = (d0 < d1) ? d0 : d1;
    else if(d0 > 0)            dist = d0;
    else if(d1 > 0)            dist = d1;

    alert_level_t level = ALERT_NONE;
    if(dist > 0 && dist < 50)         level = ALERT_CLOSE;
    else if(dist >= 50 && dist < 100) level = ALERT_MEDIUM;
    else if(dist >= 100 && dist < 200) level = ALERT_FAR;

    if(level != ALERT_NONE) {
        event_t evt;
        evt.type = EVENT_OBSTACLE_DETECTED;
        evt.timestamp = millis();
        evt.data.alert_level = level;
        EventManager::getInstance().postEvent(evt);
        transitionTo(APP_STATE_ALERT);  // guard prevents re-posting every loop tick
    }

    // Return to IDLE if cane has been still for 5 seconds
    if(data.accel_magnitude < 1.05f) {
        if(still_since_detecting == 0) still_since_detecting = millis();
        if(millis() - still_since_detecting > 5000) {
            still_since_detecting = 0;
            transitionTo(APP_STATE_IDLE);
        }
    } else {
        still_since_detecting = 0;
    }
}

void AppStateMachine::onStateAlert() {
    uint32_t alert_time = millis() - state_start_time;
    
    // Stay in alert for 200ms then return to detecting
    if(alert_time > 200) {
        transitionTo(APP_STATE_DETECTING);
    }
}

void AppStateMachine::onStateSleeping() {
    // Wake up on significant movement (same threshold as IDLE → DETECTING)
    const sensor_data_t& data = SensorManager::getInstance().getSensorData();
    if(data.accel_magnitude > 1.2f) {
        transitionTo(APP_STATE_DETECTING);
    }
}
