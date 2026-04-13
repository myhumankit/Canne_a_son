#include "peripheralManager.h"
#include "buzzer.h"
#include "vibreur.h"
#include <Arduino.h>

// Global instances (minimal)
buzzer bzz(BUZZER_PIN, 2000, 0, 10);
vibreur vzz(VIBREUR_PIN, 20000, 1, 10);

PeripheralManager::PeripheralManager() {
    memset(&state, 0, sizeof(state));
}

bool PeripheralManager::init() {
    Serial.println("[PERIPH] Initializing peripherals...");

    bzz.bipSystemReady();
    delay(100);
    vzz.bipSystemReady();

    Serial.println("  ✓ Buzzer ready");
    Serial.println("  ✓ Vibreur ready");

    return true;
}

void PeripheralManager::signalError() {
    // 3 short rapid beeps — signals sensor init failure to user
    for(int i = 0; i < 3; i++) {
        bzz.startBip(800);
        vzz.startBip(200);
        delay(100);
        bzz.stopBip();
        vzz.stopBip();
        delay(100);
    }
}

void PeripheralManager::update() {
    uint32_t now = millis();
    
    // Auto-stop buzzer if timeout
    if(state.buzzer_active && now >= state.buzzer_end_time) {
        bzz.stopBip();
        state.buzzer_active = false;
    }
    
    // Auto-stop vibreur if timeout
    if(state.vibreur_active && now >= state.vibreur_end_time) {
        vzz.stopBip();
        state.vibreur_active = false;
    }
}

void PeripheralManager::triggerAlert(alert_level_t level, uint16_t duration_ms) {
    playBuzz(level);
    playVibration(level);
    
    state.buzzer_end_time = millis() + duration_ms;
    state.vibreur_end_time = millis() + duration_ms;
}

void PeripheralManager::stopAlert() {
    bzz.stopBip();
    vzz.stopBip();
    state.buzzer_active = false;
    state.vibreur_active = false;
}

void PeripheralManager::muteAll() {
    stopAlert();
}

void PeripheralManager::playBuzz(alert_level_t level) {
    int freq = 1500;
    
    switch(level) {
        case ALERT_CLOSE:
            freq = 4000;
            break;
        case ALERT_MEDIUM:
            freq = 2500;
            break;
        case ALERT_FAR:
            freq = 1500;
            break;
        default:
            return;
    }
    
    bzz.startBip(freq);
    state.buzzer_active = true;
}

void PeripheralManager::playVibration(alert_level_t level) {
    int intensity = 128;
    
    switch(level) {
        case ALERT_CLOSE:
            intensity = 255;
            break;
        case ALERT_MEDIUM:
            intensity = 200;
            break;
        case ALERT_FAR:
            intensity = 150;
            break;
        default:
            return;
    }
    
    vzz.startBip(intensity);
    state.vibreur_active = true;
}
