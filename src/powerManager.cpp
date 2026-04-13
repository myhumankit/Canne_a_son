/*
 *  Gestion des modes de consommation énergétique (MODE_RUN → MODE_HIBERNATION)
 *
 *  @author BOUFALLOUS Amin
 *  @modified 2026-04-11
 */

#include "powerManager.h"
#include "peripheralManager.h"
#include "sensorManager.h"
#include "CH201.h"
#include "TFMiniPlus.h"
#include "buzzer.h"
#include "vibreur.h"
#include <Arduino.h>
#include <esp_pm.h>

extern ch_dev_t chirp_devices[];
extern TFMiniPlus TFMP;

PowerManager::PowerManager() : current_mode(MODE_RUN) {
    memset(&consumption, 0, sizeof(consumption_t));
}

bool PowerManager::init() {
    Serial.println("[POWER] Initializing power management...");
    // Force apply MODE_RUN config regardless of current_mode guard
    current_mode = MODE_STANDBY;  // temporarily different so setPowerMode() doesn't skip
    setPowerMode(MODE_RUN);
    return true;
}

void PowerManager::setPowerMode(power_mode_t mode) {
    if(mode == current_mode) return;
    
    Serial.printf("[POWER] Mode: %d → %d\n", current_mode, mode);
    current_mode = mode;
    
    // Apply configuration to all components
    configESP32(mode);
    configCH201(mode);
    configMPU6050(mode);
    configTFMiniPlus(mode);
    configBuzzer(mode);
    configVibreur(mode);

    // Mute peripherals when not in active mode (syncs PeripheralManager state)
    if(mode != MODE_RUN) {
        PeripheralManager::getInstance().muteAll();
    }
    
    printStats();
}

void PowerManager::configESP32(power_mode_t mode) {
    switch(mode) {
        case MODE_RUN:
            {
                esp_pm_config_esp32c3_t pm_config = {
                    .max_freq_mhz = 160,
                    .min_freq_mhz = 160,
                    .light_sleep_enable = false
                };
                esp_pm_configure(&pm_config);
            }
            consumption.esp32 = 150.0f;
            break;
            
        case MODE_STANDBY:
            {
                esp_pm_config_esp32c3_t pm_config = {
                    .max_freq_mhz = 80,
                    .min_freq_mhz = 80,
                    .light_sleep_enable = false
                };
                esp_pm_configure(&pm_config);
            }
            consumption.esp32 = 50.0f;
            break;
            
        case MODE_LIGHT_SLEEP:
            {
                esp_pm_config_esp32c3_t pm_config = {
                    .max_freq_mhz = 80,
                    .min_freq_mhz = 40,
                    .light_sleep_enable = true
                };
                esp_pm_configure(&pm_config);
            }
            consumption.esp32 = 10.0f;
            break;
            
        case MODE_DEEP_SLEEP:
            consumption.esp32 = 1.0f;
            break;
            
        case MODE_HIBERNATION:
            consumption.esp32 = 0.1f;
            break;
    }
}

void PowerManager::configCH201(power_mode_t mode) {
    switch(mode) {
        case MODE_RUN:
            ch_set_sample_interval(&chirp_devices[0], 100);
            ch_set_sample_interval(&chirp_devices[1], 100);
            consumption.ch201 = 30.0f;
            break;
        case MODE_STANDBY:
            ch_set_sample_interval(&chirp_devices[0], 500);
            ch_set_sample_interval(&chirp_devices[1], 500);
            consumption.ch201 = 10.0f;
            break;
        case MODE_LIGHT_SLEEP:
        case MODE_DEEP_SLEEP:
            ch_set_mode(&chirp_devices[0], CH_MODE_IDLE);
            ch_set_mode(&chirp_devices[1], CH_MODE_IDLE);
            SensorManager::getInstance().clearDistances();
            consumption.ch201 = 0.5f;
            break;
        case MODE_HIBERNATION:
            ch_set_mode(&chirp_devices[0], CH_MODE_IDLE);
            ch_set_mode(&chirp_devices[1], CH_MODE_IDLE);
            SensorManager::getInstance().clearDistances();
            consumption.ch201 = 0.1f;
            break;
    }
}

void PowerManager::configMPU6050(power_mode_t mode) {
    switch(mode) {
        case MODE_RUN:
            consumption.mpu6050 = 4.0f;
            break;
        case MODE_STANDBY:
            consumption.mpu6050 = 1.5f;
            break;
        case MODE_LIGHT_SLEEP:
            consumption.mpu6050 = 0.8f;
            break;
        case MODE_DEEP_SLEEP:
        case MODE_HIBERNATION:
            consumption.mpu6050 = 0.1f;
            break;
    }
}

void PowerManager::configTFMiniPlus(power_mode_t mode) {
    switch(mode) {
        case MODE_RUN:
            // Note: TFMiniPlus frame rate is set during begin(), cannot be changed dynamically
            consumption.tfminiplus = 80.0f;
            break;
        case MODE_STANDBY:
            // Note: TFMiniPlus frame rate is set during begin(), cannot be changed dynamically
            consumption.tfminiplus = 40.0f;
            break;
        case MODE_LIGHT_SLEEP:
            // Note: TFMiniPlus frame rate is set during begin(), cannot be changed dynamically
            consumption.tfminiplus = 20.0f;
            break;
        case MODE_DEEP_SLEEP:
        case MODE_HIBERNATION:
            // Note: TFMiniPlus frame rate is set during begin(), cannot be changed dynamically
            consumption.tfminiplus = 5.0f;
            break;
    }
}

void PowerManager::configBuzzer(power_mode_t mode) {
    consumption.buzzer = (mode == MODE_RUN) ? 20.0f : 0.0f;
}

void PowerManager::configVibreur(power_mode_t mode) {
    consumption.vibreur = (mode == MODE_RUN) ? 15.0f : 0.0f;
}

void PowerManager::printStats() {
    consumption.total = consumption.esp32 + consumption.ch201 + 
                       consumption.mpu6050 + consumption.tfminiplus + 
                       consumption.buzzer + consumption.vibreur;
    
    Serial.println("┌─ [POWER STATS] ─────────────┐");
    Serial.printf("│ ESP32    : %6.1f mA       │\n", consumption.esp32);
    Serial.printf("│ CH201    : %6.1f mA       │\n", consumption.ch201);
    Serial.printf("│ MPU6050  : %6.1f mA       │\n", consumption.mpu6050);
    Serial.printf("│ LIDAR    : %6.1f mA       │\n", consumption.tfminiplus);
    Serial.printf("│ Buzzer   : %6.1f mA       │\n", consumption.buzzer);
    Serial.printf("│ Vibreur  : %6.1f mA       │\n", consumption.vibreur);
    Serial.println("├────────────────────────────┤");
    Serial.printf("│ TOTAL    : %6.1f mA       │\n", consumption.total);
    
    if(consumption.total > 0) {
        float battery_hours = 3000.0f / consumption.total;
        Serial.printf("│ Batterie : %5.1f heures    │\n", battery_hours);
    }
    Serial.println("└────────────────────────────┘");
}
