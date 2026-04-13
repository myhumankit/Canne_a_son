/*
 * CONFIG_EXAMPLE.h
 * 
 * Fichier d'exemple montrant comment utiliser les nouveaux modes
 * et maintenir la rétrocompatibilité avec le code existant
 */

#ifndef CONFIG_EXAMPLE_H
#define CONFIG_EXAMPLE_H

#include "types.h"
#include "powerManager.h"
#include "sensorManager.h"
#include "peripheralManager.h"
#include "appStateMachine.h"

// ========== MODES DE CONFIGURATION ==========

// Mode 1: Application Normale (Défaut)
inline void configureNormalMode() {
    PowerManager::getInstance().setPowerMode(MODE_RUN);
    // Tous capteurs actifs, CPU 160MHz
    // Batterie : 25 heures
}

// Mode 2: Mode Économie (Utilisateur assis/voiture)
inline void configureEconomyMode() {
    PowerManager::getInstance().setPowerMode(MODE_STANDBY);
    // Capteurs réduits, CPU 80MHz
    // Batterie : 85+ heures
}

// Mode 3: Mode Repos (Canne posée sur table)
inline void configureRestMode() {
    PowerManager::getInstance().setPowerMode(MODE_LIGHT_SLEEP);
    // Mesures très rares, CPU dort
    // Batterie : 12+ jours
}

// Mode 4: Mode Hibernation (Canne au placard)
inline void configureHibernationMode() {
    PowerManager::getInstance().setPowerMode(MODE_HIBERNATION);
    // Tout éteint sauf RTC
    // Batterie : 1 an
}

// ========== DÉTECTION AUTOMATIQUE DE MODE ==========

/*
 * Cette fonction doit être appelée régulièrement dans la boucle principale
 * pour détecter automatiquement le changement de mode selon l'activité
 */
inline void autoDetectMode(const sensor_data_t& sensor_data) {
    // Calcul de l'amplitude du mouvement
    float accel_mag = sensor_data.accel_magnitude;
    
    static unsigned long last_movement_time = millis();
    static power_mode_t last_mode = MODE_RUN;
    
    unsigned long now = millis();
    unsigned long time_idle = now - last_movement_time;
    
    // Détection de mouvement (> 0.7 G)
    if(accel_mag > 0.7f) {
        last_movement_time = now;
        
        if(last_mode != MODE_RUN) {
            PowerManager::getInstance().setPowerMode(MODE_RUN);
            last_mode = MODE_RUN;
        }
    } else {
        // Pas de mouvement - adapter mode selon durée
        if(time_idle > 300000) {  // 5 minutes
            if(last_mode != MODE_HIBERNATION) {
                PowerManager::getInstance().setPowerMode(MODE_HIBERNATION);
                last_mode = MODE_HIBERNATION;
            }
        } else if(time_idle > 120000) {  // 2 minutes
            if(last_mode != MODE_DEEP_SLEEP) {
                PowerManager::getInstance().setPowerMode(MODE_DEEP_SLEEP);
                last_mode = MODE_DEEP_SLEEP;
            }
        } else if(time_idle > 30000) {  // 30 secondes
            if(last_mode != MODE_LIGHT_SLEEP) {
                PowerManager::getInstance().setPowerMode(MODE_LIGHT_SLEEP);
                last_mode = MODE_LIGHT_SLEEP;
            }
        } else if(time_idle > 5000) {  // 5 secondes
            if(last_mode != MODE_STANDBY) {
                PowerManager::getInstance().setPowerMode(MODE_STANDBY);
                last_mode = MODE_STANDBY;
            }
        }
    }
}

// ========== EXEMPLE D'INTÉGRATION DANS MAIN ==========

/*
 * void setup() {
 *     // ... initialization ...
 *     configureNormalMode();
 * }
 * 
 * void loop() {
 *     if(sensor_ready_flag) {
 *         sensor_ready_flag = false;
 *         
 *         SensorManager::getInstance().update();
 *         
 *         if(SensorManager::getInstance().isDataReady()) {
 *             const sensor_data_t& data = 
 *                 SensorManager::getInstance().getSensorData();
 *             
 *             // Détection auto de mode
 *             autoDetectMode(data);
 *             
 *             // Analyser obstacle
 *             if(data.distance_ch201[0] < 50) {
 *                 PeripheralManager::getInstance()
 *                     .triggerAlert(ALERT_CLOSE, 100);
 *             }
 *         }
 *     }
 *     
 *     EventManager::getInstance().processEvents();
 *     AppStateMachine::getInstance().update();
 *     PeripheralManager::getInstance().update();
 *     
 *     delay(1);
 * }
 */

// ========== CALLBACK PERSONNALISÉ ==========

/*
 * Pour ajouter une logique personnalisée lors d'un événement :
 */
void myCustomObstacleCallback(const event_t* evt) {
    // Votre logique personnalisée
    Serial.printf("[CUSTOM] Obstacle détecté à: %d cm\n", 
                  evt->data.sensor_data.distance_ch201[0]);
    
    // Pouvez déclencher alarme personnalisée
    // PeripheralManager::getInstance().triggerAlert(...);
}

// Enregistrer dans setup():
// EventManager::getInstance().subscribe(EVENT_OBSTACLE_DETECTED, 
//                                       myCustomObstacleCallback);

#endif
