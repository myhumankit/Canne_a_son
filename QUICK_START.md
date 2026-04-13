# Quick Start — Canne à Son v3

> Guide développeur — mise en route en 5 minutes.
>
> **Auteur :** BOUFALLOUS Amin — Avril 2026

---

## Prérequis

- [PlatformIO](https://platformio.org/) installé (CLI ou extension VS Code)
- ESP32-C3 DevKitM-1 connecté en USB

---

## Build & Flash

```bash
# Compiler
pio run

# Flasher
pio run --target upload

# Moniteur série (9600 baud)
pio device monitor --baud 9600

# Tout en une commande
pio run --target upload && pio device monitor --baud 9600
```

> Ne pas utiliser `-e esp32-c3` — l'environnement s'appelle `esp32c3` dans `platformio.ini`.

---

## Sortie série attendue au démarrage

```
╔════════════════════════════════════════╗
║  ✓ CANNE À SON - Architecture V3       ║
║  ✓ Optimisé + Événementiel             ║
╚════════════════════════════════════════╝

[SENSOR] Initializing all sensors...
  ✓ CH201 ready
  ✓ MPU6050 ready (offsets calibrated)
  ✓ LIDAR ready
[PERIPH] Initializing peripherals...
  ✓ Buzzer ready
  ✓ Vibreur ready
[POWER] Initializing power management...
✓ System ready
```

Si un capteur échoue → 3 bips courts 800 Hz + message `❌` sur le port série.
Si le MPU6050 est absent → la canne reste en mode `DETECTING` en permanence (sécurité).

---

## Les 5 composants clés

### SensorManager — lire les capteurs

```cpp
// Dans loop(), après sensor_ready_flag
SensorManager::getInstance().update();

const sensor_data_t& data = SensorManager::getInstance().getSensorData();
uint16_t dist_ch201 = data.distance_ch201[0];  // mm
int16_t  dist_lidar = data.distance_lidar;      // cm
float    accel      = data.accel_magnitude;     // g (≈1.0 au repos)
```

### PeripheralManager — buzzer + vibreur

```cpp
// Déclencher une alerte avec durée
PeripheralManager::getInstance().triggerAlert(ALERT_CLOSE, 200);   // 200 ms
PeripheralManager::getInstance().triggerAlert(ALERT_MEDIUM, 200);
PeripheralManager::getInstance().triggerAlert(ALERT_FAR, 200);

// Arrêt immédiat
PeripheralManager::getInstance().muteAll();

// Appeler dans loop() pour gérer les timeouts automatiques
PeripheralManager::getInstance().update();
```

### EventManager — bus d'événements

```cpp
// S'abonner (dans setup())
EventManager::getInstance().subscribe(EVENT_OBSTACLE_DETECTED, [](const event_t& e) {
    // e.data.alert_level disponible
});

// Dispatcher (dans loop())
EventManager::getInstance().processEvents();
```

> L'`AppStateMachine` poste elle-même `EVENT_OBSTACLE_DETECTED` et `EVENT_IMU_MOVED`.
> Ne pas reposter ces événements manuellement depuis `loop()`.

### AppStateMachine — machine d'états

```cpp
// L'AppStateMachine est autonome — appeler dans loop()
AppStateMachine::getInstance().update();

// Lire l'état courant si besoin
app_state_t state = AppStateMachine::getInstance().getState();
```

États disponibles : `APP_STATE_INIT`, `APP_STATE_IDLE`, `APP_STATE_DETECTING`, `APP_STATE_ALERT`, `APP_STATE_SLEEPING`.

### PowerManager — consommation

```cpp
PowerManager::getInstance().setPowerMode(MODE_RUN);       // 160 MHz, CH201 100 ms
PowerManager::getInstance().setPowerMode(MODE_STANDBY);   // 80 MHz, CH201 500 ms
PowerManager::getInstance().setPowerMode(MODE_LIGHT_SLEEP); // DFS, CH201 idle
```

Le `PowerManager` est appelé automatiquement par l'`AppStateMachine` lors des transitions d'état.

---

## Structure de main.cpp

`main.cpp` est un orchestrateur pur (~50 lignes). Ne pas y mettre de logique applicative.

```cpp
#include <Arduino.h>
#include "eventManager.h"
#include "sensorManager.h"
#include "peripheralManager.h"
#include "powerManager.h"
#include "appStateMachine.h"

volatile bool sensor_ready_flag = false;

void IRAM_ATTR onTimer() {
    sensor_ready_flag = true;
}

void setup() {
    Serial.begin(9600);

    SensorManager::getInstance().init();
    PeripheralManager::getInstance().init();
    PowerManager::getInstance().init();
    AppStateMachine::getInstance().init();

    // Timer 100 ms pour déclenchement capteurs
    // (voir main.cpp pour implémentation complète esp_timer)

    Serial.println("✓ System ready");
}

void loop() {
    if(sensor_ready_flag) {
        sensor_ready_flag = false;
        SensorManager::getInstance().update();
    }

    EventManager::getInstance().processEvents();
    AppStateMachine::getInstance().update();
    PeripheralManager::getInstance().update();

    delay(1);
}
```

---

## Ajouter un capteur

1. Créer `include/MonCapteur.h` et `src/MonCapteur.cpp` en s'inspirant de [include/sensorTemplate.h](include/sensorTemplate.h)
2. Ajouter un masque dans `sensorManager.h` : `static const uint8_t SENSOR_MON_CAPTEUR = (1 << 4);`
3. Ajouter `readMonCapteur()` dans `SensorManager` et l'appeler dans `update()`
4. Ajouter le champ de données dans `sensor_data_t` (types.h)

## Ajouter un événement

1. Ajouter `EVENT_MON_EVENEMENT` dans `event_type_t` (types.h)
2. Poster : `EventManager::getInstance().postEvent(evt);`
3. S'abonner dans `setup()` : `EventManager::getInstance().subscribe(EVENT_MON_EVENEMENT, maCallback);`

---

## Règles à respecter

- **Jamais de `delay()` dans `loop()`** — le timing passe par le flag ISR ou les timeouts de `PeripheralManager`.
- **Accès via `::getInstance()`** — jamais d'instanciation directe.
- **Types partagés dans `types.h`** — pas d'enums/structs dans les headers individuels.
- **Ne pas appeler `postEvent(EVENT_OBSTACLE_DETECTED)` depuis `loop()`** — l'`AppStateMachine` le fait.

---

## Ressources

- [README.md](README.md) — Vue d'ensemble projet
- [README_ARCHITECTURE.md](README_ARCHITECTURE.md) — Référence technique complète
- [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md) — Changelog v2 → v3
- [include/sensorTemplate.h](include/sensorTemplate.h) — Base pour nouveaux capteurs
