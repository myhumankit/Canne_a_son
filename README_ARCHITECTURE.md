# Architecture Technique — Canne à Son v3

> Référence technique détaillée des composants logiciels.
> Pour la vue d'ensemble projet, voir [README.md](README.md).
>
> **Auteur :** BOUFALLOUS Amin — Avril 2026

---

## Principes de conception

| Pattern | Où | Pourquoi |
|---------|-----|---------|
| **Singleton** | Tous les managers | Une seule instance, accès global sûr, pas de duplication mémoire |
| **Event-Driven** | EventManager | Découplage total entre composants |
| **State Machine** | AppStateMachine | Comportement prévisible, pas de race conditions |
| **Non-blocking I/O** | ISR + flags | CPU toujours réactif, batterie préservée |
| **Template C++** | Callback, SensorBase | Type-safe, zéro overhead runtime |

---

## Composants

### `EventManager` — Bus d'événements

- Queue circulaire de **16 événements** (FIFO)
- Jusqu'à **8 callbacks** par type d'événement
- Thread-safe sur ESP32-C3 (single-core) : les callbacks s'exécutent dans le contexte `loop()`, jamais en ISR

```cpp
// S'abonner
EventManager::getInstance().subscribe(EVENT_OBSTACLE_DETECTED, onObstacleDetected);

// Poster
event_t evt = { EVENT_OBSTACLE_DETECTED, millis(), {.alert_level = ALERT_CLOSE} };
EventManager::getInstance().postEvent(evt);

// Dispatcher (à appeler dans loop)
EventManager::getInstance().processEvents();
```

**Types d'événements disponibles (`types.h`) :**

| Événement | Déclenché par | Payload |
|-----------|--------------|---------|
| `EVENT_SENSOR_READY` | `SensorManager::update()` | `sensor_data_t` |
| `EVENT_OBSTACLE_DETECTED` | `AppStateMachine::onStateDetecting()` | `alert_level_t` |
| `EVENT_OBSTACLE_CLEARED` | (réservé) | — |
| `EVENT_IMU_MOVED` | `onStateIdle()` / `onStateSleeping()` | — |
| `EVENT_IMU_STILL` | (réservé) | — |
| `EVENT_LOW_BATTERY` | (réservé) | — |
| `EVENT_MODE_CHANGED` | `PowerManager::setPowerMode()` | `power_mode_t` |

---

### `SensorManager` — Lecture capteurs

Lit tous les capteurs actifs toutes les 100 ms, déclenché par le flag ISR.

```
SensorManager::update()
    ├── readCH201()    — sendReceiveCH201() x2 (trigger + read dans le même cycle)
    ├── readMPU6050()  — getAttitude() + getAccelMagnitude()
    └── readLIDAR()    — TFMP.getDistance()
```

**Masques de capteurs :**

| Constante | Bit | Capteur |
|-----------|-----|---------|
| `SENSOR_CH201_0` | 0 | CH201 capteur 0 |
| `SENSOR_CH201_1` | 1 | CH201 capteur 1 |
| `SENSOR_MPU6050` | 2 | IMU |
| `SENSOR_LIDAR` | 3 | TFMiniPlus |

**Données produites (`sensor_data_t`) :**

```cpp
typedef struct {
    unsigned int distance_ch201[2];  // mm, 0 = pas de cible
    unsigned int amplitude_ch201[2];
    int16_t      distance_lidar;     // cm, -1 = erreur
    float        pitch;              // degrés
    float        roll;               // degrés
    float        accel_magnitude;    // g (≈1.0 au repos)
    uint8_t      confidence;
    uint32_t     timestamp;          // millis()
} sensor_data_t;
```

**Comportement si capteur absent :**
- CH201 absent → `distance_ch201[0/1] = 0` → aucune alerte
- MPU6050 absent → `imu_available = false` → état DETECTING permanent (fallback sécurité)
- LiDAR absent → `distance_lidar` non mis à jour

---

### `AppStateMachine` — Machine d'états

```
          ┌─────────────────────────────────────────┐
          │              INIT                       │
          │  (→ IDLE ou DETECTING si IMU absent)    │
          └────────────────┬────────────────────────┘
                           │
          ┌────────────────▼────────────────────────┐
          │              IDLE                       │
          │  Attente mouvement (MODE_STANDBY)        │
          │  → SLEEPING après 2 min                 │
          │  → DETECTING si accel > 1.2g            │
          └──────────┬──────────────────────────────┘
                     │ accel > 1.2g
          ┌──────────▼──────────────────────────────┐
          │           DETECTING                     │
          │  Scan actif (MODE_RUN)                  │
          │  → ALERT si obstacle détecté            │
          │  → IDLE si immobile 5 s                 │
          └──────────┬──────────────────────────────┘
                     │ obstacle < 200 cm
          ┌──────────▼──────────────────────────────┐
          │             ALERT                       │
          │  Buzzer + vibreur actifs (MODE_RUN)     │
          │  → DETECTING après 200 ms               │
          └─────────────────────────────────────────┘

          ┌─────────────────────────────────────────┐
          │           SLEEPING                      │
          │  Capteurs CH201 en idle (MODE_DEEP_SLEEP│
          │  → DETECTING si accel > 1.2g            │
          └─────────────────────────────────────────┘
```

**Transitions déclenchées par événements (`onEvent`) :**

| Événement | État source | État cible |
|-----------|------------|-----------|
| `EVENT_OBSTACLE_DETECTED` | tout | `ALERT` |
| `EVENT_OBSTACLE_CLEARED` | tout | `DETECTING` |
| `EVENT_IMU_MOVED` | tout | `DETECTING` |

---

### `PeripheralManager` — Sorties

Gère le buzzer (LEDC canal 0) et le vibreur (LEDC canal 1) avec **timeouts automatiques** via `update()`.

```cpp
triggerAlert(ALERT_CLOSE, 200);  // buzz 4000 Hz + vibration 255 pendant 200ms
muteAll();                        // arrêt immédiat + sync état interne
signalError();                    // 3 bips courts 800 Hz (erreur capteur)
```

**Correspondance fréquences :**

| Niveau | Fréquence buzzer | Duty vibreur | Signification |
|--------|-----------------|-------------|---------------|
| `ALERT_CLOSE` | 4000 Hz | 255 | Obstacle < 50 cm |
| `ALERT_MEDIUM` | 2500 Hz | 200 | Obstacle 50–100 cm |
| `ALERT_FAR` | 1500 Hz | 150 | Obstacle 100–200 cm |

---

### `PowerManager` — Gestion énergie

| Mode | CPU | CH201 | Consommation estimée |
|------|-----|-------|---------------------|
| `MODE_RUN` | 160 MHz | 100 ms | ~300 mA |
| `MODE_STANDBY` | 80 MHz | 500 ms | ~105 mA |
| `MODE_LIGHT_SLEEP` | 40–80 MHz (DFS) | Idle | ~35 mA |
| `MODE_DEEP_SLEEP` | — | Idle | ~7 mA (simulé) |
| `MODE_HIBERNATION` | — | Idle | ~5 mA (simulé) |

> `MODE_DEEP_SLEEP` et `MODE_HIBERNATION` désactivent les capteurs et coupent les alertes, mais `esp_deep_sleep_start()` n'est pas encore appelé (nécessite configuration wakeup GPIO).

---

### Timers hardware (ESP32-C3)

| Timer | Groupe | Période | Rôle |
|-------|--------|---------|------|
| `timer_sensor` (main.cpp) | Group0 / T0 | 100 ms | Set `sensor_ready_flag` |
| `My_timer` (CH201.cpp) | Group0 / T1 | 100 ms | Déclenche mesure CH201 |

Les deux ISRs sont marquées `IRAM_ATTR` pour éviter les crashes lors des lectures flash I2C.

---

### I2C — Partage du bus

Tous les capteurs partagent le même bus I2C (SDA=GPIO 6, SCL=GPIO 7, 100 kHz).
`Wire.begin(PIN_SDA, PIN_SCL, UC_FREQUENCY)` est appelé dans `chbsp_board_init()` (CH201 BSP).
Les accès I2C se font séquentiellement dans `SensorManager::update()` — aucun risque de collision (ESP32-C3 = single-core, les ISRs ne font qu'écrire un flag).

---

## Ajouter un capteur

1. Créer `include/MonCapteur.h` et `src/MonCapteur.cpp` en suivant [include/sensorTemplate.h](include/sensorTemplate.h)
2. Ajouter un masque dans `sensorManager.h` : `static const uint8_t SENSOR_MON_CAPTEUR = (1 << 4);`
3. Ajouter `readMonCapteur()` dans `SensorManager` et l'appeler dans `update()`
4. Ajouter le champ de données dans `sensor_data_t` (types.h)

## Ajouter un événement

1. Ajouter `EVENT_MON_EVENEMENT` dans `event_type_t` (types.h)
2. Poster : `EventManager::getInstance().postEvent(evt);`
3. S'abonner dans `setup()` : `EventManager::getInstance().subscribe(EVENT_MON_EVENEMENT, maCallback);`

## Ajouter un mode de puissance

1. Ajouter la valeur dans `power_mode_t` (types.h)
2. Implémenter les `case` dans chaque `configXxx()` de `PowerManager`
