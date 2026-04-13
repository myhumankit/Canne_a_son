# Changelog v2 → v3 — Canne à Son

> Ce document décrit les changements apportés lors de la refactorisation vers l'architecture v3.
> La migration est **terminée**. Ce fichier sert de référence historique.
>
> **Auteur :** BOUFALLOUS Amin — Avril 2026

---

## Résumé

| Axe | v2 | v3 |
|-----|----|----|
| Architecture | Monolithique (main.cpp ~300 lignes) | Singletons + bus d'événements + machine d'états |
| Gestion capteurs | Inline dans loop() | `SensorManager::update()` toutes les 100 ms via ISR |
| Alertes | `son.makeSound(buz_active[])` + logique ad-hoc | `PeripheralManager::triggerAlert(level, duration_ms)` |
| Modes énergie | Aucun | 5 modes (RUN → HIBERNATION) via `PowerManager` |
| Gestion mouvement | Détection pitch/roll dans loop() | Machine d'états IDLE/DETECTING/ALERT/SLEEPING |

---

## Fichiers supprimés

| Fichier | Raison |
|---------|--------|
| `src/sound.cpp` | Définissait `buzzer bzz` et `vibreur vzz` en doublon avec `peripheralManager.cpp` — erreur de lien "multiple definition". Toute la logique sonore/haptique est désormais dans `PeripheralManager`. |
| `src/main_old.cpp` (si présent) | Remplacé par la nouvelle architecture. |

---

## Nouveaux fichiers

| Fichier | Rôle |
|---------|------|
| `include/types.h` | Tous les enums et structs partagés (`event_t`, `sensor_data_t`, `alert_level_t`, etc.) |
| `include/eventManager.h` | Bus d'événements singleton — queue circulaire 16 événements, 8 callbacks/type |
| `include/sensorManager.h` / `src/sensorManager.cpp` | Agrégation des lectures CH201 × 2, MPU6050, TFMiniPlus |
| `include/peripheralManager.h` / `src/peripheralManager.cpp` | Buzzer + vibreur avec timeouts automatiques |
| `include/powerManager.h` / `src/powerManager.cpp` | 5 modes de consommation |
| `include/appStateMachine.h` / `src/appStateMachine.cpp` | Machine d'états INIT → IDLE → DETECTING → ALERT → SLEEPING |
| `include/boardConfig.h` | Centralise toutes les constantes GPIO et hardware |
| `include/sensorTemplate.h` | Base template pour ajouter de nouveaux capteurs |

---

## Corrections appliquées (v3 post-refacto)

### Bugs critiques corrigés

| Fichier | Problème | Fix |
|---------|----------|-----|
| `CH201.cpp` | `distance[dev_num+1]` → écriture hors tableau (index 2 sur taille 2) | `distance[dev_num]` |
| `CH201.cpp` | ISRs sans `IRAM_ATTR` → crash potentiel lors de lectures flash I2C | Ajout `IRAM_ATTR` sur les deux ISRs |
| `CH201.cpp` | `ledcWrite(0/1, 0)` dans `sendReceiveCH201()` → coupait les alertes en cours | Supprimé |
| `sensorManager.cpp` | `!IMU.Init()` → logique inversée (`Init()` retourne 0 = succès) | `IMU.Init() != 0` |
| `sensorManager.cpp` | `TFMP.getData()` → méthode inexistante | `TFMP.getDistance()` |
| `MPU6050_custom.cpp` | `IMU` déclaré `extern` partout mais jamais défini | `MPU6050_custom IMU;` ajouté dans `MPU6050_custom.cpp` |
| `MPU6050_custom.cpp` | `while(status!=0){}` → boucle infinie si Init échoue | `if(status != 0) return status;` |
| `appStateMachine.cpp` | `onStateSleeping()` vide → impossible de quitter le mode SLEEPING | Ajout check `accel_magnitude > 1.2f` |
| `appStateMachine.cpp` | `still_since` variable statique locale → valeur corrompue entre passages ALERT→DETECTING | Promu en membre `still_since_detecting`, remis à 0 dans `transitionTo(DETECTING)` |
| `powerManager.cpp` | `current_mode` initialisé à `MODE_RUN` → `setPowerMode(MODE_RUN)` dans `init()` retournait immédiatement sans configurer | `current_mode = MODE_STANDBY` avant l'appel |
| `powerManager.cpp` | Distances CH201 persistaient après mise en idle des capteurs → fausse alerte CLOSE au réveil | `clearDistances()` appelé dans `configCH201()` pour LIGHT_SLEEP/DEEP_SLEEP/HIBERNATION |
| `vibreur.cpp` | `ledcWrite(channel, 1024)` → dépasse le max 10-bit | `ledcWrite(channel, 1023)` |
| `eventManager.h` | Accès direct à `Callback::func` (membre privé) → erreur de compilation | Ajout `isSet()` public sur `Callback<T>` |
| `CH201.cpp` | `char *mode_string = "IDLE"` → interdit en C++11 | `const char *mode_string` |

### Améliorations fonctionnelles

| Fichier | Changement |
|---------|-----------|
| `MPU6050_custom.cpp` | Ajout `mpu.calcOffsets()` après `mpu.begin()` — calibration automatique au démarrage |
| `sensorManager.cpp` | Double appel `sendReceiveCH201()` par cycle — traite TIMER_FLAG et DATA_READY_FLAG dans le même cycle 100 ms |
| `appStateMachine.cpp` | Si IMU absent → démarrage en `APP_STATE_DETECTING` (fallback sécurité : détection permanente) |
| `peripheralManager.cpp` | Ajout `signalError()` — 3 bips courts 800 Hz pour signaler un échec de capteur à l'utilisateur malvoyant |
| `sensorManager.cpp` | Appel automatique de `signalError()` si au moins un capteur échoue au démarrage |

---

## Comparaison logique détection (v2 → v3)

### Avant — v2 (inline dans loop)

```cpp
void loop() {
    if(sendReceiveCH201(distance_measured, amplitude) == 2) {
        IMU.getAttitude(&pitch, &roll);
        if(pitch > -70 && pitch < -10) {
            if(distance_measured[0] < 160 && buz_active[0] == 0) {
                buz_active[0] = 1;
            }
            son.makeSound(buz_active);
        }
    }
}
```

### Après — v3 (délégué aux managers)

```cpp
void loop() {
    if(sensor_ready_flag) {
        sensor_ready_flag = false;
        SensorManager::getInstance().update();   // lit CH201, MPU, LiDAR
        EventManager::getInstance().postEvent(/* EVENT_SENSOR_READY */);
    }
    EventManager::getInstance().processEvents(); // dispatch callbacks
    AppStateMachine::getInstance().update();     // logique IDLE/DETECTING/ALERT
    PeripheralManager::getInstance().update();   // timeout buzzer/vibreur
    delay(1);
}
```

---

## Ce qui reste à implémenter

| Feature | État | Notes |
|---------|------|-------|
| Vrai deep sleep ESP32 | ⚠️ Simulé | `esp_deep_sleep_start()` non appelé — nécessite configuration wakeup GPIO (ex: INT du MPU6050) |
| Tests automatisés | ❌ Non prévu | Validation sur hardware uniquement |
