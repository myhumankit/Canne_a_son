# Canne à Son — v3

> Canne blanche électronique pour personnes malvoyantes.
> Détecte les obstacles et escaliers via capteurs ultrasons (CH201) et LiDAR (TFMiniPlus), et alerte l'utilisateur par buzzer et vibration.
>
> **Auteur :** BOUFALLOUS Amin — Avril 2026
> **Plateforme :** ESP32-C3 · PlatformIO · Arduino Framework

---

## Matériel

| Composant | Rôle | Interface |
|-----------|------|-----------|
| ESP32-C3 DevKitM-1 | MCU principal | — |
| CH201 × 2 | Ultrasons jusqu'à 4 m | I2C (SDA=6, SCL=7) |
| TFMiniPlus | LiDAR 0–12 m | I2C |
| MPU6050 | IMU (accéléro + gyro) | I2C |
| Buzzer | Alerte sonore | LEDC PWM (GPIO 0) |
| Vibreur | Alerte haptique | LEDC PWM (GPIO 10) |

---

## Démarrage rapide

### Prérequis
- [PlatformIO](https://platformio.org/) installé (CLI ou extension VS Code)

### Build & Flash

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

### Sortie série attendue au démarrage

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

> Si un capteur échoue → 3 bips courts au démarrage + message `❌` sur le port série.
> Si le MPU6050 est absent → la canne reste en mode DETECTING en permanence (sécurité).

---

## Architecture v3

L'architecture repose sur **5 Singletons**, un **bus d'événements** et une **machine d'états**.
Voir [ARCHITECTURE_v3.md](ARCHITECTURE_v3.md) pour le détail complet.

### Flux d'exécution (boucle principale)

```
Timer ISR (100 ms)
    ↓ sensor_ready_flag = true
SensorManager::update()      — lit CH201 × 2, MPU6050, TFMiniPlus
EventManager::postEvent()    — enqueue EVENT_SENSOR_READY
EventManager::processEvents()— dispatch callbacks
AppStateMachine::update()    — logique état courant
PeripheralManager::update()  — gère timeouts buzzer/vibreur
delay(1)
```

### Machine d'états

```
INIT → IDLE ──(mouvement IMU)──→ DETECTING ──(obstacle)──→ ALERT
                ↑                      |                      |
                └──(5s immobile)───────┘                      |
                                  ←──────────(200ms)──────────┘
IDLE ──(2 min sans mouvement)──→ SLEEPING ──(mouvement IMU)──→ DETECTING
```

### Niveaux d'alerte

| Niveau | Distance | Buzzer | Vibration |
|--------|----------|--------|-----------|
| `ALERT_CLOSE` | < 50 cm | 4000 Hz | 255/255 |
| `ALERT_MEDIUM` | 50–100 cm | 2500 Hz | 200/255 |
| `ALERT_FAR` | 100–200 cm | 1500 Hz | 150/255 |

---

## Structure du projet

```
include/
├── types.h              — Tous les enums et structs partagés
├── eventManager.h       — Bus d'événements (singleton, queue x16)
├── sensorManager.h      — Lecture capteurs (singleton)
├── peripheralManager.h  — Buzzer + vibreur (singleton)
├── powerManager.h       — Modes de consommation (singleton)
├── appStateMachine.h    — Machine d'états (singleton)
├── boardConfig.h        — Pins et constantes hardware
└── sensorTemplate.h     — Base template pour nouveaux capteurs

src/
├── main.cpp             — Orchestrateur (~50 lignes)
├── sensorManager.cpp
├── peripheralManager.cpp
├── powerManager.cpp
├── appStateMachine.cpp
├── buzzer.cpp
├── vibreur.cpp
├── CH201.cpp
├── MPU6050_custom.cpp
└── TFMiniPlus.cpp

lib/
├── SmartsonicCH201/     — Driver CH201 (Chirp SonicLib)
└── MPU6050/             — MPU6050_light
```

---

## Règles de développement

- **Jamais de `delay()` dans `loop()`** — tout le timing passe par le flag ISR ou les timeouts de `PeripheralManager`.
- **Types partagés dans `types.h`** — pas d'enums/structs dans les headers individuels.
- **Accès via `::getInstance()`** — jamais d'instanciation directe des managers.
- **Nouveau capteur** → implémenter avec [include/sensorTemplate.h](include/sensorTemplate.h), ajouter un masque dans `SensorManager`, lire dans `update()`.
- **Nouvel événement** → ajouter `event_type_t` dans `types.h`, poster via `postEvent()`, s'abonner via `subscribe()` dans `setup()`.

---

## État du projet

| Feature | État |
|---------|------|
| Détection obstacles CH201 × 2 | ✅ Fonctionnel |
| Alerte buzzer 3 niveaux | ✅ Fonctionnel |
| Alerte vibreur 3 niveaux | ✅ Fonctionnel |
| Détection mouvement IMU | ✅ Fonctionnel |
| Machine d'états IDLE/DETECTING/ALERT/SLEEPING | ✅ Fonctionnel |
| Fallback si IMU absent | ✅ Fonctionnel |
| Signalement erreur capteur (bips) | ✅ Fonctionnel |
| Modes économie énergie (STANDBY/LIGHT_SLEEP) | ✅ Fonctionnel |
| Vrai deep sleep ESP32 | ⚠️ Simulé — wakeup GPIO à implémenter |
| Tests automatisés | ❌ Validation sur hardware uniquement |

---

## Ressources

- [ARCHITECTURE_v3.md](ARCHITECTURE_v3.md) — Conception détaillée et diagrammes
- [QUICK_START.md](QUICK_START.md) — Guide développeur (5 min)
- [include/config_example.h](include/config_example.h) — Exemples de configuration
- [Wikilab My Human Kit](https://wikilab.myhumankit.org/index.php?title=Projets:Canne_a_son_V2) — Documentation projet v2
