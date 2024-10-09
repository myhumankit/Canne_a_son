#include <Arduino.h>
#include "TFMiniPlus.h"
#include "boardConfig.h"
#include "CH201.h"
#include "MPU6050_custom.h"
#include "sound.h"


/*
 *  Constructors call
 */
MPU6050_custom IMU;
sound son;


/*
 *  Global variables
 */
unsigned int distance_measured[NB_CH201_SENSOR+1];
unsigned int distance_old_data[NB_CH201_SENSOR+1];
unsigned int amplitude[NB_CH201_SENSOR+1];
unsigned int distance_derivative;
bool buz_active[NB_BIP] = {0,0,0,0};
float pitch, roll;
unsigned long timeBuz;


/*
 *  Function prototype
 */
void measurementProcessing();


void setup() {
    Wire.begin(PIN_SDA, PIN_SCL, 100000UL);
    Serial.begin(9600);
    delay(400);

    if(TFMP.begin(1000) != true){   // Configuration du lidar
         while(1){}
    }
    
    if(initCH201() != true){        // Configuration des capteurs ultrasons CH201
        while(1){}
    }

    IMU.Init();

    delay(100);

    son.soundSystemReady();
}

void loop() {
    if(sendReceiveCH201(distance_measured,amplitude) == 2){ // Appel de la fonction permettant de demander au capteur CH201 une mesure et permettant la lecture de cette mesure
        measurementProcessing();                            // Si la lecture de la mesure a été effectué (valeur retournée = 2), appel de la fonction de traitement
    }
}


void measurementProcessing() {
    IMU.getAttitude(&pitch, &roll);    // Lecture des valeurs provenant de la centrale inertielle

    if(pitch > -70 && pitch < -10) {   // Contrôle de la bonne position de la canne pour qu'elle ne fasse pas de bips quand elle est tenue verticalement ou posée sur une table
        /*  Acquisition lidar   */
        distance_old_data[0] = distance_measured[0];    // Stockage de la valeur précédente mesurée par le lidar
        distance_measured[0] = TFMP.getDistance();      // Mesure de la valeur par le lidar

        distance_derivative = (distance_measured[0] - distance_old_data[0]) * 1/(100*CH201_MEASUREMENT_INTERVAL);    // Calcul de la vitesse de rapprochement d'un obstacle (dérivée du signal)
        if ((/*distance_derivative < -5 ||*/ distance_measured[0] < 160) && distance_measured[0] > 30 && buz_active[0] == 0) {  // Si un obstacle proche est détecté (via dérivée ou mesure directe)
            buz_active[0] = 1;
        } else if (distance_measured[0] > 400 && buz_active[0] == 0) {  // Si une longue distance est détectée via le lidar (escalier ou trou dans le sol)
            buz_active[3] = 1;
        } else if ((distance_derivative > 5 || distance_measured[0] > 170) && distance_measured[0] > 120) { // Si il n'y a plus d'obstacles détectés (via dérivée ou mesure directe)
            buz_active[0] = 0;
            buz_active[3] = 0;
        }

        /*  Acquisition capteur ultrason droit  */
        if (distance_measured[1] < 1900 && amplitude[1] > 10000 && buz_active[1] == 0) {    // Si un obstacle proche est détecté (via mesure directe et vérification de l'amplitude du signal)
            buz_active[1] = 1;
        } else if (distance_measured[1] > 1900 || amplitude[1] < 2000) {    // Si il n'y a plus d'obstacles détectés (via mesure directe ou mesure de l'amplitude du signal)
            buz_active[1] = 0;
        }

        /*  Acquisition capteur ultrason haut  */
        if(distance_measured[2] < 1500 && distance_measured[2] > 300 && amplitude[2] > 5000 && buz_active[2] == 0){     // Si un obstacle haut est détecté (via mesure directe et vérification de l'amplitude du signal)
            buz_active[2] = 1;
            timeBuz = millis();
        } else  if ((distance_measured[2] > 1500 || amplitude[2] < 2000) && (millis() - timeBuz > 400)){    // Si il n'y a plus d'obstacles haut détectés (via mesure directe ou mesure de l'amplitude du signal)
            buz_active[2] = 0;
        }

        son.makeSound(buz_active);        
    }
}