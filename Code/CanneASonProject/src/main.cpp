#include <Arduino.h>
#include <M5Atom.h>
#include <MPU6886_custom.h>

#include "ledMatrix.h"
#include "TFMiniPlus.h"
#include "buzzer.h"
#include "boardConfig.h"
#include "CH201.h"
#include "MPU6886_custom.h"


/*
 *  Constructors call
 */
buzzer bzz(BUZZER_PIN,2000,0,10);
MPU6886_custom IMU;


/*
 *  Global variables
 */
unsigned int distance_measured[NB_CH201_SENSOR+1];
unsigned int distance_old_data[NB_CH201_SENSOR+1];
unsigned int amplitude[NB_CH201_SENSOR+1];
unsigned int distance_derivative;
bool buz_active[NB_BIP] = {0,0,0,0};
bool buz_allow = 1;
double pitch, roll;
unsigned long timeBuz;


/*
 *  Function prototype
 */
void buttonInterrupt();
void func_bip();


void setup() {
    M5.begin(false,true,true);
    ledMatrix_waitscreen();     // Ecran orange, tant que la configuration n'est pas terminée
    delay(200);
    
    pinMode(BUTTON, INPUT_PULLUP);  // Interuption pour le bouton contenu dans le M5Atom
    attachInterrupt(digitalPinToInterrupt(BUTTON), buttonInterrupt, FALLING);

    if(TFMP.begin(1000) != true){   // Configuration du lidar
        ledMatrix_redscreen();                  // Ecran rouge en cas d'erreur de configuration du lidar
        while(1);
    }

    IMU.Init();

    if(initCH201() != true){                    // Configuration des capteurs ultrasons CH201
        ledMatrix_redscreen();                  // Ecran rouge en cas d'erreur de configuration des capteurs
        while(1);
    }

    ledMatrix_greenscreen();    // Ecran vert qui indique que le système est bien initialisé
    bzz.bipSystemReady();
}

void loop() {
    if(sendReceiveCH201(distance_measured,amplitude) == 2){ // Appel de la fonction permettant de demander au capteur CH201 une mesure et permettant la lecture de cette mesure
        func_bip();                                         // Si la lecture de la mesure a été effectué (valeur retournée = 2), appel de la fonction de traitement
    }
}


void func_bip(){
    IMU.getAttitude(&pitch, &roll); // Lecture des valeurs provenant de la centrale inertielle

    if(roll > -10 && roll < 18) {   // Contrôle de la bonne position de la canne pour qu'elle ne fasse pas de bips quand elle est tenue verticalement ou posée sur une table
        /*  Acquisition lidar   */
        distance_old_data[0] = distance_measured[0];    // Stockage de la valeur précédente mesurée par le lidar
        distance_measured[0] = TFMP.getDistance();      // Mesure de la valeur par le lidar

        distance_derivative = (distance_measured[0] - distance_old_data[0]) * 1/(100*CH201_MEASUREMENT_INTERVAL);    // Calcul de la vitesse de rapprochement d'un obstacle (dérivée du signal)
        if ((/*distance_derivative < -5 ||*/ distance_measured[0] < 160) && distance_measured[0] > 30 && buz_active[0] == 0) {  // Si un obstacle proche est détecté (via dérivée ou mesure directe)
            buz_active[0] = 1;
        } else if (distance_measured[0] > 400 && buz_active[0] == 0) {  // Si une longue distance est détectée via le lidar (escalier ou trou dans le sol)
            buz_active[3] = 1;
        } else if ((distance_derivative > 5 || distance_measured[0] > 170) && distance_measured[0] > 120) { // Si il n'y a plus d'obstacles détectés (via dérivée ou mesure directe)
#ifdef DEBUG_SCREEN
            if(buz_allow)
                M5.dis.fillpix(GREEN_C);
#endif
            buz_active[0] = 0;
            buz_active[3] = 0;
        }


        /*  Acquisition capteur ultrason droit  */
        if (distance_measured[1] < 1900 && amplitude[1] > 10000 && buz_active[1] == 0) {    // Si un obstacle proche est détecté (via mesure directe et vérification de l'amplitude du signal)
            buz_active[1] = 1;
        } else if (distance_measured[1] > 1900 || amplitude[1] < 2000) {    // Si il n'y a plus d'obstacles détectés (via mesure directe ou mesure de l'amplitude du signal)
#ifdef DEBUG_SCREEN
            if(buz_allow)
                M5.dis.fillpix(GREEN_C);
#endif
            buz_active[1] = 0;
        }


        /*  Acquisition capteur ultrason haut  */
        if(distance_measured[2] < 1500 && distance_measured[2] > 300 && amplitude[2] > 5000 && buz_active[2] == 0){     // Si un obstacle haut est détecté (via mesure directe et vérification de l'amplitude du signal)
            buz_active[2] = 1;
            timeBuz = millis();
        } else  if ((distance_measured[2] > 1500 || amplitude[2] < 2000) && (millis() - timeBuz > 400)){    // Si il n'y a plus d'obstacles haut détectés (via mesure directe ou mesure de l'amplitude du signal)
#ifdef DEBUG_SCREEN
            if(buz_allow)
                M5.dis.fillpix(GREEN_C);
#endif
            buz_active[2] = 0;
        }


        /*  Retour à l'utilisateur via buzzer  */
        if (buz_active[1] && buz_allow) {           // Si un obstacle est détecté par le capteur ultrason droit
            bzz.startBip(BIP_OBSTACLE);             // Démarrage d'un bip à la fréquence configurée dans boardConfig.h
#ifdef DEBUG_SCREEN
            M5.dis.fillpix(RED_C);      //Pour debug : Rouge = obstacle détécté par le capteur ultrason droit
#endif
        } else if (buz_active[3] && buz_allow) {    // Si un esclier ou un trou est détecté par le lidar
            bzz.startBip(BIP_ESCALIER);             // Démarrage d'un bip à la fréquence configurée dans boardConfig.h
#ifdef DEBUG_SCREEN
            M5.dis.fillpix(PURPLE_C);   //Pour debug : Violet = escalier ou trou détecté par le lidar
#endif
        } else if (buz_active[0] && buz_allow) {    // Si un obstacle est détecté par le lidar
            bzz.startBip(BIP_OBSTACLE);             // Démarrage d'un bip à la fréquence configurée dans boardConfig.h
#ifdef DEBUG_SCREEN
            M5.dis.fillpix(BLUE_C);     //Pour debug : Bleu = obstacle détécté par le lidar
#endif
        } else if(buz_active[2] && buz_allow) {     // Si un obstacle en hauteur est détecté par le capteur ultrason haut
            bzz.startBip(BIP_OBSTACLE_HAUTEUR);     // Démarrage d'un bip à la fréquence configurée dans boardConfig.h
#ifdef DEBUG_SCREEN
            M5.dis.fillpix(WHITE_C);     //Pour debug : Blanc = obstacle en hauteur détécté par le  capteur ultrason droit
#endif
        }
    }
}


/*
 *  Routine d'interruption du bouton
 */
void buttonInterrupt() {
    buz_allow = !buz_allow;
    if(buz_allow)
        ledMatrix_greenscreen();
    else {
        ledMatrix_waitscreen();
        bzz.stopBip();
    }
}

