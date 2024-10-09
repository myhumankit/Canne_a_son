/*
 *  Fonctions d'utilisation du buzzer
 */

#include "sound.h"
#include "vibreur.h"

buzzer bzz(BUZZER_PIN, 2000, 0, 10);
vibreur vzz(VIBREUR_PIN, 20000, 1, 10);


// Constructor/Destructor
sound::sound() {
    pinMode(switchBuzVibreur1, INPUT_PULLUP);
    pinMode(switchBuzVibreur2, INPUT_PULLUP);
}
sound::~sound() {}


/*  Bip au démarrage de la canne  */
void sound::soundSystemReady() {
    bzz.bipSystemReady();
    vzz.bipSystemReady();
}


void sound::makeSound(bool *sensor_detection){
    /*  Retour à l'utilisateur via buzzer  */
    if (sensor_detection[1]) {              // Si un obstacle est détecté par le capteur ultrason droit
        doSound(BIP_OBSTACLE,1024);          // Démarrage d'un son aux fréquences configurée dans boardConfig.h
        printf("\nBIP OBSTACLE US\n");
    } else if (sensor_detection[3]) {       // Si un esclier ou un trou est détecté par le lidar
        doSound(BIP_ESCALIER,900);          // Démarrage d'un bip aux fréquences configurée dans boardConfig.h
        printf("\nBIP ESCALIER\n");
    } else if (sensor_detection[0]) {       // Si un obstacle est détecté par le lidar
        doSound(BIP_OBSTACLE,1024);          // Démarrage d'un bip aux fréquences configurée dans boardConfig.h
        printf("\nBIP OBSTACLE LIDAR\n");
    } else if(sensor_detection[2]) {        // Si un obstacle en hauteur est détecté par le capteur ultrason haut
        doSound(BIP_OBSTACLE_HAUTEUR,800);  // Démarrage d'un bip aux fréquences configurée dans boardConfig.h
        printf("\nBIP OBSTACLE HAUTEUR\n");
    }
}


// 0 = buzzer et vibreur, 1 = buzzer, 2 = vibreur
int sound::soundMode(){
    int valSwitch1, valSwitch2, returnvalue;
    valSwitch1 = digitalRead(switchBuzVibreur1);
    valSwitch2 = digitalRead(switchBuzVibreur2);

    if(valSwitch1 && valSwitch2) 
        returnvalue = 0;
    else if (valSwitch1 && !valSwitch2) 
        returnvalue = 1;
    else if(!valSwitch1 && valSwitch2)
        returnvalue = 2;
    
    return returnvalue;
}


void sound::doSound(int freqBuzzer, int freqVibreur){
    switch(soundMode()){
        case 0 : bzz.startBip(freqBuzzer);
                 vzz.startBip(freqVibreur);
                 break;
        case 1 : bzz.startBip(freqBuzzer);
                 break;
        case 2 : vzz.startBip(freqVibreur);
                 break;
    }    
}
