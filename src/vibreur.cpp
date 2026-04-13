/*
 *  Fonctions d'utilisation du vibreur
 *
 *  @author BOUFALLOUS Amin
 *  @modified 2026-04-11
 */

#include "vibreur.h"
#include <Arduino.h>

// Constructor/Destructor
vibreur::vibreur(int vibreurPin, int freq, int channel_, int resolution_) 
    : channel(channel_), resolution(resolution_) {
    ledcSetup(channel, freq, resolution);
    ledcAttachPin(vibreurPin, channel);
}

vibreur::~vibreur() {}

/*  Bip au démarrage de la canne  */
void vibreur::bipSystemReady() {
    ledcWrite(channel, 1023);  // 10-bit max = 2^10 - 1
    delay(500);
    ledcWrite(channel, 0);
}

/*  Démarrage d'un bip à la fréquence passée en paramètre  */
void vibreur::startBip(int dutycycle) {
    ledcWrite(channel, dutycycle);
}

/*  Arrêt du bip  */
void vibreur::stopBip() {
    ledcWrite(channel, 0);
}
