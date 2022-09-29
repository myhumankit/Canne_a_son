/*
 *  Fonctions d'utilisation du buzzer
 */

#include "buzzer.h"

// Constructor/Destructor
buzzer::buzzer(int buzzerPin, int freq, int channel_, int resolution_) : channel(channel_),
                                                                         resolution(resolution_) {
    ledcSetup(channel, freq, resolution);
    ledcAttachPin(buzzerPin, channel);
}
buzzer::~buzzer() {}


/*  Bip au démarrage de la canne  */
void buzzer::bipSystemReady() {
    ledcWrite(channel, VOLUME_BIP_SYSTEM);
    delay(200);
    ledcWrite(channel, 0);
    delay(125);
    ledcWrite(channel, VOLUME_BIP_SYSTEM);
    delay(200);
    ledcWrite(channel, 0);
}


/*  Démarrage d'un bip à la fréquence passée en paramètre  */
void buzzer::startBip(int freq) {
    ledcChangeFrequency(channel, freq, resolution);
    ledcWrite(channel, VOLUME_BIP_SYSTEM);
}


/*  Arrêt du bip  */
void buzzer::stopBip() {
    ledcWrite(channel, 0);
}


