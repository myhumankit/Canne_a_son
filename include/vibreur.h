/*
 *  Fonctions d'utilisation du vibreur
 */

#ifndef PCB_CANNEASON_VIBREUR_H
#define PCB_CANNEASON_VIBREUR_H

#include <Arduino.h>
#include "boardConfig.h"

#define VOLUME_BIP_SYSTEM VOLUME_BIP

class vibreur {
public:
    vibreur(int vibreurPin, int freq, int channel_, int resolution_);   /*  Constructor  */
    ~vibreur();                                                          /*  Destructor  */

    void bipSystemReady();      /*  Bip au démarrage de la canne  */
    void startBip(int dutycycle);   /*  Démarrage d'un bip à la fréquence passée en paramètre  */
    void stopBip();             /*  Arrêt du bip  */

private:
    int channel;
    int resolution;
};

#endif //PCB_CANNEASON_BUZZER_H
