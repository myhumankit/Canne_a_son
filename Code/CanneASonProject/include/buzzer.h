/*
 *  Fonctions d'utilisation du buzzer
 */

#ifndef PCB_CANNEASON_BUZZER_H
#define PCB_CANNEASON_BUZZER_H

#include <Arduino.h>
#include "boardConfig.h"

#define VOLUME_BIP_SYSTEM VOLUME_BIP

class buzzer {
public:
    buzzer(int buzzerPin_, int freq_, int channel_, int resolution_);   /*  Constructor  */
    ~buzzer();                                                          /*  Destructor  */

    void bipSystemReady();      /*  Bip au démarrage de la canne  */
    void startBip(int freq_);   /*  Démarrage d'un bip à la fréquence passée en paramètre  */
    void stopBip();             /*  Arrêt du bip  */

private:
    int channel;
    int resolution;
};


#endif //PCB_CANNEASON_BUZZER_H
