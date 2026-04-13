/*
 *  Fonctions d'utilisation du buzzer
 */

#ifndef PCB_CANNEASON_SOUND_H
#define PCB_CANNEASON_SOUND_H

#include "buzzer.h"

class sound {
public:
    sound();   /*  Constructor  */
    ~sound();                                                          /*  Destructor  */

    void soundSystemReady();      /*  Bip au démarrage de la canne  */
    void makeSound(bool *sensor_detection);

private:
    int soundMode();
    void doSound(int freqBuzzer, int freqVibreur);
};

#endif //PCB_CANNEASON_BUZZER_H
