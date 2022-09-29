/*
 *  Fonctions d'utilisation du lidar TF Mini Plus
 */

#ifndef PCB_CANNEASON_TFMINIPLUS_H
#define PCB_CANNEASON_TFMINIPLUS_H

#include <TFMPI2C.h>
#include <iostream>
#include <iomanip>

using namespace std;

class TFMiniPlus {
public:
    TFMiniPlus();
    ~TFMiniPlus();

    bool begin(unsigned int dataFrameRate = 50);    /*  Initialisation du lidar  */
    void printData();           /*  Affichage dans la console des données mesurées par le lidar  */
    int16_t getDistance();      /*  Lecture de la distance mesurée par le lidar  */
    void printDistance();       /*  Affichage dans la console de la distance mesurées par le lidar  */

private:
    TFMPI2C tfmP;
    int16_t distance;
};

extern TFMiniPlus TFMP;

#endif //PCB_CANNEASON_TFMINIPLUS_H
