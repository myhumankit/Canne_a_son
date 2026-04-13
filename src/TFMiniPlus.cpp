/*
 *  Fonctions d'utilisation du lidar TF Mini Plus
 */

#include "TFMiniPlus.h"

// Constructor/Destructor
TFMiniPlus::TFMiniPlus() {}
TFMiniPlus::~TFMiniPlus() {}


/*  Initialisation du lidar  */
bool TFMiniPlus::begin(unsigned int dataFrameRate) {
    printf("\nTFMPlus Init\n");

    // - - System reset - - - - - - - - - - -
    cout << "System reset...";
    if(tfmP.sendCommand( SOFT_RESET, 0))
    {
        cout << "OK" << endl;
    }
    else{
        cout << "NOT OK" << endl;
        cout << "ERROR | No lidar connected at I2C adress 0x10" << endl;
        return false;
    }

    // - - Set the data frame-rate - - - - - - - - -
    cout << "Data-Frame rate...";
    if(tfmP.sendCommand( SET_FRAME_RATE, dataFrameRate))
    {
        cout << "OK, " << setw(2) << dataFrameRate << "Hz" << endl;
    }
    else tfmP.printReply();
    cout << "TFMPlus Init OK" << endl;
    delay(50);
    return true;
}


/*  Affichage dans la console des données mesurées par le lidar  */
void TFMiniPlus::printData() {
    int16_t tfDist = 0;
    int16_t tfFlux = 0;
    int16_t tfTemp = 0;
    if(tfmP.getData( tfDist, tfFlux, tfTemp) == true){
        cout << "Dist : " << setw(4) << tfDist << " cm  Flux : " << setw(5) << tfFlux << "  Temp : " << setw(2) << tfTemp << " °C" << endl;
    }
}


/*  Lecture de la distance mesurée par le lidar  */
int16_t TFMiniPlus::getDistance() {
    int16_t tfDist = 0;
    int16_t tfFlux = 0;
    int16_t tfTemp = 0;
    if(tfmP.getData(tfDist, tfFlux, tfTemp) != true)
        return -1;
    return tfDist;
};


/*  Affichage dans la console de la distance mesurées par le lidar  */
void TFMiniPlus::printDistance() {
    cout << "Dist : " << setw(4) << distance << " cm" << endl;
};

TFMiniPlus TFMP;