/*
 *  Fonctions de modification de la couleur de l'écran
 */

#include "ledMatrix.h"


void ledMatrix_waitscreen(){
    M5.dis.fillpix(ORANGE_C);
}


void ledMatrix_greenscreen(){
    M5.dis.fillpix(GREEN_C);
}


void ledMatrix_redscreen(){
    M5.dis.fillpix(RED_C);
}