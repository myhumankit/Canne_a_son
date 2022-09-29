/*
 *  Fonctions de modification de la couleur de l'écran
 */

#ifndef PCB_CANNEASON_LEDMATRIX_H
#define PCB_CANNEASON_LEDMATRIX_H

#include <M5Atom.h>

/*
 *  Couleur de la matrice LED
 */

#define GREEN_C     0x008000
#define RED_C       0xff0000
#define ORANGE_C    0xff7f00
#define PURPLE_C    0xee82ee
#define BLUE_C      0x0000ff
#define WHITE_C     0xffffff

void ledMatrix_waitscreen();

void ledMatrix_greenscreen();

void ledMatrix_redscreen();

#endif //PCB_CANNEASON_LEDMATRIX_H
