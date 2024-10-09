/*
 * Modification de la bibliothèque du MPU6050 pour la faire fonctionner avec la canne
 *
 */

#ifndef _MPU6050_custom_H_
#define _MPU6050_custom_H_

#include <Wire.h>
#include <Arduino.h>

class MPU6050_custom {
    public:
      MPU6050_custom();
      ~MPU6050_custom(); 

      int Init(void);
      void getAttitude(float *pitch, float *roll);
};
#endif