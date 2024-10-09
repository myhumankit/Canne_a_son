/*
 * Modification de la bibliothèque du MPU6050 pour la faire fonctionner avec la canne
 */

#include <math.h>
#include <Arduino.h>
#include <Wire.h>
#include "MPU6050_custom.h"
#include "boardConfig.h"
#include "MPU6050_light.h"

MPU6050 mpu(Wire);

MPU6050_custom::MPU6050_custom() {}
MPU6050_custom::~MPU6050_custom() {}

int MPU6050_custom::Init(void){
  int status = mpu.begin();
  printf("MPU6050 status: %d\n",status);
  while(status!=0){ }

  printf("IMU OK\n");
  return 0;
}

void MPU6050_custom::getAttitude(float *pitch_, float *roll_)
{
  mpu.update();
 
  *roll_ = mpu.getAngleX();
  *pitch_ = mpu.getAngleY();
}
