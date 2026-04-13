/*
 * Modification de la bibliothèque du MPU6050 pour la faire fonctionner avec la canne
 *
 * @author BOUFALLOUS Amin
 * @modified 2026-04-11
 */

#include <math.h>
#include <Arduino.h>
#include <Wire.h>
#include "MPU6050_custom.h"
#include "boardConfig.h"
#include "MPU6050_light.h"

MPU6050 mpu(Wire);

// Global instance used by SensorManager
MPU6050_custom IMU;

MPU6050_custom::MPU6050_custom() {}
MPU6050_custom::~MPU6050_custom() {}

int MPU6050_custom::Init(void){
  int status = mpu.begin();
  printf("MPU6050 status: %d\n", status);
  if(status != 0) {
      printf("MPU6050 FAILED (err=%d)\n", status);
      return status;
  }
  // Calibrate offsets — cane must be still and flat during boot (~500ms)
  mpu.calcOffsets();
  printf("IMU OK (offsets calibrated)\n");
  return 0;
}

void MPU6050_custom::getAttitude(float *pitch_, float *roll_)
{
  mpu.update();

  *roll_ = mpu.getAngleX();
  *pitch_ = mpu.getAngleY();
}

float MPU6050_custom::getAccelMagnitude()
{
  // Called after getAttitude() — mpu.update() already fetched fresh data
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();
  return sqrt(ax*ax + ay*ay + az*az);
}
