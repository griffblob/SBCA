////////////////////////////////////////////////////////////
// Arduino firmware for use with FreeSixCube processing example
////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <DShotRMT.h>
#include "Config.h"
#include "MotorController.h"
#include "OrientationMath.h"
#include "Controller.h"




#define DEBUG
#ifdef DEBUG
#include "DebugUtils.h"
#endif

#include "CommunicationUtils.h"
#include "FreeSixIMU.h"
#include <Wire.h>

#include <math.h>
#include <string.h>

float q[4];  // hold quaternion values
float omega[3];
float R[3][3];  // hold rotation matrix

FreeSixIMU my3IMU = FreeSixIMU();
MotorController motorController0;
MotorController motorController1;
MotorController motorController2;
Controller lqrController;

uint32_t t0;
uint32_t tp;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22, 400000);

  delay(500);
  my3IMU.init();

  delay(5);
  motorController0.init(17);
  motorController2.init(16);
  motorController1.init(5);

  t0 = millis();  //arm esc
  while (millis() - t0 < 3000) {
    motorController0.setTorque(0);
   
    motorController1.setTorque(0);
    motorController2.setTorque(0);

    delay(2);
  }
  my3IMU.setInitialPosition();
 t0 = millis(); 
  tp = millis(); 
}


uint32_t state = 0;
bool ovsat = false;
uint32_t rpm =0;
void loop() {
  float tau=0;
  if (millis()-t0<5000){
    tau= (millis()-t0)/5000.0;
  }
  else if (millis()-t0<10000){
    tau= (10000-(millis()-t0))/5000.0;
  }
  else if (millis()-t0<12000){
    tau= 0.05;
  }
    else if (millis()-t0<13000){
    tau= 0.06;
  }
    else if (millis()-t0<14000){
    tau= 0.07;
  }
    else if (millis()-t0<15000){
    tau= 0.08;
  }
    else if (millis()-t0<16000){
    tau= 0.09;
  }
    else if (millis()-t0<17000){
    tau= 0.10;
  }
    else if (millis()-t0<18000){
    tau= 0.11;
  }
    else if (millis()-t0<19000){
    tau= 0.12;
  }
      else if (millis()-t0<21000){
    tau= 0;
  }
        else if (millis()-t0<23000){
    tau= 1;
  }
          else if (millis()-t0<25000){
    tau= 0;
  }
   

  if (millis()-tp>10){
    tp=tp+10;
    Serial.print(millis()-t0);
    Serial.print(",");
    Serial.print(tau);
    Serial.print(",");
    Serial.println(rpm);
  }



  motorController0.setTorque(tau);
  rpm= motorController0.getRPM();

  delayMicroseconds(100);  // small delay so serial doesn’t spam too fast
}
