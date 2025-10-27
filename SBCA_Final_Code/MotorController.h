#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <DShotRMT.h>

class MotorController {
public:
    void init(int pin);
    void setTorque(float torque);
    uint32_t getRPM();
    uint32_t getOmega();
    void Slow();
    DShotRMT anESC;


private:
    
    uint32_t rpm;
    float  tauset;
    float smoothTau(float tua) ;
    float handleTau(float tua, float rpm);
    int torqueToDShot(float Tua);
    float smooth_prev_tua = 0.0f;
    float handle_prev_tua = 0.0f;
    bool handle_waiting = false;
    int handle_prev_sign = 0;
};

#endif