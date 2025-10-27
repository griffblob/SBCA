#include "MotorController.h"
#include "Config.h"
#include <Arduino.h>

void MotorController::init(int pin) {
    anESC.begin(pin,DSHOT300, ENABLE_BIDIRECTION, 14); 
}
void MotorController::setTorque(float torque) {
    tauset= handleTau(torque,rpm);
    tauset= smoothTau(tauset);
    int dshotVal = torqueToDShot(tauset);
   anESC.send_dshot_value(dshotVal);
}
uint32_t MotorController::getRPM() {
    extended_telem_type_t telem = TELEM_TYPE_ERPM;
    anESC.get_dshot_packet(&rpm, &telem);
    return rpm;
}
uint32_t MotorController::getOmega() {
    float sign_tua = (tauset > 0) ? 1.0 : (tauset < 0 ? -1.0 : 0.0);
    extended_telem_type_t telem = TELEM_TYPE_ERPM;
    anESC.get_dshot_packet(&rpm, &telem);
    return (rpm * 2.0 * M_PI / 60.0)*sign_tua;
}
void MotorController::Slow() {
    tauset= tauset*0.995;
    int dshotVal = torqueToDShot(tauset);
   anESC.send_dshot_value(dshotVal);
}


float MotorController::smoothTau(float tua) {
    
   

    // Check if magnitude decreased
    if (fabsf(tua) < fabsf(smooth_prev_tua)) {
        // Smooth only when decreasing
        smooth_prev_tua = smooth_prev_tua + TORQUE_SMOOTHING_ALPHA * (tua - smooth_prev_tua);
    } else {
        // Immediate update when increasing
        smooth_prev_tua = tua;
    }

    return smooth_prev_tua;
}


float MotorController::handleTau(float tua, float rpm) {


    int cur_sign = (tua > 0) - (tua < 0); // sign of current tua: +1, -1, or 0

    // Detect sign change (ignore zeros)
    if (!handle_waiting && cur_sign != 0 && handle_prev_sign != 0 && cur_sign != handle_prev_sign) {
        handle_waiting = true;            // sign flip detected -> start waiting
    }

    if (handle_waiting) {
        // If tua goes back to the previous sign, allow immediately
        if (cur_sign == handle_prev_sign && cur_sign != 0) {
            handle_waiting = false;
            handle_prev_tua = tua;
            handle_prev_sign = cur_sign;
            return tua;
        }

        // If rpm is low enough, allow new sign
        if (rpm < 300.0f) {
            handle_waiting = false;
            handle_prev_tua = tua;
            if (cur_sign != 0) handle_prev_sign = cur_sign;
            return tua;
        }

        // Otherwise still blocking
        return 0.0f;
    }

    // Normal pass-through
    if (cur_sign != 0) {
        handle_prev_tua = tua;
        handle_prev_sign = cur_sign;
    }
    return tua;
}

int MotorController::torqueToDShot(float Tua) {
    if (fabs(Tua) < 0.01) {
        return 0; // stop if near zero
    }

    if (Tua < 0) {
        // Negative direction → map [-Tua_max, 0) to [48, 1047]
        float ratio = fminf(fabs(Tua), 1.0f); 
        return DSHOT_MIN_REVERSE + (int)(ratio * (DSHOT_MAX_REVERSE - DSHOT_MIN_REVERSE));
    } else {
        // Positive direction → map (0, +Tua_max] to [1048, 2047]
        float ratio = fminf(Tua , 1.0f); 
        return DSHOT_MIN_FORWARD + (int)(ratio * (DSHOT_MAX_FORWARD - DSHOT_MIN_FORWARD));
    }
}