#include "Controller.h"
#include "Config.h"
#include "OrientationMath.h"
#include <math.h>



void Controller::balanceCube(const float R[3][3], const float R_desired[3][3],
                    const float omega[3], float tau[3]) {
  float x[6];
  computeStateVector(R, R_desired, omega, x);

  // tau = -K * x
  for (int i = 0; i < 3; i++) {
    tau[i] = 0.0f;
    for (int j = 0; j < 6; j++) {
      tau[i] -= K[i][j] * x[j];
    }
  }
}

void Controller::computeStateVector(const float R[3][3], const float R_desired[3][3],
                        const float omega[3], float x[6]) {
  float Rt[3][3];  // R_desired^T
  float R_err[3][3];
  float skew[3][3];
  float angle_err[3];

  // transpose R_desired -> Rt
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      Rt[i][j] = R_desired[j][i];

  // R_err = Rt * R
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R_err[i][j] = 0;
      for (int k = 0; k < 3; k++) {
        R_err[i][j] += Rt[i][k] * R[k][j];
      }
    }
  }

  // skew = (R_err - R_err^T) / 2
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      skew[i][j] = 0.5f * (R_err[i][j] - R_err[j][i]);
    }
  }

  // vee mapping
 vee(skew, angle_err);

  // state vector x = [angle_err; omega]
  x[0] = angle_err[0];
  x[1] = angle_err[1];
  x[2] = angle_err[2];
  x[3] = omega[0];
  x[4] = omega[1];
  x[5] = omega[2];
}
void Controller::despool_tilt_controller(const float R_corner[3][3],
                                         float omega_in[3],
                                         float R_current[3][3],
                                         float R_desired[3][3])
{
    //----------------------------------------------------------------------
    // 1. Apply bias to omega if needed
    //----------------------------------------------------------------------
    float omega[3] = { omega_in[0] - 200,
                       omega_in[1] - 200,
                       -omega_in[2] -200};

    //----------------------------------------------------------------------
    // 2. Initialize static smoothing variables
    //----------------------------------------------------------------------
    static bool initialized = false;
    static float omega_smooth[3] = {0.0f, 0.0f, 0.0f};
    const float alpha = 0.005f;  // smoothing factor (tune between 0.02 - 0.2) 0.006

    if (!initialized) {
        for (int i = 0; i < 3; i++)
            omega_smooth[i] = omega[i];
        initialized = true;
    }

    //----------------------------------------------------------------------
    // 3. Exponential smoothing on omega
    //----------------------------------------------------------------------
    for (int i = 0; i < 3; i++)
        omega_smooth[i] = (1.0f - alpha) * omega_smooth[i] + alpha * omega[i];

    for (int i = 0; i < 3; i++)
        omega[i] = omega_smooth[i];

    //----------------------------------------------------------------------
    // 4. Tilt computation
    //----------------------------------------------------------------------

                                  float max_tilt = 20.0f * M_PI / 180.0f;  // 10 degrees in radians

    // Compute current tilt magnitude
    float omega_mag = sqrtf(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]);
    if (omega_mag < 1e-6f) omega_mag = 1e-6f;

    // Scale omega to represent rotation angles, clamped to ±10°
    float scale = fminf(max_tilt / omega_mag, 1.0f / 1000.0f); //1000 20 degree
    float omega_scaled[3] = { omega[0] * scale,
                              omega[1] * scale,
                              omega[2] * scale };

    float R_tilt[3][3];
    OrientationMath::eulerToR(omega_scaled, R_tilt);

    //----------------------------------------------------------------------
    // 5. Compute desired rotation
    //----------------------------------------------------------------------
    OrientationMath::matMul(R_tilt, R_corner, R_desired);
}


 
//*/
/*void Controller::despool_tilt_controller(const float R_corner[3][3],
                                          float omega_in[3],
                                          float R_current[3][3],
                                         float R_desired[3][3]) {
                                                static bool initialized = false;
    static float R_desired_prev[3][3];

    if (!initialized) {
        // Initialize persistent previous desired rotation
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                R_desired_prev[i][j] = R_corner[i][j];
        initialized = true;
    }

    // --- Parameters ---
  
    const float max_tilt = 30.0f * (M_PI / 180.0f); // 20 degrees in radians
    const float alpha = 0.5;    
    // --- Adjusted omega with your preferred offsets ---
    float omega[3] = {
        omega_in[0] + 200.0f,
        omega_in[1] - 300.0f,
        omega_in[2] + 100.0f
        // Alternative: {0, 0, 0} for testing
    };

    // --- Persistent prioritization memory ---
    static float omega_prioritized[3] = {0.0f, 0.0f, 0.0f};  

    // --- Tunable parameters ---
    const float priority_rate = 0.01f;   // how quickly dominant axis changes (0.0–1.0)
    const float bias_strength = 0.6f;    // how strongly dominant axis is favored

    // --- Compute magnitudes of angular velocity per axis ---
    float abs_omega[3] = {
        fabsf(omega[0]),
        fabsf(omega[1]),
        fabsf(omega[2])
    };

    // --- Identify dominant axis ---
    int dominant_axis = 0;
    if (abs_omega[1] > abs_omega[dominant_axis]) dominant_axis = 1;
    if (abs_omega[2] > abs_omega[dominant_axis]) dominant_axis = 2;

    // --- Smooth priority weighting over time ---
    for (int i = 0; i < 3; i++) {
        float target = (i == dominant_axis) ? 1.0f : 0.0f;
        omega_prioritized[i] = (1.0f - priority_rate) * omega_prioritized[i] + priority_rate * target;
    }

    // --- Blend omegas according to priority ---
    float omega_weighted[3];
    float sum_weights = omega_prioritized[0] + omega_prioritized[1] + omega_prioritized[2] + 1e-6f;

    for (int i = 0; i < 3; i++) {
        // Bias toward the dominant axis while keeping small contributions from others
        float bias = bias_strength * omega_prioritized[i] + (1.0f - bias_strength) * (1.0f / 3.0f);
        omega_weighted[i] = omega[i] * (bias / sum_weights);
    }

    // --- Compute tilt axis in world frame ---
    float tilt_axis_world[3] = {
        omega_weighted[0],
        omega_weighted[1],
        omega_weighted[2]
    };

    // --- Normalize ---
    float norm_axis = sqrtf(
        tilt_axis_world[0]*tilt_axis_world[0] +
        tilt_axis_world[1]*tilt_axis_world[1] +
        tilt_axis_world[2]*tilt_axis_world[2]);

    float R_target[3][3];
    if (norm_axis < 1e-6f) {
        // No tilt: use R_corner directly
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                R_target[i][j] = R_corner[i][j];
    } else {
        // Normalize and compute scaled tilt angle
        for (int i = 0; i < 3; i++) tilt_axis_world[i] /= norm_axis;
        float tilt_angle = max_tilt * fminf(1.0f, norm_axis / 100.0f);

        // Build rotation matrix for tilt
        float R_tilt[3][3];
        OrientationMath::axisAngleToRotationMatrix(tilt_axis_world, tilt_angle, R_tilt);

        // Apply tilt to corner orientation
        OrientationMath::matMul(R_tilt, R_corner, R_target);
    }

    // --- Convert to quaternions ---
    float q_target[4], q_prev[4];
    OrientationMath::rotmToQuat(R_target, q_target);
    OrientationMath::rotmToQuat(R_desired_prev, q_prev);

    // --- Smooth quaternion transition ---
    float q_desired[4];
    OrientationMath::slerp(q_prev, q_target, alpha, q_desired);

    // --- Convert back to rotation matrix ---
    OrientationMath::quatToRotm(q_desired, R_desired);

    // --- Update persistent state ---
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            ((float (*)[3])R_desired_prev)[i][j] = R_desired[i][j];
}//*/




void Controller::vee(const float S[3][3], float v[3]) {
    v[0] = S[2][1];
    v[1] = S[0][2];
    v[2] = S[1][0];
}