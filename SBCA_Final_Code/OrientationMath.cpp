#include "OrientationMath.h"
#include <math.h>

void OrientationMath::quatToRotMatrix(float q[4], float R[3][3]) {
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];

    R[0][0] = 1 - 2*(qy*qy + qz*qz);
    R[0][1] = 2*(qx*qy - qz*qw);
    R[0][2] = 2*(qx*qz + qy*qw);

    R[1][0] = 2*(qx*qy + qz*qw);
    R[1][1] = 1 - 2*(qx*qx + qz*qz);
    R[1][2] = 2*(qy*qz - qx*qw);

    R[2][0] = 2*(qx*qz - qy*qw);
    R[2][1] = 2*(qy*qz + qx*qw);
    R[2][2] = 1 - 2*(qx*qx + qy*qy);
}
void OrientationMath::matTranspose(const float A[3][3], float AT[3][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      AT[i][j] = A[j][i];
    }
  }
}
// Converts Euler angles [roll, pitch, yaw] to a rotation matrix (XYZ convention)
void OrientationMath::eulerToR(const float euler[3], float R[3][3])
{
    float roll  = euler[0];  // rotation about X
    float pitch = euler[1];  // rotation about Y
    float yaw   = euler[2];  // rotation about Z

    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    // XYZ rotation sequence (roll -> pitch -> yaw)
    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;

    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;

    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}


void OrientationMath::normalize(float v[3]) {
    float norm = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (norm < 1e-9) return;
    v[0] /= norm; v[1] /= norm; v[2] /= norm;
}

void OrientationMath::matVecMul(const float A[3][3], const float x[3], float y[3]) {
    for (int i = 0; i < 3; i++) {
        y[i] = A[i][0]*x[0] + A[i][1]*x[1] + A[i][2]*x[2];
    }
}

void OrientationMath::matMul(const float A[3][3], const float B[3][3], float C[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C[i][j] = 0.0;
            for (int k = 0; k < 3; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void OrientationMath::axisAngleToRotationMatrix(const float axis_in[3], float angle, float R[3][3]) {
    float axis[3] = {axis_in[0], axis_in[1], axis_in[2]};
    normalize(axis);

    float x = axis[0], y = axis[1], z = axis[2];
    float c = cos(angle), s = sin(angle), C = 1.0 - c;

    R[0][0] = c + x*x*C;     R[0][1] = x*y*C - z*s; R[0][2] = x*z*C + y*s;
    R[1][0] = y*x*C + z*s;   R[1][1] = c + y*y*C;   R[1][2] = y*z*C - x*s;
    R[2][0] = z*x*C - y*s;   R[2][1] = z*y*C + x*s; R[2][2] = c + z*z*C;
}



void OrientationMath::rotmToQuat(const float R[3][3], float q[4]) {
    float tr = R[0][0] + R[1][1] + R[2][2];
    if (tr > 0) {
        float S = sqrtf(tr + 1.0f) * 2.0f;
        q[0] = 0.25f * S;
        q[1] = (R[2][1] - R[1][2]) / S;
        q[2] = (R[0][2] - R[2][0]) / S;
        q[3] = (R[1][0] - R[0][1]) / S;
    } else {
        int i = 0;
        if (R[1][1] > R[0][0]) i = 1;
        if (R[2][2] > R[i][i]) i = 2;
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;

        float S = sqrtf((R[i][i] - R[j][j] - R[k][k]) + 1.0f) * 2.0f;
        q[0] = (R[k][j] - R[j][k]) / S;
        q[1 + i] = 0.25f * S;
        q[1 + j] = (R[j][i] + R[i][j]) / S;
        q[1 + k] = (R[k][i] + R[i][k]) / S;
    }
}

void OrientationMath::quatToRotm(const float q[4], float R[3][3]) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    R[0][0] = 1 - 2 * (y*y + z*z);
    R[0][1] = 2 * (x*y - z*w);
    R[0][2] = 2 * (x*z + y*w);
    R[1][0] = 2 * (x*y + z*w);
    R[1][1] = 1 - 2 * (x*x + z*z);
    R[1][2] = 2 * (y*z - x*w);
    R[2][0] = 2 * (x*z - y*w);
    R[2][1] = 2 * (y*z + x*w);
    R[2][2] = 1 - 2 * (x*x + y*y);
}

void OrientationMath::slerp(const float q1[4], const float q2[4], float t, float q_out[4]) {
    float cosTheta = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3];
    float q2c[4];
    for (int i = 0; i < 4; i++) q2c[i] = q2[i];

    if (cosTheta < 0.0f) {
        for (int i = 0; i < 4; i++) q2c[i] = -q2c[i];
        cosTheta = -cosTheta;
    }

    if (cosTheta > 0.9995f) {
        for (int i = 0; i < 4; i++)
            q_out[i] = q1[i] + t * (q2c[i] - q1[i]);
    } else {
        float theta = acosf(cosTheta);
        float sinTheta = sinf(theta);
        float w1 = sinf((1 - t) * theta) / sinTheta;
        float w2 = sinf(t * theta) / sinTheta;
        for (int i = 0; i < 4; i++)
            q_out[i] = w1 * q1[i] + w2 * q2c[i];
    }

    // Normalize
    float norm = sqrtf(q_out[0]*q_out[0] + q_out[1]*q_out[1] +
                       q_out[2]*q_out[2] + q_out[3]*q_out[3]);
    for (int i = 0; i < 4; i++) q_out[i] /= norm;
}
