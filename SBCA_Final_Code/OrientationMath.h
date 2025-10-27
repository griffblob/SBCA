#ifndef ORIENTATIONMATH_H
#define ORIENTATIONMATH_H

class OrientationMath {
public:
    static void quatToRotMatrix(float q[4], float R[3][3]);
    static void normalize(float v[3]);
    static void matVecMul(const float A[3][3], const float x[3], float y[3]);
    static void matMul(const float A[3][3], const float B[3][3], float C[3][3]);
    static void axisAngleToRotationMatrix(const float axis_in[3], float angle, float R[3][3]);
    static void matTranspose(const float A[3][3], float AT[3][3]);
    static void rotmToQuat(const float R[3][3], float q[4]);
    static void quatToRotm(const float q[4], float R[3][3]);
    static void slerp(const float q1[4], const float q2[4], float t, float q_out[4]);
    static void eulerToR(const float euler[3], float R[3][3]);
   
};

#endif