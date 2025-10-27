#ifndef CONTROLLER_H
#define CONTROLLER_H

class Controller {
public:
    void balanceCube(const float R[3][3], const float R_desired[3][3],
                    const float omega[3], float tau[3]);
    void despool_tilt_controller(const float R_corner[3][3],float omega_in[3],float R_current[3][3],float R_desired[3][3]);

private:
    void computeStateVector(const float R[3][3], const float R_desired[3][3],
                           const float omega[3], float x[6]);
    static void vee(const float S[3][3], float v[3]);
};

#endif