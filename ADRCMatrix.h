#ifndef ADRC_MATRIX_H
#define ADRC_MATRIX_H

#include "Eigen/Dense" // For matrix operations in simulation

using namespace Eigen;

class ADRCMatrix {
private:
    Matrix3f A, B, L, C;
    Vector3f z;
    float b0, Ts, Kp, Kd;
    float prevError;

public:
    ADRCMatrix(float b0, float Ts, float control_bw, float observer_bw);
    void updateESO(float y, float u);
    float computeControl(float setpoint, float y);
};

#endif
