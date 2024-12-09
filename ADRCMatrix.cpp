#include "ADRCMatrix.h"

ADRCMatrix::ADRCMatrix(float b0, float Ts, float control_bw, float observer_bw)
    : b0(b0), Ts(Ts), prevError(0.0)
{
    A << 0, 1, 0,
        0, 0, 1,
        0, 0, 0;

    B << 0,
        b0,
        0;

    C << 1, 0, 0;

    L << 3 * observer_bw, 0, 0,
        0, 3 * observer_bw * observer_bw, 0,
        0, 0, observer_bw * observer_bw * observer_bw;

    Kp = control_bw * control_bw / b0;
    Kd = 2 * control_bw / b0;

    z << 0, 0, 0;
}

void ADRCMatrix::updateESO(float y, float u)
{
    Vector3f error = C.transpose() * (y - C * z);
    z = z + Ts * (A * z + B * u + L * error);
}

float ADRCMatrix::computeControl(float setpoint, float y)
{
    float error = setpoint - y;
    float dError = (error - prevError) / Ts;
    float u0 = Kp * error + Kd * dError;
    prevError = error;
    return u0 - z(2) / b0; // u = u0 - z3/b0
}
