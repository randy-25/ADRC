// Include Eigen library
#include "eigen.h"
#include <Eigen/Dense>

using namespace Eigen;

// Define a structure to hold ADRC parameters and states
typedef struct {
    // Input
    float t_sampling;
    float b0;
    float t_settling;
    int k_eso;

    // Protection
    float u_min; // Minimum control signal
    float u_max; // Maximum control signal

    // Computed
    float s_cl;
    float z_eso;
    float l1, l2, l3;
    float Kp, Kd;

    MatrixXf Ad;
    MatrixXf Bd;
    MatrixXf Cd;
    MatrixXf Lc;
    MatrixXf x_hat;

    // Output
    float u;
    float u_prev; // Previous control signal (for filtering)
} ADRC;

// Initialize the ADRC parameters and matrices
void initADRC(ADRC *adrc, float t_sampling, float b0, float t_settling, int k_eso, float u_min, float u_max) {
    adrc->t_sampling = t_sampling;
    adrc->b0 = b0;
    adrc->t_settling = t_settling;
    adrc->k_eso = k_eso;

    adrc->u_min = u_min;
    adrc->u_max = u_max;

    adrc->s_cl = -6.0 / t_settling;
    adrc->z_eso = exp(k_eso * adrc->s_cl * t_sampling);
    adrc->l1 = 1 - pow(adrc->z_eso, 3);
    adrc->l2 = 3.0f / (2 * t_sampling) * pow((1 - adrc->z_eso), 2) * (1 + adrc->z_eso);
    adrc->l3 = 1.0f / pow(t_sampling, 2) * pow((1 - adrc->z_eso), 3);
    adrc->Kp = adrc->s_cl * adrc->s_cl;
    adrc->Kd = -2 * adrc->s_cl;

    adrc->Ad = MatrixXf(3, 3);
    adrc->Ad << 1, t_sampling, pow(t_sampling, 2) / 2.0f,
                0, 1, t_sampling,
                0, 0, 1;

    adrc->Bd = MatrixXf(3, 1);
    adrc->Bd << b0 * pow(t_sampling, 2) / 2.0f, b0 * t_sampling, 0;

    adrc->Cd = MatrixXf(1, 3);
    adrc->Cd << 1, 0, 0;

    adrc->Lc = MatrixXf(3, 1);
    adrc->Lc << adrc->l1, adrc->l2, adrc->l3;

    adrc->x_hat = MatrixXf::Zero(3, 1); // Initialize state to zero
    adrc->u = 0;
    adrc->u_prev = 0; // Initialize previous control signal
}

// Reset ADRC states
void resetADRC(ADRC *adrc) {
    adrc->x_hat = MatrixXf::Zero(3, 1);
    adrc->u = 0;
    adrc->u_prev = 0;
}

// Update the Extended State Observer (ESO)
void updateEso(ADRC *adrc, float y, float u) {
    adrc->x_hat = (adrc->Ad - (adrc->Lc * adrc->Cd) * adrc->Ad) * adrc->x_hat
                + (adrc->Bd - (adrc->Lc * adrc->Cd) * adrc->Bd) * u
                + adrc->Lc * y;
}

// Compute the control signal
float computeControlSignal(ADRC *adrc, float setpoint) {
    // Compute raw control signal
    adrc->u = (adrc->Kp * (setpoint - adrc->x_hat(0, 0)) - adrc->Kd * adrc->x_hat(1, 0) - adrc->x_hat(2, 0)) / adrc->b0;

    // Clamp the control signal
    adrc->u = fmax(adrc->u_min, fmin(adrc->u, adrc->u_max));

    // Update previous control signal
    adrc->u_prev = adrc->u;

    return adrc->u;
}

MatrixXf getXHAT(ADRC *adrc){
  return adrc->x_hat;
}
