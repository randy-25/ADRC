#include "ADRC.h"

ADRC::ADRC(float t_sampling, float b0, float t_settling, float setpoint, int k_eso){
    this->t_sampling = t_sampling;
    this->b0 = b0;
    this->t_settling = t_settling;
    this->setpoint = setpoint;
    this->k_eso = k_eso;

    this->s_cl = -6.0 / t_settling;
    this->z_eso = exp(this->k_eso*s_cl * t_sampling);
    this->l1 = 1 - pow(z_eso, 3);
    this->l2 = 3.0f / (2 * t_sampling) * pow((1 - z_eso), 2) * (1 + z_eso);
    this->l3 = 1.0f / pow(t_sampling, 2) * pow((1 - z_eso), 3);
    this->Kp = s_cl * s_cl;
    this->Kd = -2 * s_cl;

    this->Ad << 1, t_sampling, pow(t_sampling, 2)/2.0f,
                0, 1, t_sampling,
                0, 0, 1;
    this->Bd << b0 * pow(t_sampling, 2) / 2.0f, b0 * t_sampling, 0;
    this->Cd << 1, 0, 0;
    this->Lc << l1, l2, l3;
    this->x_hat << 0, 0, 0; // state initialization
}

void ADRC::updateEso(float y, float u){
    this->x_hat = (this->Ad - ((this->Lc*this->Cd)*this->Ad))*this->x_hat + (this->Bd - ((this->Lc*this->Cd)*this->Bd))*u + this->Lc*y;
}

float ADRC::computeControlSignal(float setpoint, float b0){
    this->u = ((this->Kp* (setpoint - this->x_hat(0))) - this->Kd * this->x_hat(1) - this->x_hat(2)) / b0;
    return this->u;
}

void ADRC::resetADRC(){
    this->x_hat << 0, 0, 0;
    this->u = 0;
}