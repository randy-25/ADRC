#include "Eigen/Dense"

class ADRC{
    private:
        // Input
        float t_sampling;
        float b0;
        float t_settling; // settling time 2%
        float setpoint;
        int k_eso; // 3 - 10


        //Computed
        float s_cl;
        float z_eso;
        float l1, l2, l3;
        float Kp, Kd;
        float x1, x2, x3;

        Eigen::Matrix3f Ad;
        Eigen::Matrix<float, 3, 1> Bd;
        Eigen::Matrix<float, 1, 3> Cd;
        Eigen::Matrix<float, 3, 1> Lc;
        Eigen::Matrix<float, 3, 1> x_hat;

        //output
        float u;
        
    public:
        ADRC(float t_sampling, float b0, float t_settling, float setpoint, int k_eso);
        void updateEso(float y, float u);
        float computeControlSignal(float setpoint, float b0);
        void resetADRC();
};