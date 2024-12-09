#include "Eigen/Dense"

class ADRC{
    private:
        // Input
        float t_sampling;
        float b0;
        float t_settling; // settling time 2%


        //Computed
        float s_cl;
        float s_eso;
        float z_eso;
        float l1, l2, l3;
        float Kp, Kd;
};