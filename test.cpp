#include <iostream>
#include "Eigen/Dense"

using namespace std;

int main(){
    Eigen::Matrix<int, 2, 2> A;
    Eigen::Matrix<int, 2, 2> B;

    A << 1, 2
        , 3, 4;

    B << 5, 6
        , 7, 8;

    
    cout << A << endl;
    cout << B << endl;
    Eigen::MatrixXi C = A*B;
    cout << C << endl;
    return 0;
}