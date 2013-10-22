
#include <iostream>
#include "generated/PaulsGeneratedError.h"
#include <Eigen/Core>

using namespace std;

int main() 
{
//    Eigen::VectorXd U(2); U << 120,230;
    Eigen::VectorXd q_1_2(4); q_1_2 << 0.5, 0.0, 0.5, 0.0;
    Eigen::VectorXd P_3(3); P_3 << 1.0, 0.0, 0.0;

    error_term::PaulsGeneratedError pge;

    Eigen::Vector2d F;
    Eigen::Matrix<double, 2, 4> Jq12, Jq23;
    Eigen::Matrix<double, 2, 3> JP3;
    pge.evaluate(q_1_2, P_3, &F, &Jq12,  &JP3);
    cout << "F" << endl << F << endl;
    cout << "Jq12" << endl << Jq12 << endl;
    cout << "JP3" << endl << JP3 << endl;
    return 0;
}

