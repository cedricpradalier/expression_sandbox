
#include <iostream>
#include "expressions/PaulsGeneratedError.h"

using namespace std;

int main() 
{
    Eigen::VectorXd U(2); U << 120,230;
    Eigen::VectorXd q_1_2(4); q_1_2 << 0.5, 0.0, 0.5, 0.0;
    Eigen::VectorXd q_2_3(4); q_2_3 << 0.5, 0.5, 0.0, 0.0;
    Eigen::VectorXd P_3(3); P_3 << 1.0, 0.0, 0.0;

    error_term::PaulsGeneratedError pge(U);

    pge.evaluate(q_1_2, q_2_3, P_3);
    cout << "F" << endl << pge.getValue() << endl;
    cout << "J" << endl << pge.getJacobian() << endl;

    return 0;
}

