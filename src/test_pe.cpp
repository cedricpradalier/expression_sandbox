
#include <iostream>
#include "expressions/ProjectionError.h"

using namespace std;

int main() 
{
    Eigen::VectorXd U(2); U << 120,230;
    double alpha = M_PI/3;
    double f = 1000;
    Eigen::VectorXd X(3); X << 1.0, 0.3, 5.0;
    Eigen::VectorXd t(3); t << 0.0, 0.0, 0.0;

    error_term::ProjectionError pe(U);

    pe.evaluate(alpha, f, X, t);
    cout << "F" << endl << pe.getValue() << endl;
    cout << "J" << endl << pe.getJacobian() << endl;

    return 0;
}

