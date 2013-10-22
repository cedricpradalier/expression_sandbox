
#ifndef __PAULSGENERATEDERRORINTANGENTSPACE_H__
#define __PAULSGENERATEDERRORINTANGENTSPACE_H__
#include <map>
#include <string>
#include <math.h>
#include <Eigen/Core>
#include <optimize_pow.hpp>
// This file is auto-generated. Do not modify


namespace error_term {

class PaulsGeneratedErrorInTangentSpace 
{
    public:
        struct Variable {
            size_t index;
            size_t dim;
            Variable() {}
            Variable(size_t i, size_t d) : index(i), dim(d) {}
        };
        typedef std::map<std::string, Variable> VariableMap;
    protected:
        // The list of variables, their index and dimension
        //VariableMap variables;
        // Function value and jacobian
        //Eigen::VectorXd F;
        //Eigen::MatrixXd J;
        // Data

    public:
        // Constructor
        PaulsGeneratedErrorInTangentSpace(

        )        {
           //variables["q_1_2"] = Variable(0,4);
           //variables["P_2"] = Variable(4,3);
        }


        //const Eigen::VectorXd & getValue() const {return F;}
        //const Eigen::MatrixXd & getJacobian() const {return J;}
        //const VariableMap & getVariables() const {return variables;}

        // Evaluate J and F
        bool evaluate(
           const Eigen::Matrix<double, 4, 1> & q_1_2,
           const Eigen::Matrix<double, 3, 1> & P_2,
           Eigen::Matrix<double, 2, 1> * F,
           Eigen::Matrix<double, 2, 3> * Jq_1_2,
           Eigen::Matrix<double, 2, 3> * JP_2) {
           if (F) {
               double __x0 = pow(q_1_2(3), 2);
               double __x1 = -P_2(1);
               double __x2 = q_1_2(0)*q_1_2(3);
               double __x3 = q_1_2(1)*q_1_2(2);
               (*F)(0) = -P_2(0)*(__x0 + pow(q_1_2(2), 2)) + (1.0L/2.0L)*P_2(0) + P_2(2)*(q_1_2(0)*q_1_2(2) + q_1_2(1)*q_1_2(3)) + __x1*(__x2 - __x3);
               (*F)(1) = P_2(0)*(__x2 + __x3) + (1.0L/2.0L)*P_2(1) - P_2(2)*(q_1_2(0)*q_1_2(1) - q_1_2(2)*q_1_2(3)) + __x1*(__x0 + pow(q_1_2(1), 2));
           }
           if (Jq_1_2 && JP_2) {
               double __x0 = -2;
               double __x1 = pow(q_1_2(0), -__x0);
               double __x2 = pow(q_1_2(1), -__x0);
               double __x3 = pow(q_1_2(2), -__x0);
               double __x4 = 1.0L/2.0L - pow(q_1_2(3), -__x0);
               double __x5 = q_1_2(0)*q_1_2(2) + q_1_2(1)*q_1_2(3);
               double __x6 = q_1_2(0)*q_1_2(3) + q_1_2(1)*q_1_2(2);
               double __x7 = __x1 - __x2;
               double __x8 = -P_2(2)*__x0;
               double __x9 = q_1_2(0)*q_1_2(1);
               double __x10 = q_1_2(0)*q_1_2(3);
               double __x11 = q_1_2(1)*q_1_2(2);
               double __x12 = q_1_2(2)*q_1_2(3);
               double __x13 = -__x3;
               double __x14 = P_2(1)*(q_1_2(0)*q_1_2(1) + q_1_2(2)*q_1_2(3));
               double __x15 = P_2(0)*(q_1_2(0)*q_1_2(2) - q_1_2(1)*q_1_2(3));
               double __x16 = P_2(2)*(-__x3 - __x4 + __x7 + 1.0L/2.0L);
               (*Jq_1_2)(0,0) = 0;
               (*Jq_1_2)(0,1) = -__x0*__x14 + __x0*__x15 + __x16;
               (*Jq_1_2)(0,2) = P_2(0)*__x0*__x6 - P_2(1)*(-__x13 + __x4 + __x7 - 1.0L/2.0L) + __x8*(-__x12 + __x9);
               (*Jq_1_2)(1,0) = __x0*__x14 - __x0*__x15 - __x16;
               (*Jq_1_2)(1,1) = 0;
               (*Jq_1_2)(1,2) = P_2(0)*(__x1 + __x13 + __x2 + __x4 - 1.0L/2.0L) + P_2(1)*__x0*(__x10 - __x11) + __x5*__x8;
               (*JP_2)(0,0) = __x13 + __x4;
               (*JP_2)(0,1) = -__x10 + __x11;
               (*JP_2)(0,2) = __x5;
               (*JP_2)(1,0) = __x6;
               (*JP_2)(1,1) = -__x2 + __x4;
               (*JP_2)(1,2) = __x12 - __x9;
           }
           return true;
       }

};

}; // error_term

#endif // __PAULSGENERATEDERRORINTANGENTSPACE_H__
        