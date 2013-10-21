
#ifndef __PAULSGENERATEDERROR_H__
#define __PAULSGENERATEDERROR_H__
#include <map>
#include <string>
#include <math.h>
#include <Eigen/Core>
// This file is auto-generated. Do not modify


namespace error_term {

class PaulsGeneratedError 
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
        PaulsGeneratedError(

        )        {
           //variables["q_1_2"] = Variable(0,4);
           //variables["q_2_3"] = Variable(4,4);
           //variables["P_3"] = Variable(8,3);
        }


        //const Eigen::VectorXd & getValue() const {return F;}
        //const Eigen::MatrixXd & getJacobian() const {return J;}
        //const VariableMap & getVariables() const {return variables;}

        // Evaluate J and F
        bool evaluate(
           const Eigen::Matrix<double, 4, 1> & q_1_2,
           const Eigen::Matrix<double, 4, 1> & q_2_3,
           const Eigen::Matrix<double, 3, 1> & P_3,
           Eigen::Matrix<double, 2, 1> * F,
           Eigen::Matrix<double, 2, 4> * Jq_1_2,
           Eigen::Matrix<double, 2, 4> * Jq_2_3,
           Eigen::Matrix<double, 2, 3> * JP_3) {
           if (F) {
               double __x0 = -2;
               double __x1 = pow(q_1_2(3), -__x0);
               double __x2 = pow(q_2_3(1), -__x0);
               double __x3 = pow(q_2_3(2), -__x0);
               double __x4 = pow(q_2_3(3), -__x0);
               double __x5 = __x2 + __x4;
               double __x6 = __x3 + __x4;
               double __x7 = q_2_3(0)*q_2_3(2) + q_2_3(1)*q_2_3(3);
               double __x8 = q_2_3(0)*q_2_3(3) + q_2_3(1)*q_2_3(2);
               double __x9 = q_2_3(0)*q_2_3(1) - q_2_3(2)*q_2_3(3);
               double __x10 = -q_2_3(0)*q_2_3(3) + q_2_3(1)*q_2_3(2);
               double __x11 = P_3(0)*__x0*__x6 + P_3(0) - P_3(1)*__x0*__x10 - P_3(2)*__x0*__x7;
               double __x12 = P_3(0)*__x0*(q_2_3(0)*q_2_3(2) - q_2_3(1)*q_2_3(3)) - P_3(1)*__x0*(q_2_3(0)*q_2_3(1) + q_2_3(2)*q_2_3(3)) + P_3(2)*__x0*(__x2 + __x3) + P_3(2);
               double __x13 = -P_3(0)*__x0*__x8 + P_3(1)*__x0*__x5 + P_3(1) + P_3(2)*__x0*__x9;
               double __x14 = q_1_2(0)*q_1_2(3);
               double __x15 = q_1_2(1)*q_1_2(2);
               (*F)(0) = -P_3(0)*__x6 + (1.0L/2.0L)*P_3(0) + P_3(1)*__x10 + P_3(2)*__x7 - __x11*(__x1 + pow(q_1_2(2), -__x0)) - __x12*(q_1_2(0)*q_1_2(2) - q_1_2(1)*q_1_2(3)) + __x13*(__x14 + __x15);
               (*F)(1) = P_3(0)*__x8 - P_3(1)*__x5 + (1.0L/2.0L)*P_3(1) - P_3(2)*__x9 - __x11*(__x14 - __x15) + __x12*(q_1_2(0)*q_1_2(1) + q_1_2(2)*q_1_2(3)) - __x13*(__x1 + pow(q_1_2(1), -__x0));
           }
           if (Jq_1_2 && Jq_2_3 && JP_3) {
               double __x0 = -2;
               double __x1 = pow(q_1_2(3), -__x0);
               double __x2 = pow(q_2_3(1), -__x0);
               double __x3 = pow(q_2_3(2), -__x0);
               double __x4 = pow(q_2_3(3), -__x0);
               double __x5 = __x1 + pow(q_1_2(1), -__x0);
               double __x6 = __x1 + pow(q_1_2(2), -__x0);
               double __x7 = -__x0*__x2 - 1;
               double __x8 = __x3 + __x4;
               double __x9 = -__x4 + 1.0L/2.0L;
               double __x10 = P_3(0)*q_2_3(1) + P_3(1)*q_2_3(2);
               double __x11 = P_3(0)*q_2_3(1) + P_3(2)*q_2_3(3);
               double __x12 = P_3(1)*q_2_3(2) + P_3(2)*q_2_3(3);
               double __x13 = q_1_2(0)*q_1_2(1) + q_1_2(2)*q_1_2(3);
               double __x14 = q_1_2(0)*q_1_2(3) + q_1_2(1)*q_1_2(2);
               double __x15 = q_2_3(0)*q_2_3(1) + q_2_3(2)*q_2_3(3);
               double __x16 = q_2_3(0)*q_2_3(2) + q_2_3(1)*q_2_3(3);
               double __x17 = q_2_3(0)*q_2_3(3) + q_2_3(1)*q_2_3(2);
               double __x18 = P_3(0)*q_2_3(2) - P_3(1)*q_2_3(1);
               double __x19 = P_3(0)*q_2_3(3) - P_3(2)*q_2_3(1);
               double __x20 = P_3(1)*q_2_3(3) - P_3(2)*q_2_3(2);
               double __x21 = q_1_2(0)*q_1_2(2) - q_1_2(1)*q_1_2(3);
               double __x22 = q_1_2(0)*q_1_2(3) - q_1_2(1)*q_1_2(2);
               double __x23 = q_2_3(0)*q_2_3(1) - q_2_3(2)*q_2_3(3);
               double __x24 = q_2_3(0)*q_2_3(2) - q_2_3(1)*q_2_3(3);
               double __x25 = q_2_3(0)*q_2_3(3) - q_2_3(1)*q_2_3(2);
               double __x26 = P_3(0)*q_2_3(0) + P_3(1)*__x0*q_2_3(3) + P_3(2)*q_2_3(2);
               double __x27 = P_3(0)*__x0*q_2_3(2) + P_3(1)*q_2_3(1) + P_3(2)*q_2_3(0);
               double __x28 = P_3(0)*q_2_3(3) + P_3(1)*q_2_3(0) + P_3(2)*__x0*q_2_3(1);
               double __x29 = -__x0*__x3 + __x7;
               double __x30 = -__x0*__x4 + __x7;
               double __x31 = -__x0*__x3 - __x0*__x4 - 1;
               double __x32 = P_3(0)*q_2_3(0) - P_3(1)*q_2_3(3) - P_3(2)*__x0*q_2_3(2);
               double __x33 = -P_3(0)*q_2_3(2) - P_3(1)*__x0*q_2_3(1) + P_3(2)*q_2_3(0);
               double __x34 = -P_3(0)*__x0*q_2_3(3) + P_3(1)*q_2_3(0) - P_3(2)*q_2_3(1);
               double __x35 = P_3(0)*__x0*__x8 + P_3(0) + P_3(1)*__x0*__x25 - P_3(2)*__x0*__x16;
               double __x36 = P_3(0)*__x0*__x24 - P_3(1)*__x0*__x15 + P_3(2)*__x0*(__x2 + __x3) + P_3(2);
               double __x37 = -P_3(0)*__x0*__x17 + P_3(1)*__x0*(__x2 - __x9 + 1.0L/2.0L) + P_3(1) + P_3(2)*__x0*__x23;
               double __x38 = -__x35;
               double __x39 = -__x36;
               double __x40 = __x37*q_1_2(1);
               double __x41 = __x37*q_1_2(3);
               double __x42 = __x36*q_1_2(1);
               (*Jq_1_2)(0,0) = __x39*q_1_2(2) + __x41;
               (*Jq_1_2)(0,1) = __x36*q_1_2(3) + __x37*q_1_2(2);
               (*Jq_1_2)(0,2) = __x0*__x35*q_1_2(2) + __x39*q_1_2(0) + __x40;
               (*Jq_1_2)(0,3) = __x0*__x35*q_1_2(3) + __x37*q_1_2(0) + __x42;
               (*Jq_1_2)(1,0) = __x38*q_1_2(3) + __x42;
               (*Jq_1_2)(1,1) = __x0*__x40 + __x35*q_1_2(2) + __x36*q_1_2(0);
               (*Jq_1_2)(1,2) = __x35*q_1_2(1) + __x36*q_1_2(3);
               (*Jq_1_2)(1,3) = __x0*__x41 + __x36*q_1_2(2) + __x38*q_1_2(0);
               (*Jq_2_3)(0,0) = -__x0*__x14*__x19 - __x0*__x18*__x21 - __x0*__x20*__x6 - __x20;
               (*Jq_2_3)(0,1) = __x0*__x12*__x6 + __x0*__x14*__x33 + __x0*__x21*__x28 + __x12;
               (*Jq_2_3)(0,2) = -__x0*__x11*__x14 - __x0*__x21*__x32 + __x0*__x27*__x6 + __x27;
               (*Jq_2_3)(0,3) = __x0*__x10*__x21 - __x0*__x14*__x26 - __x0*__x34*__x6 - __x34;
               (*Jq_2_3)(1,0) = __x0*__x13*__x18 + __x0*__x19*__x5 - __x0*__x20*__x22 + __x19;
               (*Jq_2_3)(1,1) = __x0*__x12*__x22 - __x0*__x13*__x28 - __x0*__x33*__x5 - __x33;
               (*Jq_2_3)(1,2) = __x0*__x11*__x5 + __x0*__x13*__x32 + __x0*__x22*__x27 + __x11;
               (*Jq_2_3)(1,3) = -__x0*__x10*__x13 - __x0*__x22*__x34 + __x0*__x26*__x5 + __x26;
               (*JP_3)(0,0) = -__x0*__x14*__x17 - __x0*__x21*__x24 + __x31*__x6 - __x8 + 1.0L/2.0L;
               (*JP_3)(0,1) = __x0*__x15*__x21 - __x0*__x25*__x6 - __x14*__x30 - __x25;
               (*JP_3)(0,2) = __x0*__x14*__x23 + __x0*__x16*__x6 + __x16 + __x21*__x29;
               (*JP_3)(1,0) = __x0*__x13*__x24 + __x0*__x17*__x5 + __x17 + __x22*__x31;
               (*JP_3)(1,1) = -__x0*__x13*__x15 - __x0*__x22*__x25 - __x2 + __x30*__x5 + __x9;
               (*JP_3)(1,2) = __x0*__x16*__x22 - __x0*__x23*__x5 - __x13*__x29 - __x23;
           }
           return true;
       }

};

}; // error_term

#endif // __PAULSGENERATEDERROR_H__
        