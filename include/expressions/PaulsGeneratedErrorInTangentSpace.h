
#ifndef __PAULSGENERATEDERRORINTANGENTSPACE_H__
#define __PAULSGENERATEDERRORINTANGENTSPACE_H__
#include <map>
#include <string>
#include <math.h>
#include <Eigen/Core>
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
           Eigen::Matrix<double, 2, 3> * Jq_1_2,
           Eigen::Matrix<double, 2, 3> * Jq_2_3,
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
               double __x1 = pow(q_1_2(0), -__x0);
               double __x2 = pow(q_1_2(1), -__x0);
               double __x3 = pow(q_1_2(2), -__x0);
               double __x4 = pow(q_1_2(3), -__x0);
               double __x5 = pow(q_2_3(0), -__x0);
               double __x6 = pow(q_2_3(1), -__x0);
               double __x7 = pow(q_2_3(2), -__x0);
               double __x8 = pow(q_2_3(3), -__x0);
               double __x9 = __x2 + __x4;
               double __x10 = __x3 + __x4;
               double __x11 = __x6 + __x7;
               double __x12 = -__x0*__x6 - 1;
               double __x13 = __x7 + __x8;
               double __x14 = q_1_2(0)*q_1_2(1) + q_1_2(2)*q_1_2(3);
               double __x15 = q_1_2(0)*q_1_2(3) + q_1_2(1)*q_1_2(2);
               double __x16 = q_2_3(0)*q_2_3(1) + q_2_3(2)*q_2_3(3);
               double __x17 = q_2_3(0)*q_2_3(2) + q_2_3(1)*q_2_3(3);
               double __x18 = q_2_3(0)*q_2_3(3) + q_2_3(1)*q_2_3(2);
               double __x19 = -q_1_2(0)*q_1_2(2) + q_1_2(1)*q_1_2(3);
               double __x20 = -__x19;
               double __x21 = -q_1_2(0)*q_1_2(3) + q_1_2(1)*q_1_2(2);
               double __x22 = -__x21;
               double __x23 = q_2_3(0)*q_2_3(1) - q_2_3(2)*q_2_3(3);
               double __x24 = q_2_3(0)*q_2_3(2) - q_2_3(1)*q_2_3(3);
               double __x25 = q_2_3(0)*q_2_3(3) - q_2_3(1)*q_2_3(2);
               double __x26 = -__x14;
               double __x27 = -__x15;
               double __x28 = -__x9;
               double __x29 = -__x10;
               double __x30 = -__x13;
               double __x31 = -__x0*__x7 + __x12;
               double __x32 = -__x0*__x8 + __x12;
               double __x33 = -__x0*__x7 - __x0*__x8 - 1;
               double __x34 = __x1 + __x28 + __x3;
               double __x35 = __x1 + __x2 + __x29;
               double __x36 = -__x11 + __x5 + __x8;
               double __x37 = __x5 - __x6 + __x7 - __x8;
               double __x38 = __x30 + __x5 + __x6;
               double __x39 = P_3(0)*__x0*__x24 - P_3(1)*__x0*__x16;
               double __x40 = -__x39;
               double __x41 = -P_3(0)*__x0*__x18 + P_3(2)*__x0*__x23;
               double __x42 = P_3(1)*__x0*__x25 - P_3(2)*__x0*__x17;
               double __x43 = -__x42;
               double __x44 = P_3(0)*__x0*__x13 + P_3(0) + __x42;
               double __x45 = P_3(1)*__x0*(__x6 + __x8) + P_3(1) + __x41;
               double __x46 = -P_3(2)*__x0*__x11 - P_3(2) + __x40;
               double __x47 = P_3(2)*__x36 + __x39;
               double __x48 = P_3(1)*__x37 + __x41;
               double __x49 = P_3(0)*__x38 + __x42;
               double __x50 = -P_3(2)*__x36 + __x40;
               double __x51 = -__x48;
               double __x52 = -P_3(0);
               (*Jq_1_2)(0,0) = -__x0*(__x20*__x45 + __x27*__x46);
               (*Jq_1_2)(0,1) = __x0*__x20*__x44 + __x35*__x46;
               (*Jq_1_2)(0,2) = __x0*__x15*__x44 + __x35*__x45;
               (*Jq_1_2)(1,0) = __x0*__x14*__x45 + __x34*(P_3(2)*__x0*__x11 + P_3(2) + __x39);
               (*Jq_1_2)(1,1) = __x0*(__x14*(P_3(0)*__x0*__x30 + __x43 + __x52) + __x22*__x46);
               (*Jq_1_2)(1,2) = __x0*__x22*__x45 - __x34*(__x0*__x30*__x52 - __x43 - __x52);
               (*Jq_2_3)(0,0) = -__x0*(__x20*__x51 - __x27*__x50);
               (*Jq_2_3)(0,1) = __x0*__x10*__x47 + __x0*__x20*(__x38*__x52 + __x43) + __x47;
               (*Jq_2_3)(0,2) = __x0*__x10*__x51 + __x0*__x27*__x49 + __x51;
               (*Jq_2_3)(1,0) = __x0*__x26*__x48 + __x0*__x50*__x9 + __x50;
               (*Jq_2_3)(1,1) = __x0*(__x14*__x49 + __x22*__x47);
               (*Jq_2_3)(1,2) = __x0*__x22*__x51 + __x0*__x49*__x9 + __x49;
               (*JP_3)(0,0) = __x0*__x18*__x27 + __x0*__x19*__x24 + __x10*__x33 + __x30 + 1.0L/2.0L;
               (*JP_3)(0,1) = __x0*__x16*__x20 + __x0*__x25*__x29 - __x25 + __x27*__x32;
               (*JP_3)(0,2) = __x0*__x10*__x17 + __x0*__x15*__x23 + __x17 + __x20*__x31;
               (*JP_3)(1,0) = __x0*__x14*__x24 + __x0*__x18*__x9 + __x18 + __x22*__x33;
               (*JP_3)(1,1) = __x0*__x16*__x26 + __x0*__x21*__x25 + __x32*__x9 - __x6 - __x8 + 1.0L/2.0L;
               (*JP_3)(1,2) = __x0*__x17*__x22 + __x0*__x23*__x28 - __x23 + __x26*__x31;
           }
           return true;
       }

};

}; // error_term

#endif // __PAULSGENERATEDERRORINTANGENTSPACE_H__
        