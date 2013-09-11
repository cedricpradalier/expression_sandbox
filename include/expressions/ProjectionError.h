
#ifndef __PROJECTIONERROR_H__
#define __PROJECTIONERROR_H__
#include <map>
#include <string>
#include <math.h>
#include <Eigen/Core>
// This file is auto-generated. Do not modify


namespace error_term {

class ProjectionError 
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
        VariableMap variables;
        // Function value and jacobian
        Eigen::VectorXd F;
        Eigen::MatrixXd J;
        // Data
        Eigen::MatrixXd U;

    public:
        // Constructor
        ProjectionError(
           const Eigen::MatrixXd & _U_
        ) :
           F(2), J(2,8),
           U(_U_)
        {
           variables["alpha"] = Variable(0,1);
           variables["f"] = Variable(1,1);
           variables["x"] = Variable(2,3);
           variables["t"] = Variable(5,3);
        }


        const Eigen::VectorXd & getValue() const {return F;}
        const Eigen::MatrixXd & getJacobian() const {return J;}
        const VariableMap & getVariables() const {return variables;}

        // Evaluate J and F
        bool evaluate(
           double alpha,
           double f,
           const Eigen::MatrixXd & x,
           const Eigen::MatrixXd & t,
           bool evalF=true,bool evalJ=true) {
           if (evalF) {
               double __x0 = sin(alpha);
               double __x1 = cos(alpha);
               double __x2 = -f/x(2);
               F(0) = U(0) + __x2*(-__x0*x(1) + __x1*x(0));
               F(1) = U(1) + __x2*(__x0*x(0) + __x1*x(1));
           }
           if (evalJ) {
               double __x0 = sin(alpha);
               double __x1 = cos(alpha);
               double __x2 = __x0*x(0) + __x1*x(1);
               double __x3 = -__x0*x(1) + __x1*x(0);
               double __x4 = -1/x(2);
               double __x5 = f/pow(x(2), 2);
               double __x6 = f/x(2);
               double __x7 = __x1*__x4*f;
               J(0,0) = __x2*__x6;
               J(0,1) = __x3*__x4;
               J(0,2) = __x7;
               J(0,3) = __x0*__x6;
               J(0,4) = __x3*__x5;
               J(0,5) = 0;
               J(0,6) = 0;
               J(0,7) = 0;
               J(1,0) = __x3*__x4*f;
               J(1,1) = __x2*__x4;
               J(1,2) = -__x0*__x6;
               J(1,3) = __x7;
               J(1,4) = __x2*__x5;
               J(1,5) = 0;
               J(1,6) = 0;
               J(1,7) = 0;
           }
           return true;
       }

};

}; // error_term

#endif // __PROJECTIONERROR_H__
        