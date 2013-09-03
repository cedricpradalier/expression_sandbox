#ifndef EXPRESSION_H
#define EXPRESSION_H

#include <set>
#include <map>
#include <string>
#include <assert.h>
#include <boost/shared_ptr.hpp>
#include <boost/format.hpp>

// Probably feasible directly with variables
typedef std::map<std::string,double> SubstitutionMap;
typedef std::map<std::string,int> VariableId;

// place holder
class ExpressionImpl;
typedef boost::shared_ptr<const ExpressionImpl> ExpressionPtr; 

class Expression {
    protected:
        ExpressionPtr e;
        Expression(ExpressionPtr f) : e(f) {}
    public:
        Expression() {}
        Expression(const std::string & s);
        Expression(double value);
        Expression(const Expression & x);

        double value(const SubstitutionMap & map) const;
        double operator()(const SubstitutionMap & map) const{
            return value(map);
        }

        Expression jacobian(const VariableId & ids) const;
        VariableId getVariables() const;  

        Expression operator+() const;
        Expression operator-() const;
        Expression operator+(const Expression & x) const;
        Expression operator-(const Expression & x) const;
        Expression operator*(const Expression & x) const;
        Expression operator/(const Expression & x) const;

        std::string toString() const;

        friend Expression cos(const Expression & e);
        friend Expression sin(const Expression & e);
        friend Expression exp(const Expression & e);
};




#endif // EXPRESSION_H
