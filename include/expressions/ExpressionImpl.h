#ifndef EXPRESSION_IMPL_H
#define EXPRESSION_IMPL_H

#include <set>
#include <map>
#include <string>
#include <math.h>
#include <assert.h>
#include <boost/shared_ptr.hpp>
#include <boost/format.hpp>

#include "expressions/Expressions.h"

using boost::format;

typedef enum {
    EXP_UNARY_PLUS, 
    EXP_UNARY_MINUS,
    EXP_UNARY_COS,
    EXP_UNARY_SIN,
    EXP_UNARY_EXP,
} ExpressionUnaryOperatorType;

typedef enum {
    EXP_BINARY_PLUS,
    EXP_BINARY_MINUS,
    EXP_BINARY_MULTIPLY,
    EXP_BINARY_DIVIDE
} ExpressionBinaryOperatorType;


class ExpressionImpl {
    public:
        ExpressionImpl() {}
        virtual ~ExpressionImpl() {}
        virtual double value(const SubstitutionMap & map) const = 0;
        virtual ExpressionPtr jacobian(const VariableId & ids) const = 0;
        virtual std::string toString() const = 0;
        virtual void collectVariables(std::set<std::string> names) const = 0;
};

class ExpressionConst : public ExpressionImpl {
    protected:
        double val;
    public:
        ExpressionConst(double v) : val(v) {}
        virtual ~ExpressionConst() {}
        virtual double value(const SubstitutionMap & map) const {
            return val;
        }
        virtual ExpressionPtr jacobian(const VariableId & ids) const {
            return ExpressionPtr(new ExpressionConst(0.0));
        }
        virtual std::string toString() const {return str(format("%g") % val);}
        virtual void collectVariables(std::set<std::string> names) const {}
};


class ExpressionVariable : public ExpressionImpl {
    protected:
        std::string name;
        // could add some flags there, assume, ...
    public:
        ExpressionVariable(const std::string & s) : name(s) {}
        virtual ~ExpressionVariable() {}
        virtual double value(const SubstitutionMap & map) const {
            SubstitutionMap::const_iterator it = map.find(name);
            // Exception
            assert(it != map.end());
            return it->second;
        }
        virtual ExpressionPtr jacobian(const VariableId & ids) const {
            // TODO: because this requires matrix/vector types
            return ExpressionPtr(new ExpressionConst(1.0));
        }

        virtual std::string toString() const {return name;}
        virtual void collectVariables(std::set<std::string> names) const {
            names.insert(name);
        }
};

class ExpressionBinaryOperator;

class ExpressionUnaryOperator : public ExpressionImpl {
    protected:
        ExpressionUnaryOperatorType optype;
        ExpressionPtr arg;
    public:
        ExpressionUnaryOperator(ExpressionUnaryOperatorType ot, ExpressionPtr a) :
            optype(ot), arg(a) {}
        virtual ~ExpressionUnaryOperator() {}
        virtual double value(const SubstitutionMap & map) const {
            switch (optype) {
                case EXP_UNARY_PLUS: 
                    return arg->value(map);
                case EXP_UNARY_MINUS: 
                    return -arg->value(map);
                case EXP_UNARY_COS: 
                    return ::cos(arg->value(map));
                case EXP_UNARY_SIN: 
                    return ::sin(arg->value(map));
                case EXP_UNARY_EXP: 
                    return ::exp(arg->value(map));
                default: 
                    // exception
                    assert(false);
            }
            return 0.0; // unreachable, for the compiler
        }
        virtual ExpressionPtr jacobian(const VariableId & ids) const ;

        virtual std::string toString() const {
            switch (optype) {
                case EXP_UNARY_PLUS: 
                    return "(+" + arg->toString()+")";
                case EXP_UNARY_MINUS: 
                    return "(-" + arg->toString()+")";
                case EXP_UNARY_COS: 
                    return "cos(" + arg->toString() + ")";
                case EXP_UNARY_SIN: 
                    return "sin(" + arg->toString() + ")";
                case EXP_UNARY_EXP: 
                    return "exp(" + arg->toString() + ")";
                default: 
                    // exception
                    assert(false);
            }
            return std::string();
        }
        virtual void collectVariables(std::set<std::string> names) const {
            arg->collectVariables(names);
        }
};

class ExpressionBinaryOperator : public ExpressionImpl {
    protected:
        ExpressionBinaryOperatorType optype;
        ExpressionPtr arg1,arg2;
    public:
        ExpressionBinaryOperator(ExpressionBinaryOperatorType ot, ExpressionPtr a, ExpressionPtr b) :
            optype(ot), arg1(a), arg2(b) {}
        virtual ~ExpressionBinaryOperator() {}
        virtual double value(const SubstitutionMap & map) const {
            switch (optype) {
                case EXP_BINARY_PLUS: 
                    return arg1->value(map) + arg2->value(map);
                case EXP_BINARY_MINUS: 
                    return arg1->value(map) - arg2->value(map);
                case EXP_BINARY_MULTIPLY: 
                    return arg1->value(map) * arg2->value(map);
                case EXP_BINARY_DIVIDE: 
                    return arg1->value(map) / arg2->value(map);
                default: 
                    // exception
                    assert(false);
            }
            return 0.0; // unreachable, for the compiler
        }
        virtual ExpressionPtr jacobian(const VariableId & ids) const ;

        virtual std::string toString() const {
            switch (optype) {
                case EXP_BINARY_PLUS: 
                    return "("+arg1->toString() +"+"+ arg2->toString()+")";
                case EXP_BINARY_MINUS: 
                    return "("+arg1->toString() +"-"+ arg2->toString()+")";
                case EXP_BINARY_MULTIPLY: 
                    return "("+arg1->toString() +"*"+ arg2->toString()+")";
                case EXP_BINARY_DIVIDE: 
                    return "("+arg1->toString() +"/"+ arg2->toString()+")";
                default: 
                    // exception
                    assert(false);
            }
            return std::string();
        }
        virtual void collectVariables(std::set<std::string> names) const {
            arg1->collectVariables(names);
            arg2->collectVariables(names);
        }
};


#endif // EXPRESSION_IMPL_H
