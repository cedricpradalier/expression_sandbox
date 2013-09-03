
#include "expressions/Expressions.h"
#include "expressions/ExpressionImpl.h"

Expression::Expression(const std::string & s) {
    e.reset(new ExpressionVariable(s));
}

Expression::Expression(double value) {
    e.reset(new ExpressionConst(value));
}

Expression::Expression(const Expression & x) {
    e = x.e;
}


double Expression::value(const SubstitutionMap & map) const {
    return e->value(map);
}

Expression Expression::jacobian(const VariableId & ids) const {
    return e->jacobian(ids);
}

VariableId Expression::getVariables() const {
    size_t i =0;
    std::set<std::string> names;
    VariableId ids;
    e->collectVariables(names);
    for (std::set<std::string>::const_iterator it = names.begin();
            it != names.end() ; it++,i++) {
        ids.insert(VariableId::value_type(*it,i));
    }
    return ids;
}


Expression Expression::operator+() const {
    return ExpressionPtr(new ExpressionUnaryOperator(EXP_UNARY_PLUS, e));
}

Expression Expression::operator-() const {
    return ExpressionPtr(new ExpressionUnaryOperator(EXP_UNARY_MINUS, e));
}

Expression Expression::operator+(const Expression & x) const {
    return Expression(ExpressionPtr(new ExpressionBinaryOperator(EXP_BINARY_PLUS, e, x.e)));
}

Expression Expression::operator-(const Expression & x) const {
    return Expression(ExpressionPtr(new ExpressionBinaryOperator(EXP_BINARY_MINUS, e, x.e)));
}

Expression Expression::operator*(const Expression & x) const {
    return Expression(ExpressionPtr(new ExpressionBinaryOperator(EXP_BINARY_MULTIPLY, e, x.e)));
}

Expression Expression::operator/(const Expression & x) const {
    return Expression(ExpressionPtr(new ExpressionBinaryOperator(EXP_BINARY_DIVIDE, e, x.e)));
}


std::string Expression::toString() const {
    return e->toString();
}

Expression cos(const Expression & x) {
    return Expression(ExpressionPtr(new ExpressionUnaryOperator(EXP_UNARY_COS,x.e)));
}

Expression sin(const Expression & x) {
    return Expression(ExpressionPtr(new ExpressionUnaryOperator(EXP_UNARY_SIN,x.e)));
}

Expression exp(const Expression & x) {
    return Expression(ExpressionPtr(new ExpressionUnaryOperator(EXP_UNARY_EXP,x.e)));
}


