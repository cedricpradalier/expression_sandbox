
#include "expressions/ExpressionImpl.h"


ExpressionPtr ExpressionUnaryOperator::jacobian(const VariableId & ids) const {
    switch (optype) {
        case EXP_UNARY_PLUS: 
            return arg->jacobian(ids);
        case EXP_UNARY_MINUS: 
            return ExpressionPtr(new ExpressionUnaryOperator(EXP_UNARY_MINUS,arg->jacobian(ids)));
        case EXP_UNARY_COS: 
            return ExpressionPtr(new ExpressionBinaryOperator(EXP_BINARY_MULTIPLY,
                        ExpressionPtr(new ExpressionUnaryOperator(EXP_UNARY_MINUS,
                                ExpressionPtr(new ExpressionUnaryOperator(EXP_UNARY_SIN, arg)))),
                        arg->jacobian(ids)));
        case EXP_UNARY_SIN: 
            return ExpressionPtr(new ExpressionBinaryOperator(EXP_BINARY_MULTIPLY,
                        ExpressionPtr(new ExpressionUnaryOperator(EXP_UNARY_SIN, arg)),
                        arg->jacobian(ids)));
        case EXP_UNARY_EXP: 
            return ExpressionPtr(new ExpressionBinaryOperator(EXP_BINARY_MULTIPLY,
                        ExpressionPtr(new ExpressionUnaryOperator(EXP_UNARY_EXP,arg)),
                        arg->jacobian(ids)));
        default: 
            // exception
            assert(false);
    }
    return ExpressionPtr(); // unreachable, for the compiler
}

ExpressionPtr ExpressionBinaryOperator::jacobian(const VariableId & ids) const {
    switch (optype) {
        case EXP_BINARY_PLUS: 
            return arg1->jacobian(ids),arg2->jacobian(ids);
        case EXP_BINARY_MINUS: 
            return ExpressionPtr(new ExpressionBinaryOperator(EXP_BINARY_MINUS,arg1->jacobian(ids),arg2->jacobian(ids)));
        case EXP_BINARY_MULTIPLY: 
                return ExpressionPtr(new ExpressionBinaryOperator(EXP_BINARY_PLUS,
                            ExpressionPtr(new ExpressionBinaryOperator(EXP_BINARY_MULTIPLY,arg1->jacobian(ids),arg2)),
                            ExpressionPtr(new ExpressionBinaryOperator(EXP_BINARY_MULTIPLY,arg1,arg2->jacobian(ids)))));
        case EXP_BINARY_DIVIDE: 
                {
                    ExpressionPtr numerator(new ExpressionBinaryOperator(EXP_BINARY_MINUS,
                                ExpressionPtr(new ExpressionBinaryOperator(EXP_BINARY_MULTIPLY,arg1->jacobian(ids),arg2)),
                                ExpressionPtr(new ExpressionBinaryOperator(EXP_BINARY_MULTIPLY,arg1,arg2->jacobian(ids)))));
                    ExpressionPtr denominator(new ExpressionBinaryOperator(EXP_BINARY_MULTIPLY,arg2,arg2));
                    return ExpressionPtr(new ExpressionBinaryOperator(EXP_BINARY_DIVIDE,numerator,denominator));
                }
        default: 
                // exception
                assert(false);
    }
    return ExpressionPtr(); // unreachable, for the compiler
}

