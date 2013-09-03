
#include "expressions/ExpressionImpl.h"


ExpressionPtr ExpressionUnaryOperator::jacobian(const VariableId & ids) const {
    switch (optype) {
        case EXP_UNARY_PLUS: 
            return arg->jacobian(ids);
        case EXP_UNARY_MINUS: 
            return EX(ExpressionUnaryOperator(EXP_UNARY_MINUS,arg->jacobian(ids)));
        case EXP_UNARY_COS: 
            return EX(ExpressionBinaryOperator(EXP_BINARY_MULTIPLY,
                        EX(ExpressionUnaryOperator(EXP_UNARY_MINUS,
                                EX(ExpressionUnaryOperator(EXP_UNARY_SIN, arg)))),
                        arg->jacobian(ids)));
        case EXP_UNARY_SIN: 
            return EX(ExpressionBinaryOperator(EXP_BINARY_MULTIPLY,
                        EX(ExpressionUnaryOperator(EXP_UNARY_SIN, arg)),
                        arg->jacobian(ids)));
        case EXP_UNARY_EXP: 
            return EX(ExpressionBinaryOperator(EXP_BINARY_MULTIPLY,
                        EX(ExpressionUnaryOperator(EXP_UNARY_EXP,arg)),
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
            return EX(ExpressionBinaryOperator(EXP_BINARY_MINUS,arg1->jacobian(ids),arg2->jacobian(ids)));
        case EXP_BINARY_MULTIPLY: 
                return EX(ExpressionBinaryOperator(EXP_BINARY_PLUS,
                            EX(ExpressionBinaryOperator(EXP_BINARY_MULTIPLY,arg1->jacobian(ids),arg2)),
                            EX(ExpressionBinaryOperator(EXP_BINARY_MULTIPLY,arg1,arg2->jacobian(ids)))));
        case EXP_BINARY_DIVIDE: 
                {
                    ExpressionPtr numerator(new ExpressionBinaryOperator(EXP_BINARY_MINUS,
                                EX(ExpressionBinaryOperator(EXP_BINARY_MULTIPLY,arg1->jacobian(ids),arg2)),
                                EX(ExpressionBinaryOperator(EXP_BINARY_MULTIPLY,arg1,arg2->jacobian(ids)))));
                    ExpressionPtr denominator(new ExpressionBinaryOperator(EXP_BINARY_MULTIPLY,arg2,arg2));
                    return EX(ExpressionBinaryOperator(EXP_BINARY_DIVIDE,numerator,denominator));
                }
        default: 
                // exception
                assert(false);
    }
    return ExpressionPtr(); // unreachable, for the compiler
}

