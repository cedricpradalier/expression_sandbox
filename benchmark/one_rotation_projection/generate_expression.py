#!/usr/bin/python
from sympy.matrices import *
from sympy import sin,cos,sqrt,Subs

from sys import path
path.append('../../src')

from Quaternions import *
from ErrorTerm import *

def createExp(q_1_2, P_2):
    p_1 = quaternion_rotate(q_1_2,P_2)
    return Matrix([p_1[0] / 2,p_1[1] / 2])

def createE():
    E = ErrorTerm(namespace="error_term")
    
    # U = E.declareData("U",2);
    
    q_1_2 = E.declareVariable('q_1_2',4);
    P_2 = E.declareVariable('P_2',3);
    
    E.setFunction(createExp(q_1_2, P_2));
    E.write_header(classname="PaulsGeneratedError",path="generated")

def createEt():
    Et = ErrorTerm(namespace="error_term")
    q_1_2_const = Et.declareData('q_1_2',4);
    
    phi_1_2 = Et.declareVariable('phi_1_2',3);
    P_2 = Et.declareVariable('P_2',3);

    q_1_2 = quaternion_mult(vector_to_pure_imag_quaternion_plus_identity(phi_1_2), q_1_2_const);

    exp = createExp(q_1_2, P_2)
    Et.setFunction(exp)

    for v in phi_1_2:
        print v
        Et.J = Et.J.replace(v, 0)
        Et.function = Et.function.replace(v, 0)
    
    Et.variables.remove(('phi_1_2',phi_1_2, 3));
    Et.variables.insert(0, ('q_1_2', q_1_2, 3));
    Et.data.pop('q_1_2');

    Et.write_header(classname="PaulsGeneratedErrorInTangentSpace",path="generated")

createE()
createEt()
