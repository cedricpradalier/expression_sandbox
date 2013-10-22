#!/usr/bin/python
from sympy.matrices import *
from sympy import sin,cos,sqrt,Subs

from sys import path
path.append('../../src')

from Quaternions import *
from ErrorTerm import *

def createExp(q_1_2, q_2_3, P_3):
    q_1_2_inv = quaternion_inv(q_1_2)
    p_2 = quaternion_rotate(q_2_3,P_3)
    p_1 = quaternion_rotate(q_1_2_inv,p_2)
    # F = U - Up
    return Matrix([p_1[0] / 2,p_1[1] / 2])

def createE():
    E = ErrorTerm(namespace="error_term")
    
    # U = E.declareData("U",2);
    
    q_1_2 = E.declareVariable('q_1_2',4);
    q_2_3 = E.declareVariable('q_2_3',4);
    P_3 = E.declareVariable('P_3',3);
    
    E.setFunction(createExp(q_1_2, q_2_3, P_3));
    E.write_header(classname="PaulsGeneratedError",path="generated")

def createEt():
    Et = ErrorTerm(namespace="error_term")
    q_1_2_const = Et.declareData('q_1_2',4);
    q_2_3_const = Et.declareData('q_2_3',4);
    
    phi_1_2 = Et.declareVariable('phi_1_2',3);
    phi_2_3 = Et.declareVariable('phi_2_3',3);
    P_3 = Et.declareVariable('P_3',3);
    
    
    q_1_2 = quaternion_mult(vector_to_pure_imag_quaternion_plus_identity(phi_1_2), q_1_2_const);
    q_2_3 = quaternion_mult(vector_to_pure_imag_quaternion_plus_identity(phi_2_3), q_2_3_const);

    exp = createExp(q_1_2, q_2_3, P_3)
    Et.setFunction(exp)

    for e in (phi_1_2, phi_2_3):
        for v in e:
            print v
            Et.J = Et.J.replace(v, 0)
            Et.function = Et.function.replace(v, 0)
    
    Et.variables.remove(('phi_1_2',phi_1_2, 3));
    Et.variables.remove(('phi_2_3',phi_2_3, 3));
    Et.variables.insert(0, ('q_2_3', q_2_3, 3));
    Et.variables.insert(0, ('q_1_2', q_1_2, 3));
    Et.data.pop('q_1_2');
    Et.data.pop('q_2_3');

    Et.write_header(classname="PaulsGeneratedErrorInTangentSpace",path="generated")

createE()
createEt()
