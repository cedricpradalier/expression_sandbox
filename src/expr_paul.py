#!/usr/bin/python
from sympy.matrices import *
from sympy import sin,cos,sqrt,Subs
from ErrorTerm import *

def quaternion_inv(q):
    return Matrix([q[0],-q[1],-q[2],-q[3]])

def quaternion_norm(q):
    s = q.transpose() * q;
    return sqrt(s[0,0])

def quaternion_normalize(q):
    n = quaternion_norm(q)
    return q / n

def quaternion_rotate(q,p):
#     q = quaternion_normalize(q_g)
    t2 =  q[0] * q[1];
    t3 =  q[0] * q[2];
    t4 =  q[0] * q[3];
    t5 = -q[1] * q[1];
    t6 =  q[1] * q[2];
    t7 =  q[1] * q[3];
    t8 = -q[2] * q[2];
    t9 =  q[2] * q[3];
    t1 = -q[3] * q[3];
    x = 2 * ((t8 + t1) * p[0] + (t6 - t4) * p[1] + (t3 + t7) * p[2]) + p[0];
    y = 2 * ((t4 + t6) * p[0] + (t5 + t1) * p[1] + (t9 - t2) * p[2]) + p[1];
    z = 2 * ((t7 - t3) * p[0] + (t2 + t9) * p[1] + (t5 + t8) * p[2]) + p[2];
    return Matrix([x,y,z])

def vector_to_pure_imag_quaternion(p):
    return Matrix([0, p[0], p[1], p[2]])

def vector_to_pure_imag_quaternion_plus_identity(p):
    return Matrix([1, p[0], p[1], p[2]])

def quaternion_mult(z, w):
  r0 = z[0] * w[0] - z[1] * w[1] - z[2] * w[2] - z[3] * w[3];
  r1 = z[0] * w[1] + z[1] * w[0] + z[2] * w[3] - z[3] * w[2];
  r2 = z[0] * w[2] - z[1] * w[3] + z[2] * w[0] + z[3] * w[1];
  r3 = z[0] * w[3] + z[1] * w[2] - z[2] * w[1] + z[3] * w[0];
  return Matrix([r0, r1, r2, r3])

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
    E.write_header(classname="PaulsGeneratedError",path="include/expressions")

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

    Et.write_header(classname="PaulsGeneratedErrorInTangentSpace",path="include/expressions")

#createE()
createEt()
