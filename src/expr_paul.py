#!/usr/bin/python
from sympy.matrices import *
from sympy import sin,cos
from ErrorTerm import *

def quaternion_inv(q):
    return Matrix([q[0],-q[1],-q[2],-q[3]])

def quaternion_norm(q):
    return sqrt(q.transpose() * q)

def quaternion_normalize(q):
    n = quaternion_norm(q)
    return q / n

def quaternion_mul(q,p):
    q = quaternion_normalize(q)
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

E = ErrorTerm(namespace="error_term")

U = E.declareData("U",2);

q_1_2 = E.declareVariable('q_1_2',4);
q_2_3 = E.declareVariable('q_2_3',4);
P_3 = E.declareVariable('P_3',3);

q_1_2_inv = quaternion_inv(q_1_2)
p_2 = quaternion_mul(q_2_3,p_3)
p_1 = quaternion_mul(q_1_2_inv,p_2)

Up = Matrix([p_1[0]/p_1[2],p_1[1]/p_1[2]])
F = U - Up
E.setFunction(F)

print F
print E.J

E.write_header(classname="ProjectionError",path="include/expressions")

