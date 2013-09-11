#!/usr/bin/python
from sympy.matrices import *
from sympy import sin,cos
from ErrorTerm import *

def RotZ(alpha):
    return Matrix([[cos(alpha),-sin(alpha), 0],
        [sin(alpha),cos(alpha),0],[0,0,1]])


E = ErrorTerm(namespace="error_term")

U = E.declareData("U",2);

alpha = E.declareVariable('alpha');
f = E.declareVariable('f');
X = E.declareVariable('x',3);
t = E.declareVariable('t',3);

R = RotZ(alpha);
T = eye(4)
T[0:3,0:3] = R
T[0:3,3] = t

K = Matrix([[f,0,0,0],[0,f,0,0],[0,0,1,0]])

Xh = zeros(4,1);
Xh[0:3,0] = X

Xp = K * T * Xh
Up = Matrix([Xp[0]/Xp[2],Xp[1]/Xp[2]])

F = U - Up
E.setFunction(F)

print F
print E.J

E.write_header(classname="ProjectionError",path="include/expressions")

