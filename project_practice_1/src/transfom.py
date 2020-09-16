#!/usr/bin/env python
import math
from sympy import *

theta=math.radians(45)
x=Symbol('x')
eq1=pow(4.32-2.92*cos(x)-1.4*sin(theta),2)+pow(1.4*sin(theta)-2.9*sin(x)+3.16,2)-pow(3.16,2)
soln1=solve(eq1,x)
y=Symbol('y')
eq2=pow(4.32-2.92*cos(y)-1.4*sin(theta),2)+pow(-1.4*sin(theta)+2.9*sin(y)+3.16,2)-pow(3.16,2)
soln2=solve(eq2,y)
print(soln1,soln2)

print(math.degrees(soln1[0]))
print(math.degrees(soln2[1]))