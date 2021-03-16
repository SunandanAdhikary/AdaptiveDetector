from z3 import *
import math
import numpy as np

s = Solver()
set_option(rational_to_decimal=True)
xabs1 = np.zeros(2, dtype=float)
zabs1 = np.zeros(2, dtype=float)
x1 = np.zeros(2, dtype=float)
z1 = np.zeros(2, dtype=float)
xabs2 = np.zeros(2, dtype=float)
zabs2 = np.zeros(2, dtype=float)
x2 = np.zeros(2, dtype=float)
z2 = np.zeros(2, dtype=float)
y1 = np.zeros(2, dtype=float)
attackOnY1 = np.zeros(2, dtype=float)
r1 = np.zeros(2, dtype=float)
u1 = np.zeros(2, dtype=float)
uattacked1 = np.zeros(2, dtype=float)
attackOnU1 = np.zeros(2, dtype=float)
r = np.zeros(2, dtype=float)

RGain={}
RGain[1] = np.zeros((1,2), dtype=float)
RGain[2] = np.zeros((1,2), dtype=float)
#declarations
y1_0 = Real('y1_0')
r1_0 = Real('r1_0')
rabs1_0 = Real('rabs1_0')
x1_0 = Real('x1_0')
z1_0 = Real('z1_0')
xabs1_0 = Real('xabs1_0')
RGain11_0 = Real('RGain11_0')
zabs1_0 = Real('zabs1_0')
x2_0 = Real('x2_0')
z2_0 = Real('z2_0')
xabs2_0 = Real('xabs2_0')
RGain12_0 = Real('RGain12_0')
zabs2_0 = Real('zabs2_0')
u1_0 = Real('u1_0')
uattacked1_0 = Real('uattacked1_0')
r_0 = Real('r_0')
#declarations
y1_1 = Real('y1_1')
r1_1 = Real('r1_1')
rabs1_1 = Real('rabs1_1')
x1_1 = Real('x1_1')
z1_1 = Real('z1_1')
xabs1_1 = Real('xabs1_1')
RGain11_1 = Real('RGain11_1')
zabs1_1 = Real('zabs1_1')
x2_1 = Real('x2_1')
z2_1 = Real('z2_1')
xabs2_1 = Real('xabs2_1')
RGain12_1 = Real('RGain12_1')
zabs2_1 = Real('zabs2_1')
u1_1 = Real('u1_1')
uattacked1_1 = Real('uattacked1_1')
r_1 = Real('r_1')

#Init

attackOnU1_0 = Real('attackOnU1_0')
attackOnY1_0 = Real('attackOnY1_0')
s.add(ForAll([x1_0,z1_0,x2_0,z2_0,attackOnU1_0,attackOnY1_0],
Implies(
And(
Or(x1_0 == 17.5,x1_0 == -17.5),
And(z1_0 <= 17.5,z1_0 >= -17.5),
Or(x2_0 == 21.0,x2_0 == -21.0),
And(z2_0 <= 21.0,z2_0 >= -21.0),
attackOnU1_0==0.00,
((((0.0*attackOnY1_0) + (1*x1_0) + (0*x2_0) + (0*((- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))+ (0.0*attackOnU1_0)))) - (1*z1_0) - (0*z2_0) - (0*(- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))))*(((0.0*attackOnY1_0) + (1*x1_0) + (0*x2_0) + (0*((- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))+ (0.0*attackOnU1_0)))) - (1*z1_0) - (0*z2_0) - (0*(- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))))<=12.6041*4.35),
attackOnY1_0== 0.00,
),
And(
(If(((1.0*x1_0) + (0.1*x2_0) + (0.005*((- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))+ (0.0*attackOnU1_0))))<0,(-1)*((1.0*x1_0) + (0.1*x2_0) + (0.005*((- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))+ (0.0*attackOnU1_0)))),((1.0*x1_0) + (0.1*x2_0) + (0.005*((- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))+ (0.0*attackOnU1_0)))))) < 7.500000000000001,
(If(((1.0*z1_0) + (0.1*z2_0) + (0.005*(- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))) + (0.9902*(((0.0*attackOnY1_0) + (1*x1_0) + (0*x2_0) + (0*((- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))+ (0.0*attackOnU1_0)))) - (1*z1_0) - (0*z2_0) - (0*(- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))))))<0,(-1)*((1.0*z1_0) + (0.1*z2_0) + (0.005*(- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))) + (0.9902*(((0.0*attackOnY1_0) + (1*x1_0) + (0*x2_0) + (0*((- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))+ (0.0*attackOnU1_0)))) - (1*z1_0) - (0*z2_0) - (0*(- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0)))))),((1.0*z1_0) + (0.1*z2_0) + (0.005*(- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))) + (0.9902*(((0.0*attackOnY1_0) + (1*x1_0) + (0*x2_0) + (0*((- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))+ (0.0*attackOnU1_0)))) - (1*z1_0) - (0*z2_0) - (0*(- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0)))))))) < 7.500000000000001,
(If(((0.0*x1_0) + (1.0*x2_0) + (0.1*((- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))+ (0.0*attackOnU1_0))))<0,(-1)*((0.0*x1_0) + (1.0*x2_0) + (0.1*((- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))+ (0.0*attackOnU1_0)))),((0.0*x1_0) + (1.0*x2_0) + (0.1*((- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))+ (0.0*attackOnU1_0)))))) < 9.000000000000002,
(If(((0.0*z1_0) + (1.0*z2_0) + (0.1*(- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))) + (0.9892*(((0.0*attackOnY1_0) + (1*x1_0) + (0*x2_0) + (0*((- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))+ (0.0*attackOnU1_0)))) - (1*z1_0) - (0*z2_0) - (0*(- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))))))<0,(-1)*((0.0*z1_0) + (1.0*z2_0) + (0.1*(- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))) + (0.9892*(((0.0*attackOnY1_0) + (1*x1_0) + (0*x2_0) + (0*((- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))+ (0.0*attackOnU1_0)))) - (1*z1_0) - (0*z2_0) - (0*(- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0)))))),((0.0*z1_0) + (1.0*z2_0) + (0.1*(- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))) + (0.9892*(((0.0*attackOnY1_0) + (1*x1_0) + (0*x2_0) + (0*((- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0))+ (0.0*attackOnU1_0)))) - (1*z1_0) - (0*z2_0) - (0*(- ((16.0302+RGain11_0)*z1_0) - ((5.6622+RGain12_0)*z2_0)))))))) < 9.000000000000002
)  
)
))
fsmt = open("results/robust_Ctrl/trajectory/trajectory_1.smt2", "w+")
fsmt.write(s.sexpr())
fsmt.write("(check-smt)")
fsmt.close()

if s.check() != sat:
	print(s.check())
	isSat = 0
else:
	print(s.check())
	isSat = 1
	f0 = open("results/robust_Ctrl/trajectory/trajectory_robustC.z3result", "w+")
	f0.write("1")
	f0.close()
	m = s.model()
	for d in m.decls():
		print ("%s = %s" % (d.name(), m[d]))
if isSat==1:
	f0 = open("results/robust_Ctrl/trajectory/trajectory_robustC.z3result", "w+")
	f0.write("1")
	f0.close()
