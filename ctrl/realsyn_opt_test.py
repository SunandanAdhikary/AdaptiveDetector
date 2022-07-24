import cvxopt as cvx
import matplotlib as mpl
import picos as pic
from control import *
import numpy as np

# suspension control from "Multi-Objective Co-Optimization of FlexRay-based Distributed Control Systems"
# states: car position, car velocity, suspension load position, suspension velocity
# input suspension load force
# output car position
# settling time = 10 #6
# ontime ~ 20
Ts = 0.04
A = np.matrix('''0.995860265603438   0.037869610530141   0.002126692778129   0.001604928255535;
-0.174562623798343   0.908578953704267   0.046168363355559   0.057309439566604;
0.031113870808023   0.016049282555348   0.935759216765522   0.015154441376985;
1.083961049716042   0.573094395666044  -2.296316359874869   0.090689864367794''')
B = np.matrix('0.090476772045251;   4.869622344172837;   1.263769287906919;  16.972101281961859')
C = np.matrix('1, 0, 0, 0')
D = np.matrix('0')
Q = np.zeros(np.shape(A))
Q[0,0]= 1000
Q[1,1]=10
Q[2,2]=1
Q[3,3]=10
R = np.matrix('0.00001')#0.1
# [Gain,S,E] = ctrl.lqr(A,B,Q,R)
Gain = np.matrix('0.432009800770720   0.081099550600499  -0.005165537236106    0.007976809540882')
#('7.6110    0.3412    0.0186    0.0157')
proc_dev= 0.01
meas_dev= 0.001
# QN = 50*proc_dev**2*(B*np.transpose(B))#90*proc_dev**2*(B*np.transpose(B))
# RN = 0.00005#1*meas_dev**2
# G = np.matrix(np.eye(np.shape(A)))
# L = ctrl.lqe(A,G,QN,RN)
L = np.matrix('''1.644025197924256;  19.067087904601571;   5.655508944611840;   2.067758126353495''')
# ('''0.1298; 0.1642;0.1312;-0.0622''')
FF = np.matrix('-0.455825774196075')#('-7.6533')
ref = np.matrix('0')
safex = np.matrix('-20,-200,-100,-600;20,200,100,600')
#('-15,0,-20,0;15,5,20,10') 
tolerance = [1,1]
threshold = 15.625984375 # FAR = 0.34
bias = 1.305990516912957
sensorRange = np.matrix('-20;20')
actuatorRange = np.matrix('-1000;1000')
innerCircleDepth = 0.5
mean = 0
mean_hyst = 0.01
var = 10
var_hyst = var/10
        
# opt prob
xdim = len(A)
udim = len(B)
ydim = len(L)
new_A = A-B*Gain
min_eig = max(abs(np.linalg.eigvals(new_A)))
E = np.sqrt(min_eig)*np.eye(xdim)#In general a function of A and B
# try:
#     P = dlyap(np.transpose(new_A),Q,None,E)
# except ValueError:
#     print("Opps! Error finding the minimum spectral value")
P =np.matrix('''4689.79194696471	-17635.7763515500	-4302.07675158788	-6613.99409340525;
-17635.7763515500	105318.133836658	26828.4724865502	-5221.47163608261;
-4302.07675158788	26828.4724865502	7335.53592238014	-4499.87647154114;
-6613.99409340525	-5221.47163608261	-4499.87647154114	88212.3253119795''')
Aprob = cvx.matrix(P)
cprob = cvx.matrix([1])

#create the problem, variables and params
prob=pic.Problem()
AA=cvx.matrix(Aprob,tc='d') #each AA[i].T is a 3 x 5 observation matrix
print(AA)
AA=pic.new_param('Aprob',AA)
print(AA)
cc=pic.new_param('cprob',cprob)
for d in range(xdim):
    d_vec = []
    for temp in range(0,d):
            d_vec.append(0)
    d_vec.append(1)
    for temp in range(d,xdim-1):
            d_vec.append(0)
# print(d_vec)
ss = pic.new_param('s',cvx.matrix(d_vec))
# print(ss)
x = prob.add_variable('x',AA.size[1])

#define the constraints and objective function
prob.add_list_of_constraints(
        [x.T*AA*x < cc], #constraints
        )

prob.set_objective('max', ss|x)

#solve the problem and retrieve the optimal weights of the optimal design.
# print prob
prob.solve(verbose=0,solver='cvxopt')
x=x.value

print(x)