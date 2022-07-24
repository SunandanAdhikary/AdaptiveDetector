% X_R = 0.2*X_S
clear;
clc;
% clf;
% {esp_bicycle,dcmotor,quadrotor,fuel_injection,driveline_mngmnt,esp,ttc}
system = "aircraft"

%% driveline_mngmnt(Modeling and Control of an Engine Fuel Injection System)
if system=="driveline_mngmnt" % linearized clutch+driveline
Ts = 0.02;
A = [0,0,1,-i_t,0;
    0,0,0,1/i_f;-1;
    -k_c/J1,0,-cc/J1,cc*i_t/J1,0;
    k_c*i_t/J2,-k_d/(i_f*J2),cc*i_t/J2,-(cc*i_t*i_t+b2+cd/(i_f*i_f))/J2,cd/(i_f*J2);
    0,kd/J3,0,cd/(i_f*J3),-(b3+cd)/J3];
B = [0.00516,-0.0172;
    -0.00073,0.09841;
    -0.00011,0.13589];
C = [0,1,0,0,0,1];
D = [0,0];

Q= 0.1*eye(size(A,2));
R= 500*eye(size(B,2));
[K,S,E] = dlqr(A,B,Q,R);
QN = 1500;
RN = eye(1);
sys_ss = ss(A-B*K,B,C,D,Ts);
[kalmf,L,P,M] = kalman(sys_ss,QN,RN);
safex = [4,100,100];
% safer region of this system to start from
ini = 1;
% from perfReg.py with this system
perf = [-1.67,-1.47];
% for central chi2 FAR < 0.05
th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
sensorRange = [4];  % columnwise range of each y
actuatorRange = [36];   % columnwise range of each u
proc_var= 1; meas_var=0.01;
% s.settlingTime = 13; 
% from system_with_noise.m with this system
s.noisy_zvar= 0.14;
% from system_with_noise.m with this system
s.noisy_zmean= 0.52;
s.noisy_delta= 1.86;
s.nonatk_zvar= 12.6041;%15.8507
s.nonatk_zmean= 0.6064;
s.nonatk_delta= 0.0292;
s.uatkon=[1];   % attack on which u
s.yatkon=[1];   % attack on which y
end

%% fuel_injection(Modeling and Control of an Engine Fuel Injection System)
if system=="fuel_injection"
Ts = 0.01;
A = [0.18734,0.13306,0.10468;
    0.08183,0.78614,-0.54529;
    -0.00054 0.10877,0.26882];
B = [0.00516,-0.0172;
    -0.00073,0.09841;
    -0.00011,0.13589];
C = [158.16,8.4277,-0.44246];
D = [0,0];
Q= 0.1*eye(size(A,2));
R= 500*eye(size(B,2));
[K,S,E] = dlqr(A,B,Q,R);
QN = 15000;
RN = 0.1*eye(1);
sys_ss = ss(A-B*K,B,C,D,Ts);
[kalmf,L,P,M] = kalman(sys_ss,QN,RN);
safex = [4,100,100];
% safer region of this system to start from
ini = 1;
% from perfReg.py with this system
perf = [];
% for central chi2 FAR < 0.05
th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
sensorRange = [4];  % columnwise range of each y
actuatorRange = [36];   % columnwise range of each u
proc_var= 1; meas_var=0.01;
% s.settlingTime = 13; 
% from system_with_noise.m with this system
noisy_zvar= 0.14;
% from system_with_noise.m with this system
noisy_zmean= 0.52;
noisy_delta= 1.86;
nonatk_zvar= 12.6041;%15.8507
nonatk_zmean= 0.6064;
nonatk_delta= 0.0292;
uatkon=[1];   % attack on which u
yatkon=[1];   % attack on which y
end

%% quadrotor(Thesis_KTH-Francesco_Sabatino)
if system=="quadrotor"
Ts = 0.02;
A1 = [0,1,0,0;0,0,1,0;0,0,0,1;0,0,0,0];
A2 = [0,1;0,0];
A=blkdiag(A1,A1,A1,A2);
B1=zeros(4,4);
B1(4,1)=1;
B2=zeros(4,4);
B2(4,2)=1;
B3=zeros(4,4);
B1(4,3)=1;
B4=zeros(2,4);
B = [B1;B2;B3;B4];
C1 = [1;0;0;0];
C2 = [1;0];
C = blkdiag(C1',C1',C1',C2');
D = zeros(size(C,1),size(B,2));

Q= eye(size(A,2));
R= 0.01*eye(size(B,2));
[K,S,E] = dlqr(A,B,Q,R);
QN = 1500;
RN = eye(1);
sys_ss = ss(A-B*K,B,C,D,Ts);
[kalmf,L,P,M] = kalman(sys_ss,QN,RN);
safex = [4,100,100];
% from perfReg.py with this system
perf = [-1.67,-1.47];
% safer region of this system to start from
ini = perf;
% for central chi2 FAR < 0.05
th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
sensorRange = [4];  % columnwise range of each y
actuatorRange = [36];   % columnwise range of each u
proc_var= 1; meas_var=0.01;
settlingTime = 13; 
% from system_with_noise.m with this system
noisy_zvar= 0.14;
% from system_with_noise.m with this system
noisy_zmean= 0.52;
noisy_delta= 1.86;
nonatk_zvar= 12.6041;%15.8507
nonatk_zmean= 0.6064;
nonatk_delta= 0.0292;
uatkon=[1];   % attack on which u
yatkon=[1];   % attack on which y
end

%% dcmotor(2020_RTSS_Real-Time Attack-Recovery for Cyber-Physical Systems Using Linear Approximations)
if system=="dcmotor"
    % states: rotational ang., angular vel., armature current
    % output rotational angle
    % input armature voltage
    Ts = 0.1;
    J = 0.01;
    b = 0.1;
    KK = 0.01;
    RR = 1;
    LL = 0.5;
    A = [0 1 0;
        0 -b/J KK/J;
        0 -KK/LL -RR/LL];
    B = [0; 0; 1/LL];
    C = [1 0 0];
    D = [0];
    Q= eye(size(A,2));
    R= 0.01*eye(size(B,2));
    [K,S,E] = dlqr(A,B,Q,R);
    QN = 1500;
    RN = eye(1);
    sys_ss = ss(A-B*K,B,C,D,Ts);
    [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
    safex = [-4,0,4,100,100];
    % from perfReg.py with this system
    perf = [-1.67,-1.67,-1.67;-1.47,-1.47,-1.47];
    % safer region of this system to start from
    ini = perf;
    % for central chi2 FAR < 0.05
    th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    sensorRange = [4];  % columnwise range of each y
    actuatorRange = [36];   % columnwise range of each u
    proc_var= 0.1; meas_var=0.01;
    settlingTime = 13; 
    % from system_with_noise.m with this system
    noisy_zvar= 0.14;
    % from system_with_noise.m with this system
    noisy_zmean= 0.52;
    noisy_delta= 1.86;
    nonatk_zvar= 12.6041;%15.8507
    nonatk_zmean= 0.6064;
    nonatk_delta= 0.0292;
    uatkon=[1];   % attack on which u
    yatkon=[1];   % attack on which y
end
%% aircraft(ctms/Real-Time Attack-Recovery for Cyber-Physical Systems Using Linear Approximations)
if system=="aircraft"
    % states: attack angle, pitch rate, pitch angle
    % output pitch angle
    % input angle of deflection
    A = [-0.313 56.7 0;-0.0139 -0.426 0;0 56.7 0];
    B = [0.232;0.0203;0];
    C = [0 0 1];
    D = zeros(size(C,1),size(B,2));
    Ts = 0.02;    
    Q= 4.5*C'*C;
    R= 0.2;
    [K,S,E] = dlqr(A,B,Q,R);% from ctms
    QN = 1500;
    RN = 10*eye(1);
    sys_ss = ss(A-B*K,B,C,D,Ts);
    [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
    % safery constraints
    safex = [0 0 0; 2 2 2];
    % performance region of this system
    perf = [0.68 0.68 0.68;0.72 0.72 0.72];
    % safer region of this system to start from
    ini = perf;
    % for central chi2 FAR < 0.05
    th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    sensorRange = [0;2];  % columnwise range of each y
    % columnwise range of each u
    actuatorRange = [-0.4363 ;0.4363 ];   % from ctms control tab
    proc_var= 0.001;
    meas_var=0.0001;
    settlingTime = 13; 
    % from system_with_noise.m with this system
    noisy_zvar= 0.14;
    % from system_with_noise.m with this system
    noisy_zmean= 0.52;
    noisy_delta= 1.86;
    nonatk_zvar= 12.6041;%15.8507
    nonatk_zmean= 0.6064;
    nonatk_delta= 0.0292;
    uatkon=[1];   % attack on which u
    yatkon=[1];   % attack on which y 
end
%% esp_bicycle
if system=="esp_bicycle"
Ts=0.04;
A = [0.6278   -0.0259;
    0.4644    0.7071];

B = [0.1246   -0.00000028;
    3.2763    0.000016];

C = [0    1.0000
 -338.7813    1.1293];

D = [0         0;
  169.3907         0];

K = [5.261 -0.023;
    -414911.26, 57009.48];

L = [-0.00000000002708 -0.00000000063612;
    0.00000000033671  0.00000000556308];

safex = [1,2];
ini = 1;
perf = 0.2;
th = 4.35; 
settlingTime = 13 ;
sensorRange = [2.5;15];  % columnwise range of each y
actuatorRange = [0.8125;0.8125]; % columnwise range of each u
noisy_zvar=0.1;
noisy_zmean= 0.5;
proc_var= 0.1; meas_var=0.001;
end

%% esp
if system=="esp"
Ts=0.04;
A = [0.4450 -0.0458;1.2939 0.4402];
B = [0.0550;4.5607];
C = [0 1];
D = [0];
p = 0.00001;
Q = p*(C'*C);
R = 0.000001;
[K,S,E] = dlqr(A,B,Q,R)
sys_ss =ss(A-B*K,B,C,D,Ts);
safex=[1 2];
ini= 0.1;
perf=0.1;
QN = 5000000;
RN = 10*eye(1);
[kalmf,L,P,M] = kalman(sys_ss,QN,RN,Ts);
% K = [-0.0987 0.1420];
L = [-0.0390;0.4339];
proc_var= 0.1; meas_var=0.001;
end

%% ttc
if system=="ttc"
    Ts = 0.1;
    A = [1.0000    0.1000; 0    1.0000];
    B = [0.0050; 0.1000];
    C = [1 0];
    D = [0];
    K = [16.0302    5.6622];  % settling time around 10
    L = [0.9902; 0.9892];
ini=0.1;
perf=0.3;
safex=[25 30]
proc_var= 0.1; meas_var=0.01;
end

%% init 
time=100;
xdim=size(A,1);
udim=size(B,2);
ydim=size(C,1);
rng shuffle;
% x=(2*ini*safex*rand(xdim)-safex*ini)';
for k=1:xdim
   x(k,1) = (2*ini(k)*rand-ini(k))';
%         x(k,1) = -1*(safex(k)*init)';
end  
% proc_var= 1; meas_var=0.01;
% x = [0.0976827903;0.1724794346];
y = C*x;
z = zeros(xdim,1);
u = -K*z;
max_x = abs(x);
max_u = abs(u);
state=zeros(xdim,time+1);
state(:,1)= x;
state_est=zeros(xdim,time+1);
out = zeros(ydim,time+1);
out(:,1)= y;
res = zeros(ydim,time+1);
est_err = zeros(xdim,time+1);

for i=2:time+1
   i
   y = C*x + meas_var*(2.*rand(size(C,1),1)-eye());
   r = y - C*z;
   z = A*z + B*u + L*r
   x = A*x + B*u + proc_var*rand(size(C,1),1)
   u = -K*z;
   
   for j=1:xdim
       if abs(x(j))>max_x(j)
           max_x(j) = abs(x(j));
       end
   end
   for jj=1:udim
       if abs(u(jj))>max_u(jj)
           max_u(jj) = abs(u(jj));
       end
   end   
   
   res(:,i)=r;
   est_err(:,i)=(x-z);
   state(:,i) = x;
   state_est(:,i) = z;
   out(:,i) = y;
   
end


for j=1:xdim
    figure
    hold on;
    plot(state(j,:)');
    plot(state_est(j,:)');
    legend(strcat({'x','z'},{num2str(j),num2str(j)}));
    axis([1 time safex(1,j) safex(2,j)]);
    hold off;
end

max_x
max_u
if exist('res_cov','var')
    res_cov = max(res_cov,cov(res))
else
    res_cov = cov(res)
end
if exist('res_mean','var')
    res_mean = min(res_mean,mean(res))
else
    res_mean = mean(res)
end
chi_sq = mean(res)*inv(cov(res))*mean(res)'
