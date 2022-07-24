clc;
clear all;

system = "trajectory"

%% systems
if system=="esp"
    A = [0.4450 -0.0458;1.2939 0.4402];
    B = [0.0550;4.5607];
    C = [0 1];
    K = [0.2826    0.0960];
    L = [-0.0390;0.4339];
    safex = [1 2];
    safer = 1;
    sensor_limit = 2.5;
    actuator_limit = 0.8125*2;
    perf_region_depth = 0.1;
    threshold = 4.35;
    nonatk_cov = 0.1169;
    safer_center = [-0.7258    1.7258   -0.7250    1.5611]';
    safer_scale = 0.2742;
    t=10;
end
if system=="trajectory"
    A = [1.0000    0.1000; 0    1.0000];
    B = [0.0050; 0.1000];
    C = [1 0];
    D = [0];
    Q= eye(size(A,2));
    R= eye(size(B,2));
    [K,S,E] = dlqr(A,B,Q,R);
    QN = 1500;
    RN = eye(1);
    sys_ss = ss(A,B,C,D,0.1);
    [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
    safex = [25 30];
    safer = 0.6;
    sensor_limit = 30;
    actuator_limit = 72; %36;
    perf_region_depth = 0.3;
    threshold = 4.35;
    nonatk_cov = 12.6;
    safer_center = [18.1468   -9.5400   17.3949   -12.6147]';
    safer_scale = 3.3258;
    t=50;
end

%% extended state-space
AA= [A zeros(size(A));L*C (A-L*C)] ;
BB= [B;B];
BB_a = [B zeros(size(C,2),size(C,1));zeros(size(A,1),size(B,2)) L];
C1 = [C zeros(1,size(A,1))];
C2 = [zeros(1,size(A,1)) C];
KK = [zeros(size(K)) K];
AA_cl = AA-BB*KK;

lower_safety = -[safex safex]';
upper_safety = [safex safex]';

lower_goal = safer_center - safer_scale;
upper_goal = safer_center + safer_scale;
max_radius = min([safex';safex'] - safer_center);

%% Optimization Problem
constraints = [];

sdpvar scale;
z = sdpvar(4,1);
u1 = sdpvar(1,t);
au = sdpvar(1,t);
ay = sdpvar(1,t);

x0 = scale*z + safer_center;
xi=x0;
% a{1} = [au(1);ay(1)];
ui= -KK*xi + u1(1) + au(1);

for i=1:t
    constraints = [constraints, (-1)*sensor_limit <= C1*xi + ay(1) <= sensor_limit];
    constraints = [constraints, ...
        (-1)*sqrt(threshold*nonatk_cov) <= C1*xi + ay(i) - C2*xi <= sqrt(threshold*nonatk_cov)];
    constraints = [constraints, (-1)*actuator_limit <= ui <= actuator_limit];
    constraints = [constraints, lower_safety <= xi <= upper_safety];

    xi = AA_cl*xi + BB_a*[au(i);ay(i)];
    if i<t
        ui= -KK*xi + u1(i+1) + au(i+1);
    end
end 

constraints = [constraints, lower_goal <= xi <= upper_goal];
optimize([constraints, uncertain(z), -1 <= z <=1],-scale)

value(scale)
% for i=1:t
%     value(u1(i))
% end

