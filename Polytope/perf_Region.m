clc;
clear all;

system = "esp"

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
    actuator_limit = 0.8125;
    perf_region_depth = 0.1;
    threshold = 4.35;
    nonatk_cov = 0.1169;
    safer_center = [-0.7258    1.7258   -0.7250    1.5611]';
    safer_scale = 0.2742;
    t=8;
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
    actuator_limit = 36; %36;
    perf_region_depth = 0.3;
    threshold = 4.35;
    nonatk_cov = 12.6;
    safer_center = [18.1468   -9.5400   17.3949   -12.6147]';
    safer_scale = 3.3258;
    t=20;
end

%% extended state-space
AA= [A zeros(size(A));L*C (A-L*C)] ;
BB= [B;B];
C1 = [C zeros(1,size(A,1))];
C2 = [zeros(1,size(A,1)) C];
KK = [zeros(size(K)) K];
AA_cl = AA-BB*KK;

lower_safety = safer_center - safer_scale;
upper_safety = safer_center + safer_scale;


%% Optimization Problem
z = sdpvar(4,1);
center = sdpvar(4,1);

sdpvar scale;
x0 = scale*z + center;
constraints = [];
xi=x0;
ui=KK*xi;
for i=1:t
    constraints = [constraints, (-1)*sensor_limit <= C1*xi <= sensor_limit];
    constraints = [constraints, (-1)*sqrt(threshold*nonatk_cov) ...
                    <= C1*xi - C2*xi <= sqrt(threshold*nonatk_cov)];
    constraints = [constraints, (-1)*actuator_limit <= ui <= actuator_limit];
%     constraints = [constraints, center - scale*[1 1 1 1]' <= xi <= center + scale*[1 1 1 1]'];?
    constraints = [constraints, norm(xi-center,Inf)<=scale];
    xi = AA_cl*xi;
    ui = KK*xi;
end

optimize([constraints, uncertain(z), -1 <= z <=1],-scale)
value(scale)
value(center)

