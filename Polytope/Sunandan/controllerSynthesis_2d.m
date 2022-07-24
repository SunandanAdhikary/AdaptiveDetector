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
    safex = [2 2];
    safer = 1;
    sensor_limit = 2.5;
    actuator_limit = 0.8125*2;
    perf_region_depth = 0.1;
    threshold = 4.35;
    nonatk_cov = 0.1169;
    % for different scale of x and xhat
    safer_center = [0 -0.6423]';
    safer_scale_x = 1;
    safer_scale_xhat = 1;
    t=20;
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
    safex = [30 30];
    safer = 0.6;
    sensor_limit = 30;
    actuator_limit = 72; %36;
    perf_region_depth = 0.3;
    threshold = 4.35;
    nonatk_cov = 12.6;
    % for different sclae of x and xhat
    safer_center = [0 0]';
    safer_scale_x = 19.2712;
    safer_scale_xhat = 19.5873;
    t=18;
end

%% extended state-space
AA= [A zeros(size(A));L*C (A-L*C)] ;
BB= [B;B];
C1 = [C zeros(1,size(A,1))];
C2 = [zeros(1,size(A,1)) C];
KK = [zeros(size(K)) K];
AA_cl = AA-BB*KK;

lower_safety = -safex';
upper_safety = safex';

lb_init = safer_center - safer_scale_x;
ub_init = safer_center + safer_scale_x;
lb_init_hat = safer_center - safer_scale_xhat;
ub_init_hat = safer_center + safer_scale_xhat;

%% Optimization Problem
constraints = [];

z = sdpvar(size(A,1),1);
% center = sdpvar(size(A,1),1);
center = safer_center;
sdpvar scale;
sdpvar scalehat;
u1 = sdpvar(1,t);
au = sdpvar(1,t);
ay = sdpvar(1,t);

x0 = z*scale + center;
xhat0=z*scalehat + center;

xi=x0;
xhati = xhat0;
ui=-K*xhati + u1(1);

for i=1:t    
    constraints = [constraints, (-1)*sensor_limit <= C*xi + ay(1) <= sensor_limit];
    constraints = [constraints, (-1)*sensor_limit <= C*xhati <= sensor_limit];
    constraints = [constraints, ...
        (-1)*sqrt(threshold*nonatk_cov) <= C*xi + ay(1) - C*xhati <= sqrt(threshold*nonatk_cov)];
    constraints = [constraints, (-1)*actuator_limit <= ui + + au(1) <= actuator_limit];
    constraints = [constraints, lower_safety <= xi <= upper_safety];
    constraints = [constraints, lower_safety <= xhati <= upper_safety];

    xi= A*xi+ B*(ui + au(i));
    xhati= L*C*xi+ (A-L*C)*xhati + B*ui;
    
    if i<t
        ui=-K*xhati + u1(i+1);
    end
end 

constraints = [constraints, lb_init <= xi <= ub_init, ...
    lb_init_hat <= xhati <= ub_init_hat, ...
    scale >= safer_scale_x + 0.001, scalehat >= safer_scale_xhat + 0.001];
optimize([constraints, uncertain(z), -1 <= z <=1],-scale)

value(scale)
value(scalehat)
% value(center)
% for i=1:t
%     value(u1(i))
% end

