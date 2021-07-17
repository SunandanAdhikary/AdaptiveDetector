clc;
clf;
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
    actuator_limit = 0.8125;
    perf_region_depth = 0.1;
    threshold = 4.35;
    nonatk_cov = 0.1169;
    % for different scale of x and xhat
    safer_center = [0 -0.6423]';
    safer_scale_x = 1;
    safer_scale_xhat = 1;
    t=5;
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
    actuator_limit = 50;%36;%50 with t=8 gain synthesized with unsafe control
    perf_region_depth = 0.3;
    threshold = 4.35;
    nonatk_cov = 12.6;
    % for different sclae of x and xhat
    safer_center = [0 0]';
    safer_scale_x = 19.4405;
    safer_scale_xhat = 21.4091;
    t = 8;
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
sdpvar scale;
sdpvar scalehat;
K1 = sdpvar(repmat(size(K,1),t+1),repmat(size(K,2),t+1));
K1{end}=zeros(size(K));

x0 = z*scale + safer_center;
xhat0=z*scalehat + safer_center;

xi=x0;
xhati = xhat0;
ui=-(K + K1{1})*xhati;

P_lyap = sdpvar(size(AA_cl,1),size(AA_cl,2));
constraints = [constraints, AA_cl'*P_lyap*AA_cl-P_lyap <= ...
        -0.001*eye(size(P_lyap)), -P_lyap <= -0.001*eye(size(P_lyap))];

for i=1:t    
    constraints = [constraints, (-1)*sensor_limit <= C*xi <= sensor_limit];
    constraints = [constraints, (-1)*sensor_limit <= C*xhati <= sensor_limit];
    constraints = [constraints, ...
        (-1)*sqrt(threshold*nonatk_cov) <= C*xi - C*xhati <= sqrt(threshold*nonatk_cov)];
    constraints = [constraints, (-1)*actuator_limit <= ui <= actuator_limit];
    constraints = [constraints, lower_safety <= xi <= upper_safety];
    constraints = [constraints, lower_safety <= xhati <= upper_safety];
    
    KK = -[zeros(size(K)) K+K1{1}];
    AA_cl= AA+BB*KK;
    constraints = [constraints, AA_cl'*P_lyap*AA_cl-P_lyap <= ...
        -0.001*eye(size(P_lyap)), -P_lyap <= -0.001*eye(size(P_lyap))];

    xi= A*xi+ B*ui;
    xhati= L*C*xi+ (A-L*C)*xhati + B*ui;
    
    if i<t
        ui=-(K+K1{i+1})*xhati;
    end
end 

constraints = [constraints, lb_init <= xi <= ub_init, ...
    lb_init_hat <= xhati <= ub_init_hat, ...
    scale >= safer_scale_x + 0.1, scalehat >= safer_scale_xhat + 0.1];

optimize([constraints, uncertain(z), -1 <= z <=1],...
    -scale,sdpsettings('debug',1,'bmibnb.maxiter',1000,...
    'bmibnb.maxtime',10000))
check(constraints)

value(scale)
value(scalehat)
value(P_lyap)

for i=1:t
    K2{i} = value(K1{i});
end

% plotting
time = t+40;
plot_x1 = zeros(1,time+1);
plot_x2 = zeros(1,time+1);
plot_xhat1 = zeros(1,time+1);
plot_xhat2 = zeros(1,time+1);
plot_u = zeros(1,time+1);

disp('initial value of x');
lb_init_new = [-value(scale) -value(scale)]'
ub_init_new = [value(scale) value(scale)]'
lb_init_hat_new = [-value(scalehat) -value(scalehat)]'
ub_init_hat_new = [value(scalehat) value(scalehat)]'

init_reg = [value(lb_init_new), value(ub_init_new)]
safer_xhat_reg = [value(lb_init_hat_new),value(ub_init_hat_new)]
x_init = [init_reg(1,1); init_reg(2,1)]% change columns 
% xhat_init = [safer_xhat_reg(1,1);safer_xhat_reg(2, 1) ]% change columns
xhat_init = x_init+[sqrt(nonatk_cov*threshold);...
    sqrt(nonatk_cov*threshold)]

plot_x1(1) = x_init(1);
plot_x2(1) = x_init(2);
plot_xhat1(1) = xhat_init(1);
plot_xhat2(1) = xhat_init(2);

x = x_init;
xhat = xhat_init;
for i=1:time
    if i<=t
    u = -(K + K2{i})*xhat;
    else
        u = -K*xhat;
    end
    u= max(-actuator_limit,min(actuator_limit,u));
    x = A*x + B*u;    
    xhat = L*C*x+(A-L*C)*xhat+B*u;
    plot_x1(i+1) = x(1);
    plot_x2(i+1) = x(2);
    plot_xhat1(i+1) = xhat(1);
    plot_xhat2(i+1) = xhat(2);
    plot_u(i) = u;
end

subplot(1,2,1);
hold on;
plot(plot_x1);
plot(plot_x2);
plot(plot_xhat1);
plot(plot_xhat2);
plot((-safex'*ones(1,t))');
plot((safex'*ones(1,t))');
legend({'x1','x2','xhat1','xhat2','safexlow','safexup'});
hold off

subplot(1,2,2);
hold on;
plot(plot_u);
plot((-actuator_limit*ones(1,t))');
plot((actuator_limit*ones(1,t))');
hold off;