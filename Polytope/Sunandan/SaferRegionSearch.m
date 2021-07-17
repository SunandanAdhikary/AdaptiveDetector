clc;
clf;
clear all;

system = "trajectory"
options = sdpsettings('cachesolvers',1,...
    'sdpa.maxIteration',300,...
    'verbose',1,'debug',1,'solver',...
    '*','savesolveroutput',1,'savesolverinput',1);
%% systems
if system=="esp"
    Ts= 0.04;
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
    t=3;
end
if system=="trajectory"
    Ts= 0.1;
    A = [1.0000    0.1000; 0    1.0000];
    B = [0.0050; 0.1000];
    C = [1 0];
    D = [0];
%     K = [16.0302    5.6622];
    Q= eye(size(A,2));
    R= eye(size(B,2));
    [K,S,E] = dlqr(A,B,Q,R);
%     L = [0.9902; 0.9892];
    QN = 1500;
    RN = eye(1);
    sys_ss = ss(A,B,C,D,0.1);
    [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
    safex = [25 30];
%     safex = [35;40;35;40];
    safer = 0.6;
    sensor_limit = 30;
    actuator_limit = 36; %36;
    perf_region_depth = 0.3;
    threshold = 4.35;
    nonatk_cov = 12.6;
    t=15;
end

K
L
%% extended state-space
AA= [A zeros(size(A));L*C (A-L*C)] ;
BB= [B;B];
C1 = [C zeros(1,size(A,1))];
C2 = [zeros(1,size(A,1)) C];
KK = [zeros(size(K)) K];
AA_cl = AA-BB*KK;
%% optimization problem setup
% to find a safer fraction of the state safety limits to be used 
% as initial region in order to keep the trajectories safe
% We are forcefully keeping the center of x and xhat same
z = sdpvar(size(AA,1),1);
center = sdpvar(size(AA,1),1);
center1 = sdpvar(size(A,1),1);
center = [center1;center1];
sdpvar scale
x0 = z*scale+ center;
lb_init = center - scale;
ub_init = center + scale;
%% constraints
constraints=[];
lower_safety = -[safex safex]';
upper_safety = [safex safex]';
xi=x0;
ui=KK*xi;
for i=1:t
    constraints = [constraints, (-1)*sensor_limit <= C1*xi <= sensor_limit];
    constraints = [constraints, ...
        (-1)*sqrt(threshold*nonatk_cov) <= C1*xi - C2*xi <= sqrt(threshold*nonatk_cov)];
    constraints = [constraints, (-1)*actuator_limit <= ui <= actuator_limit];
    constraints = [constraints, lower_safety <= xi <= upper_safety];
    
    if i<t
        xi= AA_cl*xi;
        ui=KK*xi;
    end
end
constraints = [constraints, uncertain(z), -1 <= z <=1];
[A1,b1] = double(polytope(constraints))

%% solve
sol = optimize(constraints, -scale ,options)
%% check constraints
chk= check(constraints);
mdl= export(constraints, [] ,sdpsettings('solver','gurobi'));

%% plotting
safer_reg = [value(lb_init),value(ub_init)]
value(scale)
value(center)
time = t+40;
plot_x1 = zeros(1,time+1);
plot_x2 = zeros(1,time+1);
plot_xhat1 = zeros(1,time+1);
plot_xhat2 = zeros(1,time+1);
plot_u = zeros(1,time+1);
safex_short = [value(lb_init) value(ub_init)];
disp('initial value of x');
x_init = [[1 0;0 1]*safer_reg(1:2,1);...
    [1 0;0 1]*(safer_reg(3:4,1)-[sqrt(nonatk_cov*threshold);...
    sqrt(nonatk_cov*threshold)])]
plot_x1(1) = x_init(1);
plot_x2(1) = x_init(2);
plot_xhat1(1) = x_init(3);
plot_xhat2(1) = x_init(4);
x = x_init;
for i=1:time
    u = KK*x;
    x = AA_cl*x;    
    plot_x1(i+1) = x(1);
    plot_x2(i+1) = x(2);
    plot_xhat1(i+1) = x(3);
    plot_xhat2(i+1) = x(4);
    plot_u(i) = u;
end

subplot(1,2,1)
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