clc;
clear all;
yalmip("clear");
system = "trajectory"
ops = sdpsettings('verbose',1,'CACHESOLVERS',1);
%% systems
if system=="esp"
    A = [0.4450 -0.0458;1.2939 0.4402];
    B = [0.0550;4.5607];
    C = [0 1];
    K = [0.2826    0.0960];
    L = [-0.0390;0.4339];
    safex = [-1, -2; 1, 2];
    safer = 1;
    sensor_limit = 2.5;
    actuator_limit = 0.8125;
    perf_region_depth = 0.1;
    perf= perf_region_depth.*safex;
    threshold = 4.35;
    nonatk_cov = 0.1169;
    safer_center = [-0.7258    1.7258   -0.7250    1.5611]';
    safer_scale = 0.2742;
    t=8;
    rate = 0.5;
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
    safex = [-25,-30;25,30];
    safer = 0.6;
    sensor_limit = 30;
    actuator_limit = 36; %36;
    perf_region_depth = 0.3;
    perf = perf_region_depth.*safex;
    threshold = 4.35;
    nonatk_cov = 12.6;
    safer_center = [18.1468   -9.5400   17.3949   -12.6147]';
    safer_scale = 3.3258;
    t=20;
    rate = 0.5;
end
% vehicle suspension control (Multi-Objective Co-Optimization of FlexRay-based Distributed Control System)--in CCM ECU
if system=="suspension_control"
%   states: car position, car velocity, suspension load position, suspension velocity
%   input suspension load force
%   output car position
    Ts = 0.04;
    Ac = [0 1 0 0;-8 -4 8 4;0 0 0 1;80 40 -160 -60];
    Bc = [0;80;20;1120];
    Cc = [1,0,0,0];
    Dc = [0];    
    sys_ct= ss(Ac,Bc,Cc,Dc);
    sys_dt= c2d(sys_ct,Ts);
    [A,B,C,D]= ssdata(sys_dt);  
    % safery margin
    safex = [-20,-200,-100,-600;20,200,100,600]%[-15,-15,-20,-20;15,15,20,20];
    % safer region of this system to start from
    ini = 0.5.*safex;
    perf = 0.1.*safex;
    % for central chi2 FAR < 0.05
    threshold = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    sensorRange = [-20;20];  % columnwise range of each y
    actuatorRange = [-1000;1000];   % columnwise range of each u
    qweight= 1000;
    rweight = 0.00001;
    Q= qweight*(C'*C);
    Q(2,2)=10;Q(3,3)=1;Q(4,4)=10;
    R= rweight;
    proc_dev= 0.01; meas_dev=0.001;
    QN = 0.005*(B*B');
    RN = 0.00005;
%     % using lqg ragulator
%     K=[7.6110    0.3412    0.0186    0.0157];
%     L= [0.1298; 0.1642; 0.1312; -0.0622];
    settlingTime = 0.2800/Ts;
%     s.noisy_zvar= 2.6328;
%     s.noisy_zmean= 0.2719;
%     s.noisy_delta= 1.86;
%     s.nonatk_zvar= 12.6041;%15.8507
%     s.nonatk_zmean= 0.6064;
%     s.nonatk_delta= 0.0292;
    s.uatkon=[1];   % attack on which u
    s.yatkon=[1];   % attack on which y
end
% fuel_injection(Modeling and Control of an Engine Fuel Injection System)--in ECM ECU
if system=="fuel_injection"
    % states: 
    % inputs: inlet valve air flow, combustion torque
    % output: AFR
    Ts = 0.01;
    A = [0.18734,0.13306,0.10468;
        0.08183,0.78614,-0.54529;
        -0.00054 0.10877,0.26882];
    B = [0.00516,-0.0172;
        -0.00073,0.09841;
        -0.00011,0.13589];
    C = [158.16,8.4277,-0.44246];
    D = [0,0];
    Q= blkdiag(0.1,0.1,1);%blkdiag(1,2.25,25);
    R= blkdiag(1,500);
    Pr= care(A,B,Q);
    K = -inv(R)*B'*Pr;
    qweight= 1;
    rweight = 1;
    proc_dev= 0.001; meas_dev=0.0001;
    QN = eye(size(B,1));%proc_dev^2*(B*B');
    RN = 1;%meas_dev^2;
    L=[0.00150311177912917;
        0.00200806159683866;
        0.000355570756765841];
    safex = [-0.22,-1.5,-5;0.22,1.5,5].*3;
    % initial region of this system to safely start from
    ini = 0.7.*safex;
    perf = 0.1.*safex;
    % for central chi2 FAR < 0.05
    % th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    sensor_limit = 80;  % columnwise range of each y
    actuator_limit = [20;20];   % columnwise range of each u
    % ref= 14.7;
    % F= [1; -1.5];
    settling_time = 6; 
    % noisy_zvar= 0.14;
    % noisy_zmean= 0.52;
    % noisy_delta= 1.86;
    % nonatk_zvar= 12.6041;%15.8507
    % nonatk_zmean= 0.6064;
    % nonatk_delta= 0.0292;
    threshold =4.35;
    uatkon=[1];   % attack on which u
    yatkon=[1];   % attack on which y
    rate =0.5;
end
%% to measure number of iterations required to take the system from safety
%% to perormance using the normal lqr controller
lqr_iter =0 ;
%% starting from negative safety border and ending at negative performance border
x0 = safex(1,:)';
xhat0 = perf(1,:)';
x = x0;
xhat = xhat0;
u = -K*x0;
r = C*(x-xhat);
k = 0;
j =0;
xi = [x0];
xhati =[xhat0];
while x(1) < perf(1,1) || x(2) < perf(1,2)
    x = A*x+B*u;
    xhat = A*xhat +B*u+ L*r;
    r = C*(x-xhat);
    u = -K*xhat;
    k=k+1;
    if x ~= xhat
        j =j+1;
    end
    xi =[xi,x];
    xhati =[xhati,xhat];
end
xi
xhati
figure
plot(xi')
figure
plot(xhati')
lqr_iter = k
est_iter = j
%% starting from positive safety border and ending at positive performance border
x0 = safex(2,:)';
x = x0;
xhat = x0;
u = -K*x0;
r = C*(x-xhat);
k = 0;
j=0;
xi = [x0];
xhati =[xhat0];
while x(1) < perf(2,1) || x(2) < perf(2,2)
    x = A*x+B*u;
    xhat = A*xhat +B*u+ L*r;
    r = C*(x-xhat);
    u = -K*xhat;
    k=k+1;
    if x ~= xhat
        j =j+1;
    end
    xi =[xi,x];
    xhati =[xhati,xhat];
end

plot(xi');
plot(xhati');

if lqr_iter < k
    lqr_iter = k
end
if est_iter < j
    est_iter = j
end
%% gain synthesis
%% init
solved = 1;
solved1 = 1;
factor = 0.7;
slack = 0.0001;
% init
x0 = factor*safex(1,:).';
xhat0 = sdpvar(size(A,2),1);
% xhat0 = [x0(1)+threshold-slack;0];
constraints = [safex(1,:).' <= xhat0, xhat0 <=  safex(2,:).',uncertain(xhat0)];  
n = lqr_iter;
u = sdpvar(size(B,2),n);
x = sdpvar(size(A,2),n+1);
xhat = sdpvar(size(A,2),n+1);
e = sdpvar(size(A,2),n+1);
y = sdpvar(size(C,1),n+1);
r = sdpvar(size(C,1),n+1);
K_new = {};
i = 1;
% Knew =[0.0204    0.0015;
%     2.9356   -0.3267;
%    -2.7940    2.6806;
%    -5.3687    5.2348;
%    -3.6874    4.6782;
%    -4.3512   -9.3454;
%     0.3628   15.6785];
while solved1
    K_new{1} = sdpvar(size(B,2),size(A,2));
    x(:,1) = x0;
    xhat(:,1) = xhat0;
    u(:,1) = -K_new{1}*xhat0;
    y(:,1) = C*x(:,1);
    r(:,1) = y(:,1)-C*xhat(:,1);
    e(:,1) = x(:,1)-xhat(:,1);
    constraints = [constraints, norm(r(:,1),inf) <= threshold+slack,...
                    (-1)*sensor_limit <= y(:,2), y(:,2) <= sensor_limit,...
                    (-1)*actuator_limit <= u(:,1), u(:,1) <= actuator_limit,...
                    safex(1,:)'<=x(:,2),x(:,2)<=safex(2,:)'];
    x(:,i+1) = A*x(:,i) + B*u(:,i);
    y(:,i+1) = C*x(:,i+1);
    r(:,i+1) = y(:,i+1) - C*(A*xhat(:,i) + B*u(:,i));
    xhat(:,i+1) = A*xhat(:,i) + B*u(:,i) + L*r(:,i+1);
    e(:,i+1) = x(:,i+1) - xhat(:,i+1);
   constraints = [constraints, norm(r(:,i+1),inf) <= threshold+slack,...
                            (-1)*sensor_limit <= y(:,i+1), y(:,i+1) <= sensor_limit, ...
                            (-1)*actuator_limit <= u(:,i), u(:,i) <= actuator_limit,...
                            safex(1,:)'<=x(:,i+1),x(:,i+1)<=safex(2,:)'];
    cost = cost_func(u(:,i),x(:,i+1),perf,Q,R);
    sol1 = optimize([constraints,perf(1,:)'<=x(:,i+1),x(:,i+1)<=perf(2,:)'],[],ops);
    solved1 = sol1.problem;
    if solved1
        sol1 = optimize(constraints,[],ops);
        solved1 = sol1.problem;
    else
        fprintf("Can reach inside performance region from safex*"+num2str(factor)+"\n");
    end
    if solved1
        factor = factor -0.1;
        fprintf("reducing factor to "+num2str(factor)+"\n");
        x0 = factor*safex(1,:).';
        xhat0 = sdpvar(size(A,2),1);
        constraints = [safex(1,:).' <= xhat0, xhat0 <=  safex(2,:).'];  
        continue;
    else
        Knew = value(K_new{i});
    end
end
%% sim
inside = min((value(x(:,i)) >= [perf(1,:)']).*(value(x(:,i)) <= [perf(2,:)']));
m=8;
while m<lqr_iter
cost=0;
constraints = [];
for i=2:m-1
        K_new{i} = sdpvar(size(B,2),size(A,2));
        u(:,i) = -K_new{i}*xhat(:,i);
        x(:,i+1) = A*x(:,i) + B*u(:,i);
        y(:,i+1) = C*x(:,i+1);
        r(:,i+1) = y(:,i+1) - C*(A*xhat(:,i) + B*u(:,i));
        xhat(:,i+1) = A*xhat(:,i) + B*u(:,i) + L*r(:,i+1);
        e(:,i+1) = x(:,i+1) - xhat(:,i+1);
        constraints = [constraints,norm(r(:,i+1),inf) <= threshold-slack,...
                            (-1)*sensor_limit <= y(:,i+1), y(:,i+1) <= sensor_limit, ...
                            (-1)*actuator_limit <= u(:,i), u(:,i) <= actuator_limit,...
                            safex(1,:)'<=x(:,i+1),x(:,i+1)<=safex(2,:)'];
        cost = cost+cost_func(u(:,i),x(:,i+1),perf,Q,R)     
end
    sol = optimize([constraints,perf(1,:)'<=x(:,i+1),x(:,i+1)<=perf(2,:)'],cost,ops);
    solved = sol.problem
    if solved == 0
        for i=2:m-1
            Knew = [Knew;value(K_new{i})];
        end
        inside = min((value(x(:,m)) >= [perf(1,:)']).*(value(x(:,m)) <= [perf(2,:)']))
        break;
    else 
        fprintf("can not find suitable controllers in " +num2str(m)+" iterations\n");
        m=m+1;
    end
end
%% cost function : kinda CBF-CLF
function cost = cost_func(ui,xi,perf,Q,R)
    cost = 1;
    for i=1:size(perf,2)
        cost=cost*(-log(xi(i)-perf(1,i))-log(xi(i)-perf(2,i))); 
    end
    cost = xi'*Q*xi+ui'*R*ui;
end