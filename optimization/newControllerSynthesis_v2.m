clc;
clear all;
yalmip("clear");
system = "trajectory"
ops = sdpsettings('verbose',1);
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
    Ts =0.1;
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
% figure
% plot(xi')
% figure
% plot(xhati')
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
xi
xhati
% figure;
% plot(xi');
% plot(xhati');

if lqr_iter < k
    lqr_iter = k
end
if est_iter < j
    est_iter =j
end
%% gain synthesis
% augmented sytem modeling
Aaug = [A, zeros(size(A));L*C*A, A-L*C*A];
Baug = [B; B];
Caug1 = [C zeros(1,size(A,1))];
Caug2 = [zeros(1,size(A,1)) C];
Daug = zeros(size(Caug1,1),size(Baug,2));
Kaug = [zeros(size(K)) K];
q= 0.001;
r =0;%.0001;
Q= q*eye(size(Aaug,2));
R= r*eye(size(Baug,2));
%% init
solved = 1;
factor = 1;
slack = 0.0001;
augsys_d = ss(Aaug-Baug*Kaug,Baug,Caug1,Daug,Ts);
augsys = d2c(augsys_d);
[AA,BB,CC,DD] = ssdata(augsys);
% KK = sdpvar(size(Kaug))
while solved ~= 0
% x0 = sdpvar(size(A,2),1);
% assign(x0,safex(1,:).');
x0 = factor*safex(1,:).';
xhat0 = sdpvar(size(A,2),1);
constraints = [safex(1,:).' <= xhat0, xhat0 <=  safex(2,:).'];  
n = lqr_iter;
u = sdpvar(size(BB,2),n);
x = sdpvar(size(AA,2),n+1);
%     xhat = sdpvar(size(A,2),n+1);
e = sdpvar(size(AA,2),n+1);
y = sdpvar(size(Caug1,1),n+1);
r = sdpvar(size(CC,1),n+1);
K_new = {};
Pmat = {};
lyap_decay = [];
cost = 0;
x(:,1) = [x0;xhat0];
% xhat(:,1) = xhat0;
% u(:,1) = -K_new{1}*x(:,1);
y(:,1) = Caug1*x(:,1);
r(:,1) = y(:,1)-Caug2*x(:,1);
figure("Name","states from "+factor+" times inside the safety boundary")
hold on;
i = 1 ;
constraints = [constraints, norm(r(:,1),inf) <= threshold-slack];
inside = min((value(x(:,i)) >= [perf(1,:)';perf(1,:)']).*(value(x(:,i)) <= [perf(2,:)';perf(2,:)']));
while ~inside
%     Pmat{i} = sdpvar(size(Aaug,1),size(Aaug,2));
    K_new{i} = sdpvar(size(Kaug,1),size(Kaug,2));
%     K_new = inv(R+Baug'*Pmat{i}*Baug)*Baug'*Pmat{i}*Aaug;
    u(:,i) = -K_new{i}*x(:,i);
%     u(:,i) = -inv(R+Baug'*value(Pmat{i})*Baug)*Baug'*value(Pmat{i})*Aaug*x(:,i);
    y(:,i+1) = Caug1*x(:,i+1);
    r(:,i+1) = y(:,i+1) - Caug2*(Aaug*x(:,i) + Baug*u(:,i));
    x(:,i+1) = Aaug*x(:,i) + Baug*u(:,i);
%     x(:,i+1) = Aaug*x(:,i) - Baug*inv(R+Baug'*value(Pmat{i})*Baug)*Baug'*value(Pmat{i})*Aaug*x(:,i);
    if i > 1
        constraints =[];
    end
    constraints = [constraints,... 
                        norm(r(:,i+1),inf) <= threshold-slack,...
                        (-1)*sensor_limit <= y(:,i+1), y(:,i+1) <= sensor_limit,...
                        (-1)*actuator_limit <= u(:,i+1), u(:,i+1) <= actuator_limit,...
                        [safex(1,:)';safex(1,:)']<=x(:,i+1),x(:,i+1)<=[safex(2,:)';safex(2,:)']];
    cost = cost_func(u(:,i),x(:,i+1),perf,Q,R)
    sol = optimize(constraints,cost,ops);
%       constraints = [constraints,... 
%                         (Aaug-Baug*inv(R+Baug'*Pmat{i}*Baug)*Baug'*Pmat{i}*Aaug)'*Pmat*(Aaug-Baug*inv(R+Baug'*Pmat{i}*Baug)*Baug'*Pmat{i}*Aaug)-(1+lyap_decay(i))*eye(size(A,1)*2)*Pmat{i}<=0, Pmat{i}>=slack,...
%                         norm(r(:,i),inf) <= threshold-0.0001,...
%                         (-1)*sensor_limit <= y(:,i), y(:,i) <= sensor_limit,...
% %                         (-1)*actuator_limit <= u(:,i), u(:,i) <= actuator_limit,...
%                         (-1)*actuator_limit <= -inv(R+Baug'*value(Pmat{i})*Baug)*Baug'*value(Pmat{i})*Aaug*x(:,i),...
%                         -inv(R+Baug'*value(Pmat{i})*Baug)*Baug'*value(Pmat{i})*Aaug*x(:,i) <= actuator_limit,...
% %                         [safex(1,:)';safex(1,:)']<=x(:,i+1),x(:,i+1)<=[safex(2,:)';safex(2,:)'],...
%                         [safex(1,:)';safex(1,:)']<=Aaug*x(:,i) - Baug*inv(R+Baug'*value(Pmat{i})*Baug)*Baug'*value(Pmat{i})*Aaug*x(:,i)...
%                         Aaug*x(:,i) - Baug*inv(R+Baug'*value(Pmat{i})*Baug)*Baug'*value(Pmat{i})*Aaug*x(:,i)<=[safex(2,:)';safex(2,:)']];
%     sol = optimize(constraints,lyap_delay(i),ops);
    solved = sol.problem;
    if solved == 0
%         value(-inv(R+Baug'*Pmat{i}*Baug)*Baug'*Pmat{i}*Aaug)
%         value(-inv(R+Baug'*Q*Baug)*Baug'*Q*Aaug);
        value(K{i})
        plot(x(:,i)');
    else
        if i==1
            factor = factor-0.1;
            fprintf("no controller found from "+num2str(factor)+"* safex \n");
%         else
%             fprintf("no controller found for "+num2str(factor)+" after "+num2str(i)+" iterations \n");
        end
        break;
    end
    i = i+1;
    inside = min((value(x(:,i)) >= [perf(1,:)';perf(1,:)']).*(value(x(:,i)) <= [perf(2,:)';perf(2,:)']));
end
hold off;
if factor <= 0.5
    break;
else
    fprintf("========================= no controller found for "+num2str(factor)+" after "+num2str(i)+" iterations ==================\n")
    break;
end
end
%% cost function : kinda CBF-CLF
function cost = cost_func(ui,xi,perf,Q,R)
    cost = 1;
    for i=1:size(perf,2)
        cost=cost*(-log(xi(i)-perf(1,i))-log(xi(i)-perf(2,i))); 
    end
%     cost = xi'*Q*xi+ui'*R*ui;
end