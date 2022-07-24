clc;
clear all;
yalmip("clear");
system = "trajectory"
ops = sdpsettings('verbose',1);

% vehicle suspension control (Multi-Objective Co-Optimization of FlexRay-based Distributed Control System)--in CCM ECU
% if system=="suspension_control"
%   states: car position, car velocity, suspension load position, suspension velocity
%   input suspension load force
%   output car position
%     (paper)
    Ts = 0.04;
    Ac = [0 1 0 0;-8 -4 8 4;0 0 0 1;80 40 -160 -60];
    Bc = [0;80;20;1120];
    Cc = [1,0,0,0];
    Dc = [0];    
    sys_ct= ss(Ac,Bc,Cc,Dc);
    sys_dt= c2d(sys_ct,Ts);
    [A,B,C,D]= ssdata(sys_dt);  
%   (ctms)
%     Ts = 0.0005;
%     A = [0.983430135636641,0.0398092173976150,-0.00779363998762477,-0.000249899156869949,0;-1.01787772935251,0.983430135636641,-0.169054214724110,-0.0133669304232909,0;1.07355168319532,0.0271118904282865,-0.204121957253967,0.0104950906494122,0;12.1940202887991,0.511020193388355,-19.6771193083757,0.309373655207152,0;0,0,0.000500000000000000,0,1]
%     B = [3.08397717947154e-07,0.0165698643633592;1.50427924310795e-05,1.01787772935251;1.47027444962422e-06,-1.07355168319532;0.000104189490905809,-12.1940202887991;0,0];
%     C = [0,0,1,0,0];
%     D = [0,0];
    % safery margin
%   (paper)
    safex = 4.*[-20,-200,-100,-600;20,200,100,600];%[-15,-15,-20,-20;15,15,20,20];
%   (ctms)
%     safex = 2.*[-20,-200,-100,-600,-1000;20,200,100,600,1000];%[-15,-15,-20,-20;15,15,20,20];
    % safer region of this system to start from
    ini = 0.5.*safex;
    perf = 0.1.*safex;
    % for central chi2 FAR < 0.05
    threshold = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    sensor_limit = 20;  % columnwise range of each y
    actuator_limit = 100;   % columnwise range of each u
    qweight= 10000;
    rweight = 0.0001;
    Q= qweight*(C'*C);
    Q(2,2)=10;Q(3,3)=1;Q(4,4)=10;
    R= rweight;
    proc_dev= 0.01; meas_dev=0.001;
    QN = 0.005*(B*B');
    RN = 0.00005;
    [K,S,E] = dlqr(A,B,Q,R);
%     sys_ss = ss(A,B,C,D,0.1);
%     [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
    % using lqg ragulator(paper)
%     K= [7.6110    0.3412    0.0186    0.0157];
    L= [0.1298; 0.1642; 0.1312; -0.0622];

%    (ctms)
%    K = 1.0e+09.*[0.0534    0.0000    1.0898    0.0011    1.8286];
%    L= [0.1298; 0.1642; 0.1312; -0.0622; 1];

    settlingTime = 0.2800/Ts;
%     s.noisy_zvar= 2.6328;
%     s.noisy_zmean= 0.2719;
%     s.noisy_delta= 1.86;
%     s.nonatk_zvar= 12.6041;%15.8507
%     s.nonatk_zmean= 0.6064;
%     s.nonatk_delta= 0.0292;
    s.uatkon=[1];   % attack on which u
    s.yatkon=[1];   % attack on which y
    rate =0.5;
% end
n =2;
x0 = safex(:,1)';
x_dest = perf(:,1)';
q = (norm(x_dest,inf)/norm(x0,inf))^(-n);
P = dlyap(A,-q*eye(size(A)));

x =sdpvar(size(A,1),1);
cons = [x'*P*x <= -q.*P];
optimize(cons,-norm(x,2),ops);
value(x)