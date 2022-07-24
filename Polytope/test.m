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
AA= [A zeros(size(A));L*C (A-L*C)] ;
BB= [B;B];
C1 = [C zeros(1,size(A,1))];
C2 = [zeros(1,size(A,1)) C];
KK = [zeros(size(K)) K];
AA_cl = AA-BB*KK;
A_cl = A-B*K;

%% robust invariant set
system = LTISystem('A', A_cl, 'B', [0;0;]);
system.x.min = -safex';
system.x.max = safex';
system.u.min = -36;
system.u.max = 36;
InvSet = system.invariantSet()
% p = InvSet.projection(1:2)
% p.plot();
plot(InvSet);