clc;
clear all;

%% yalmip, solver setup
% addpath(genpath('C:\Program Files\MATLAB\R2018a\toolbox\sdpt3-master\'));
% install_sdpt3;
% addpath(genpath('C:\Program Files\MATLAB\R2018a\toolbox\YALMIP-master\'));
% yalmiptest;
% addpath(genpath('C:\Program Files\MATLAB\R2018a\toolbox\gurobi911\'));
% gurobi_setup;
% savepath; % open with admin proviliages
% % sdpsettings('verbose',0,'solver','sdpt3');
sdpsettings('verbose',0,'solver','sdpt3');
clc;


%% input
system = "trajectory"
format long g


%% systems
if system== "esp"
%%   esp
     Ts= 0.04;
     A = [0.4450 -0.0458;1.2939 0.4402];
     B = [0.0550;4.5607];
     C = [0 1];
     D = 0;
    % K=[-0.0987 0.1420];
     K = [0.2826    0.0960];
     K1 = [-0.2825979054 0.0165073962]
     
     L = [-0.0390;0.4339];
     safex = [1,2];
     safer=0.9;
    % from perfReg.py with this system
     init = 1;
    % from perfReg.py with this system
     perf = 0.1;
    % for central chi2 FAR < 0.05
     th = 4.5; 
     settlingTime = 5 ;
     sensorRange = [2.5] ;     % columnwise range of each y
     actuatorRange = [0.8125]; % columnwise range of each u
    % from system_with_noise.m with this system
     nonatk_zvar=0.1169;
    % from system_with_noise.m with this system
     noisy_zmean= 0.5;
     uatkon=[1];   % attack on which u
     yatkon=[1];   % attack on which y
     regionCt= 3;
     robustK = [K+K1;...
                K;...
                K];
    a_y = [-2 -2];
    a_u = [-0.25 0];
    
end

if system=="esp_journal"
%%   esp journal
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
    % from perfReg.py with this system
     init = 1;
    % from perfReg.py with this system
     perf = 0.2;
    % for central chi2 FAR < 0.05
     th = 4.35; 
     settlingTime = 12 ;
     sensorRange = [2.5;15];  % columnwise range of each y
     actuatorRange = [0.8125;10000]; % columnwise range of each u
    % from system_with_noise.m with this system
     noisy_zvar=[10000 -20;-20 15507];   
    % from system_with_noise.m with this system
     noisy_zmean= [3.4563 -371.7262];
     uatkon=[1;0];   % attack on which u
     yatkon=[1;1];   % attack on which y
end

if system=="trajectory"
%% trajectory tracking
     Ts = 0.1;
     A = [1.0000    0.1000; 0    1.0000];
     B = [0.0050; 0.1000];
     C = [1 0];
     D = [0];
     K = [16.0302    5.6622];  % settling time around 10
     L = [0.9902; 0.9892];
     safex = [25,30];
     safer= 0.7;
    % from perfReg.py with this system
     init = 1;
    % from perfReg.py with this system
     perf = 0.3;
    % for central chi2 FAR < 0.05
     th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
     sensorRange = [30];  % columnwise range of each y
     actuatorRange = [36];   % columnwise range of each u
     settlingTime = 13; 
    % from system_with_noise.m with this system
     nonatk_zvar=12.6;
    % from system_with_noise.m with this system
     noisy_zmean= 0.5;
     uatkon=[1];   % attack on which u
     yatkon=[1];   % attack on which y
     regionCt= 12;
     robustK = [K+[-15.9052	-5.7872];...
                K+[-15.9052	-5.6622];...
                K+[-15.9052	-5.6622];...
                K+[-15.9052	-5.1622];...
                K+[-15.9052	-5.1622];...
                K+[-15.9052	-4.6622];...
                K+[-15.9052	-4.6622];...              
                K+[-15.9052	-3.6622];...
                K+[-15.9052	-3.1622];...
                K+[-15.9052	-2.6622];...
                K+[-5.7718267942 -9.1534280701]];
      a_u= [-23.6036036036 -23.3333333333 ...
          -23.3333333333 -23.3333333333 ...
             -23.3333333333 -23.3333333333 -23.3333333333 ...
                  -23.3333333333  -23.3333333333];
      a_y= [25 22.1180180180 19.4707207207 17.0567567567 ...
               14.8761261261 12.9288288288 11.2148648648 9.7342342342 ...
                             8.4869369369];

end
%% sizes
xdim= size( A,2);
ydim= size( C,1);
udim= size( B,2);
%% ranges
ulim=  actuatorRange;
ylim=  sensorRange;
len= settlingTime*3;
%% new state space

AA= [A zeros(size(A));L*C (A-L*C)] ;
BB= [B;B];
CC= [C zeros(size(C))];
DD= [D];
KK= -[zeros(size(K)) K];
AA_cl= AA-BB*KK;
%% problem
P_di=sdpvar(size(AA_cl,1),size(AA_cl,1));
Problm_di=[AA_cl'*P_di*AA_cl-P_di zeros(size(AA_cl));
              zeros(size(AA_cl)) -P_di];

constraint_di= Problm_di <= 0;
optimize(constraint_di);
P=double(P_di);  

