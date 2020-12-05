function [y,isatk] = sys_env(i,th,tau,new_u,au,ay)
    %SYS_ENV Summary of this function goes here
%%%%%%%%%%%%%%%%%%%%%%%%%%%=======Systems=======%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%========================= esp ======================
% Ts=0.1;
% A = [0.4450 -0.0458;1.2939 0.4402];
% B = [0.0550;4.5607];
% C = [0 1];
% D = 0;
% K=[-0.0987 0.1420];
% L= [-0.0390;0.4339];
% safex = [1,2];
% tolerance = [0.1,0.1];
% th = 0.8;
% sensorRange = [2.5];
% actuatorRange = [0];

%====================trajectory tracking=======================
Ts=0.1;
A= [1.0000    0.1000;0    1.0000];
B= [0.0050;0.1000];
C= [1 0];
D= [0];
K= [16.0302    5.6622];  % settling time around 10
L = [0.9902;0.9892];
safex = [25,30];
tolerance = [1,1];
th = 2.15;              % new: changed it a bit so that without attack, the residue remains below threshold.
sensorRange = [30000];
actuatorRange = [36];
x1_0=normrnd(0,Sig_x10);
x2_0=normrnd(0,Sig_x20);

%%%%%%%%%%%%%%%%%%%%%%%%%%%=====FDI Attack=========%%%%%%%%%%%%%%%%%%%%%
%========================== obtained from code ======================%
% x1_0=-15.0;
% x2_0=16.6096079397;
% fdi_u=[ 35.99313719  71.98627437  58.33575204  41.29588025   0.   0.         -31.21892088  -5.09118639   0.           0.  -0.41831677   0.           0.           0.           0.   0. ];
% fdi_y=[15.01515367 13.48175215 11.26355514 10.30553807  8.22658704  4.85366237  3.61911687  2.21566942  1.21608204  0.53164255  0.1748743   0.  0.          0.          0.          0. ];
%======================================================================%

% ay=  [fdi_y,zeros(1,time-size(fdi_y,2))];
% au = [fdi_u,zeros(1,time-size(fdi_u,2))];

%%%%%%%%%%%%%%%%%%%%%%%%%%=====System Simulation======%%%%%%%%%%%%%%%%%%

%====================Chi Squared Test =================
% tau = 1; % Detection window % new: let's check for every iteration now
%======================================================


%========================= init ==========================
if i=1
    x = [x1_0;x2_0];
    xhat = [0;0];
    err = x-xhat;   % new: init est error
    u_attacked = 0;
    y = 0;
    y_attacked = 0;
    yhat = C*xhat;
    u = 0;
    % r = 0;
    r = y- yhat;
    
    x1(1) = abs(x(1));
    x2(1) = abs(x(2));
    x1hat(1) = xhat(1);
    x2hat(1) = xhat(2);
    
    % Initialization of Chi2 test variables
    est_err(1) = err(1);              % new: Estimation errors
    est_err(2,1) = err(2);
    sigma = cov(transpose(est_err));    % new: Covariance of estimation errors
    P = C*sigma*transpose(C);           % Covariance of residue -> P = C Sigma C_transpose + R; R = 0 in this simulation
    g(1) = 0;                         % gk initialized to 0
    z(1) = y_attacked-yhat;           % zk = yk - C*xhat_k
%     chi_tst= 
    uatk(1) = u_attacked(1);
    yatk(1) = y_attacked(1);
    rnorm(1) = norm(r,2);
    safex1(1)= safex(1);
    safex2(1)= safex(2);
    threshold(1)=th;
    t(1)=Ts;
else
    tm=i;
    r = y_attacked - yhat;
    r_norm = abs(r);
    
    xhat = A*xhat + B*u + L*r;
    x = A*x + B*u_attacked;
    err = x-xhat;   %est error
    
    u = -(K*xhat);
    u_attacked = u + au;
%   u_attacked = u;
    
    y = C*x;
    y_attacked = y + ay;
%     y_attacked = y;
    yhat = C*xhat;
    
    t(i+1) = tm;
    x1(i+1) = abs(x(1));
    x2(i+1) = abs(x(2));
    x_act(i+1)=x1(i+1);
    x_act(2,i+1)=x2(i+1);
    x1hat(i+1) = xhat(1);
    x2hat(i+1) = xhat(2);
    x_hat(i+1)=x1hat(i+1);
    x_hat(2,i+1)=x2hat(i+1);
    est_err(i+1) = err(1);     % new: Estimation error matrix
    est_err(2,i+1) = err(2);
    z(i+1) = r;                             % Residue zk = yk - C*xhat_k
    
    uatk(i+1) = u_attacked(1);
    rnorm(i+1) = norm(r,1);
    safex1(i+1)= safex(1);
    safex2(i+1)= safex(2);
    threshold(i+1)=th;
    
    sigma = cov(transpose(est_err));        % new: Update the error covariance (In steady state this remains approximately constant)
    P = C*sigma*transpose(C);       % Update the residue covariance C*Sigma*C_transpose    
    chi_tst(i+1)= transpose(z(i+1))*inv(P)*z(i+1);
    g(i+1) = 0;               
    if i-tau+1 > 0                  % Calculate detector window starting index
        srt = i-tau+1;              % tau is the detector window length
    else
        srt = 1;
    end
    for k=srt:i+1
        g(i+1) = g(i+1) + chi_tst(k);  % Calculate gk (chi2 test)
    end
    if g(i+1) > th
        u_attacked=u_attacked+new_u;
        isatk=1;
    else
        isatk=0;
    end
end

end

