% X_R = 0.2*X_S

clear;
clc;
clf;
%% esp_bicycle
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
ini = 1;
perf = 0.2;
% th = 4.35; 
% settlingTime = 13 ;
% sensorRange = [2.5;15];  % columnwise range of each y
% actuatorRange = [0.8125;0.8125]; % columnwise range of each u
% noisy_zvar=0.1;
% noisy_zmean= 0.5;
%% esp
% Ts=0.04;
% A = [0.4450 -0.0458;1.2939 0.4402];
% B = [0.0550;4.5607];
% C = [0 1];
% D = [0];
% p = 0.00001;
% Q = p*(C'*C);
% R = 0.000001;
% [K,S,E] = dlqr(A,B,Q,R)
% sys_ss =ss(A-B*K,B,C,D,Ts);
% safex=[1 2];
% ini= 0.1;
% perf=0.1;
% QN = 5000000;
% RN = 10*eye(1);
% [kalmf,L,P,M] = kalman(sys_ss,QN,RN,Ts);
% % K = [-0.0987 0.1420];
% L = [-0.0390;0.4339];
% L 


%% ttc
%     Ts = 0.1;
%     A = [1.0000    0.1000; 0    1.0000];
%     B = [0.0050; 0.1000];
%     C = [1 0];
%     D = [0];
%     K = [16.0302    5.6622];  % settling time around 10
%     L = [0.9902; 0.9892];
% ini=0.1;
% perf=0.3;
% safex=[25 30]
xdim=size(A,1);
udim=size(B,2);
rng shuffle;
x=(2*ini*safex*rand(xdim)-safex*ini)';
proc_var= 1; meas_var=0.01;
% x = [0.0976827903;0.1724794346];
y = C*x;
z = zeros(xdim,1);
u =zeros(udim,1);
max_x1 = abs(x(1));
max_x2 = abs(x(2));

time=10000;
plot_dist=zeros(1,time);
plot_vel=zeros(1,time);

for i=1:time
    i;
   r = C*x - C*z+meas_var*rand(size(C,1),1);
   z = A*z + B*u + L*r;
   x = A*x + B*u + proc_var*rand(size(C,1),1);
   u = -K*z;
   
   if abs(x(1))>max_x1
       max_x1 = abs(x(1));
   end
   
   if abs(x(2))>max_x2
       max_x2 = abs(x(2));
   end  
   
   res(i,:)=r;
   ester(i,:)=(x-z);
   plot_dist(i) = x(1);
   plot_vel(i) = x(2);
   plot_dist_est(i) = z(1);
   plot_vel_est(i) = z(2);
   
end

figure
hold on;
plot(plot_dist);
plot(plot_dist_est);
legend({'x1','z1'});
axis([1 time -safex(1) safex(1)]);
hold off;

figure
hold on;
plot(plot_vel);
plot(plot_vel_est);
legend({'x2','z2'});
axis([1 time -safex(2) safex(2)]);
hold off;

max_x1
max_x2
cov(res)
mean(res)
mean(res)*inv(cov(res))*mean(res)'
