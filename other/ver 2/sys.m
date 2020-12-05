% X_R = 0.2*X_S

clear;
clc;
clf;
%% esp
Ts=0.04;
A = [0.4450 -0.0458;1.2939 0.4402];
B = [0.0550;4.5607];
C = [0 1];
D = [0];
p = 0.00001;
Q = p*(C'*C);
R = 0.000001;
[K,S,E] = dlqr(A,B,Q,R)
sys_ss =ss(A-B*K,B,C,D,Ts);
safex=[1 2];
ini= 0.1;
perf=0.1;
QN = 5000000;
RN = 10*eye(1);
[kalmf,L,P,M] = kalman(sys_ss,QN,RN,Ts);
% K = [-0.0987 0.1420];
L = [-0.0390;0.4339];
L 


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

rng shuffle;
x=[2*ini*safex(1)*rand(1)-ini*safex(1);...
                    2*ini*safex(2)*rand(1)-ini*safex(2)]
% x = [0.0976827903;0.1724794346];
y = C*x;
z = [0;0];
u = 0;
max_x1 = abs(x(1));
max_x2 = abs(x(2));

time=100;
plot_dist=zeros(1,time);
plot_vel=zeros(1,time);

for i=1:time
    i;
   r = C*x - C*z+0.01*rand;
   z = A*z + B*u + L*r;
   x = A*x + B*u + rand;
   u = -K*z;
   
   if abs(x(1))>max_x1
       max_x1 = abs(x(1));
   end
   
   if abs(x(2))>max_x2
       max_x2 = abs(x(2));
   end  
   
   res(i)=r;
   ester(:,i)=(x-z);
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
var(res')