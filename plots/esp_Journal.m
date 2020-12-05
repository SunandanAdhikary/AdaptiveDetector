clear;
clc;
clf;

A = [0.6278   -0.0259; 0.4644    0.7071];

B = [0.1246   -0.00000028;3.2763    0.000016];

C = [0    1.0000; -338.7813    1.1293];

D = [0         0;169.3907         0];

K = [5.261 -0.023;-414911.26, 57009.48];

L = [-0.00000000002708 -0.00000000063612;0.00000000033671  0.00000000556308];

abs(eig(A-B*K))
abs(eig(A-L*C))

x = [0.1;0.2];
y = C*x;
z = [0;0];
u = [0;0];
max_x1 = abs(x(1));
max_x2 = abs(x(2));

time=14;
plot_dist=zeros(1,time);
plot_vel=zeros(1,time);

for i=1:time
    i
   r = C*x - C*z;
   z = A*z + B*u + L*r;
   x = A*x + B*u;     
   u = -K*z;
   
   if abs(x(1))>max_x1
       max_x1 = abs(x(1));
   end
   
   if abs(x(2))>max_x2
       max_x2 = abs(x(2));
   end  
   
   plot_dist(i) = x(1);
   plot_vel(i) = x(2);
   plot_dist_est(i) = z(1);
   plot_vel_est(i) = z(2);
end


% esp_journal_est=ss(A-B*K-L*C)
hold on;
plot(plot_dist);
plot(plot_vel);
plot(plot_dist_est);
plot(plot_vel_est);
legend({'x1','x2','z1','z2'});
axis([1 time -1 1]);
hold off;

max_x1
max_x2



% X_R = 0.2*X_S

% A = [0.4450 -0.0458;1.2939 0.4402];
% B = [0.0550;4.5607];
% C = [0 1];
% D = [0];
% 
% sys =ss(A,B,C,D);
% p = 0.00001;
% Q = p*(C'*C);
% R = 0.000001;
% [K,S,E] = dlqr(A,B,Q,R);
% 
% % K = [-0.0987 0.1420];
% L = [-0.0390;0.4339];
% 
% x = [-0.1;-0.2];
% y = C*x;
% z = [0;0];
% u = 0;
% max_x1 = 0;
% max_x2 = 0;
% 
% time=14;
% plot_dist=zeros(1,time);
% plot_vel=zeros(1,time);
% 
% for i=1:time
%     i
%    r = C*x - C*z;
%    z = A*z + B*u + L*r;
%    x = A*x + B*u;     
%    u = -K*z;
%    
%    if abs(x(1))>max_x1
%        max_x1 = abs(x(1));
%    end
%    
%    if abs(x(2))>max_x2
%        max_x2 = abs(x(2));
%    end  
%    
%    plot_dist(i) = x(1);
%    plot_vel(i) = x(2);
%    
%    plot_dist_est(i) = z(1);
%    plot_vel_est(i) = z(2);
% end
% 
% 
% hold on;
% plot(plot_dist);
% plot(plot_vel);
% plot(plot_dist_est);
% plot(plot_vel_est);
% legend({'x1','x2','z1','z2'});
% axis([1 time -1 1]);
% hold off;
% 
% max_x1
% max_x2

esp=ss(A-B*K,B,C,D,0.04);
st= stepinfo(esp)