clear;
clc;
clf;

system = "motor";

if system == "turning"
    Ac = -(25/3);
    Bc = 5;
    Cc = 1;
    Dc = 0;
    Ts = 0.02;    
    x_0 = 0;
    u_0 = 0;
    S_0 = 0;
    Kp = 0.5;
    Ki = 7;
    Kd = 0;
    t = 8/Ts;
    attack_beg = 4/Ts;
    attack_end = 5.6/Ts;
    control_beg = 5/Ts;
    attack = 1.5;
    n=1;%sensor under attack
    ref = 1; %S is the set point
    S = ref;
    upper_target = 1.2;
    lower_target = 0.8;
    upper_safety = 2.75;
    lower_safety = -2.75;
elseif system == "motor"
    b = 0.1;
    k = 0.01;
    j = 0.01;
    r = 1;
    l = 0.5;
    Ac = [0 1 0;0 -b/j k/j;0 -k/l -r/l];
    Bc = [0;0;1/l];
    Cc = eye(3);
    Dc = zeros(size(Cc,1),size(Bc,2));
    Ts = 0.1;    
    x_0 = [1.57;0;0];
    u_0 = 0;
    S_0 = [1.57;0;0];
    Kp = 11;
    Ki = 0;
    Kd = 5;
    t = 12/Ts;
    attack_beg = 6/Ts;
    attack_end = 10/Ts;
    control_beg = 7/Ts;
    attack = 2;
    n = 1;%2;%sensor under attack
    ref = [-1.57;0;0]; 
    S = ref;%S is the set point
    m = 1; %the state that is tracking a reference
    upper_target = -1.47;
    lower_target = -1.67;
    upper_safety = 4;
    lower_safety = -4;
elseif system == "aircraft"
    Ac = [-0.313 56.7 0;-0.0139 -0.426 0;0 56.7 0];
    Bc = [0.232;0.0203;0];
    Cc = eye(3);
    Dc = zeros(size(Cc,1),size(Bc,2));
    Ts = 0.02;    
    x_0 = [0;0;-0.1];
    u_0 = 0;
    S_0 = [0;0;0.5];
    Kp = 14;
    Ki = 0.8;
    Kd = 5.7;
    t = 5/Ts;
    attack_beg = 3/Ts;
    attack_end = 4.46/Ts;
    control_beg = 4/Ts;
    attack = 0.68;
    n = 3;%sensor under attack
    ref = [0;0;0.7]; 
    S = ref;%S is the set point
    m = 3; %the state that is tracking a reference
    upper_target = 0.72;
    lower_target = 0.68;
    upper_safety = 2;
    lower_safety = 0;
elseif system == "quadrotor"
    Ac = [-0.313 56.7 0;-0.0139 -0.426 0;0 56.7 0];
    Bc = [0.232;0.0203;0];
    Cc = eye(3);
    Dc = zeros(size(Cc,1),size(Bc,2));
    Ts = 0.02;    
    x_0 = [0;0;-0.1];
    u_0 = 0;
    S_0 = [0;0;0.5];
    Kp = 14;
    Ki = 0.8;
    Kd = 5.7;
    t = 5/Ts;
    attack_beg = 3/Ts;
    attack_end = 4.46/Ts;
    control_beg = 4/Ts;
    attack = 0.68;
    n = 3;%sensor under attack
    ref = [0;0;0.7]; 
    S = ref;%S is the set point
    m = 3; %the state that is tracking a reference
    upper_target = 0.72;
    lower_target = 0.68;
    upper_safety = 2;
    lower_safety = 0;
end

K = [zeros(1,size(Cc,1)) Kp*ones(1,size(Cc,1))...
    Ki*ones(1,size(Cc,1)) Kd*ones(1,size(Cc,1))];
Kg = inv(eye(size(Cc,1)) + Kd*Cc*Bc);
Apid = [Ac zeros(size(S,1));
        Cc zeros(size(S,1))];
Bpid = [Bc zeros(size(S,1));
        zeros(size(Bc)) -eye(size(S,1))];
Cpid = [Cc zeros(size(S,1));
        Cc zeros(size(S,1));
       zeros(size(Ac)) eye(size(S,1));
       Kg*Cc*Ac zeros(size(S,1))];
Dpid = [zeros(size(Bc)) zeros(size(S,1));
        zeros(size(Bc)) -eye(size(S,1));
        zeros(size(Bc)) zeros(size(S,1));
        zeros(size(Bc)) zeros(size(S,1))];

sys_ss_pid = ss(Apid,Bpid,Cpid,Dpid);
sys_d = c2d(sys_ss_pid,Ts);

A = sys_d.a;
B = sys_d.b;
C = sys_d.c;
D = sys_d.d;

x = [x_0 ; S_0];
u1 = u_0;
attack_vector = zeros(size(Cpid,1),1);
attack_vector(n) = attack;

plot_x = zeros(1,t);
plot_y = zeros(1,t);
plot_ref = zeros(1,t);

for i=1:t
   if i>=control_beg
       S = ref;
   else
       S = S_0;
   end
   u = [u1;S];
   x = A*x + B*u;
   y = C*x + D*u;
   
   if i>=attack_beg && i<=attack_end
        y = y + attack_vector;
   end
   
   u1 = -K*y;
   plot_x(i) = x(m);
   plot_y(i) = y(n);
   if size(A,1)==1
       plot_s(i) = S
   else
       plot_s(i) = S(m);
   end 
end

linewidth = 2;

hold on;
plot(upper_target*ones(1,t),'b-','LineWidth',linewidth);
plot(lower_target*ones(1,t),'b-','LineWidth',linewidth);
plot(upper_safety*ones(1,t),'r:','LineWidth',linewidth);
plot(lower_safety*ones(1,t),'r:','LineWidth',linewidth);
plot(plot_s,'LineWidth',linewidth);
plot(plot_x,'LineWidth',linewidth);
plot(plot_y,'LineWidth',linewidth);
axis([1 t lower_safety-1 upper_safety+1]);
legend({'target upper bound','target lower bound',...
    'safety upper bound','safety lower bound',...
    'reference', 'state','measurement'});
grid off;

new_sys= ss(A-B*K,B*Kp)