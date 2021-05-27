clear;
clc;
clf;

system = "turning";

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
    attack = -1.5;
    ref = 1; %S is the set point
    S = ref;
    upper_target = 1.2;
    lower_target = 0.8;
    upper_safety = 2.75;
    lower_safety = -2.75;
end

K = [0 Kp Ki Kd];
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

plot_x = zeros(1,t);
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
       y = y + attack;
   end
   
   u1 = -K*y;
   plot_x(i) = x(1);
   plot_s(i) = S;
end

linewidth = 2;

hold on;
plot(upper_target*ones(1,t),'b-','LineWidth',linewidth);
plot(lower_target*ones(1,t),'b-','LineWidth',linewidth);
plot(upper_safety*ones(1,t),'r:','LineWidth',linewidth);
plot(lower_safety*ones(1,t),'r:','LineWidth',linewidth);
plot(plot_s,'LineWidth',linewidth);
plot(plot_x,'LineWidth',linewidth);
axis([1 t lower_safety-1 upper_safety+1]);
legend({'target upper bound','target lower bound','safety upper bound','safety lower bound','reference', 'state'});
grid off;
