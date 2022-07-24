clf;
%% chi2 stats
l = 3;
dof=4;
mean_shift=3.2; % of the variable i.e. residue
X=[0:0.1:20];
Y=chi2pdf(X,dof);   % Central Chi2
Z1=ncx2pdf(X,dof*l,mean_shift);   % Non-central Chi2
Z2=ncx2pdf(X,dof*(l-1),mean_shift);   % Non-central Chi2
Z3=ncx2pdf(X,dof*(l-2),mean_shift);   % Non-central Chi2
th1= (6.2-1.5)*ones(size(Y));
th2= (6.2-1)*ones(size(Y));
th3= 6.2*ones(size(Y));

%% regions for coloring intersections
auc1=polyshape(X,Y);
auc21=polyshape(X,Z1);
auc22=polyshape(X,Z2);
auc23=polyshape(X,Z3);
auc31=polyshape(th1,Y);
auc32=polyshape(th2,Y);
auc33=polyshape(th3,Y);

far1 = intersect(auc1,auc31);
tpr1 = intersect(auc21,auc31);

far2 = intersect(auc1,auc32);
tpr2 = intersect(auc22,auc32);

far3 = intersect(auc1,auc33);
tpr3 = intersect(auc23,auc33);

%% plot 1
figure(1);
hold on;
plot(X,Y,X,Z3,th3,Y);
% hold on;
plot(far1);
% hold on;
plot(tpr1);
plot(far2);
% hold on;
plot(tpr2);
hold off;
legend('Unattacked','Attacked','Threshold');

%% plot area
figure(2);
hold on;
area(X,Y);
area(X,Z3);
area(th3,Y);
area(X,Z2);
area(th2,Y);
area(X,Z1);
area(th1,Y);
hold off;
legend('Unattacked','Attacked','Threshold');