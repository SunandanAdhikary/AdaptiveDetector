clf;
%% chi2 stats
dof=4;
mean_shift=3.2; % of the variable i.e. residue
X=[0:0.1:20];
Y=chi2pdf(X,dof);   % Central Chi2
Z=ncx2pdf(X,dof,3.2);   % Non-central Chi2
th= 6.2*ones(size(Y));

%% regions for coloring intersections
auc1=polyshape(X,Y);
auc2=polyshape(X,Z);
auc3=polyshape(th,Y);
far = intersect(auc1,auc3);
tpr = intersect(auc2,auc3);

%% plot 1
figure(1);
plot(X,Y,X,Z,th,Y);
% hold on;
% plot(far);
% hold on;
% plot(tpr);
hold off;
legend('Unattacked','Attacked','Threshold');

%% plot area
figure(2);
area(X,Y);
hold on;
area(X,Z);
hold on;
area(th,Y);
hold off;
legend('Unattacked','Attacked','Threshold');