function [g,z,z_mean,chi_tst] = testchi2(r,i,z,chi_tst,g,tau)
i
z(:,i)=r
z_mean=mean(z)
P= size(z',1)
chi_tst(i)= z(:,i)'*z(:,i)/P
g(i) = 0;              
for k = max(1,i-tau+1):i
   g(i) = g(i) + chi_tst(k)  % Calculate gk (chi2 test)
end