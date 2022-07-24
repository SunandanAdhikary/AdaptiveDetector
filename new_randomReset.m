function in = new_randomReset(in, init, safex, simlen, xdim, ydim, ylim, ulim, C, K, th, delta)
%     disp("resetting");
    % Randomization Parameters
    % Note: Y-direction parameters are only used for the 3D walker model
    rng shuffle;
    seed= rng;
    %% noise
    proc_noise= 1*zeros(xdim,simlen);
    in = in.setVariable('s.proc_noise', proc_noise);
    meas_noise= 0.01*zeros(ydim,simlen);
    in = in.setVariable('s.meas_noise', meas_noise);
    %% init
    t= 0.00;
    in = in.setVariable('s.time', t);
    x =zeros(xdim,simlen);
%         init_x = [];
    for k=1:xdim
       x(k,1) = (2*init*safex(k)*rand-safex(k)*init)'
    end   
%     init_x =(2*init*safex*rand*eye(xdim)-safex*init)'
%      x = [2*s.safex'*randn(1,xdim)-s.safex' zeros(xdim,simlen-1)] 
%     x(:,1) = init_x ;
    in = in.setVariable('s.x_act', x);
    xhat = zeros(xdim,simlen);
    in = in.setVariable('s.xhat', xhat);
    est_err= x -xhat;
    in = in.setVariable('s.est_err', est_err);
%     u_act = max(-ulim,min(ulim,K*xhat));
%     in = in.setVariable('s.u_act', u_act);
%     a_u= zeros(size(u_act));
%     in = in.setVariable('s.a_u', a_u);
%     uatk= max(-ulim,min(ulim,u_act +a_u));
%     in = in.setVariable('s.uatk', uatk);
%     y_act = max(-ylim,min(ylim,C*x+meas_noise));
%     in = in.setVariable('s.y_act', y_act);
%     a_y = zeros(size(y_act));
%     in = in.setVariable('s.a_y', a_y);
%     yatk = max(-ylim,min(ylim,y_act+a_y));
%     in = in.setVariable('s.yatk', yatk);
%     z = yatk-C*xhat;
%     in = in.setVariable('s.z', z);
%     in = in.setVariable('s.z_mean', zeros(size(z,1),simlen));
%     in = in.setVariable('s.z_var',zeros(size(z,1),simlen));
%     g = zeros(1,simlen);
%     in = in.setVariable('s.g', g);
%     chi_tst= zeros(1,simlen);
%     in = in.setVariable('s.chi_tst', chi_tst);
%     threshold= th*ones(1,simlen);
%     in = in.setVariable('s.threshold', threshold);
%     tau= ones(1,simlen);
%     in = in.setVariable('s.tau', tau);
%     non_cent= zeros(1,simlen);
%     in = in.setVariable('s.non_cent', non_cent);
%     avgfar= chi2cdf(th,1*size(C,1),'upper');
%     in = in.setVariable('s.avgfar', avgfar.*ones(1,simlen));
%     avgtpr = ncx2cdf(th,1*size(C,1),non_cent,'upper');
%     in = in.setVariable('s.avgtpr', avgtpr.*ones(1,simlen));
%     disp("reset");
%     evalin('base','save("system.mat","-struct","s")');
end