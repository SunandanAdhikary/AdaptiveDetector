classdef RlEnvionment < matlab.System
    % Untitled4 Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties

    end

    properties(DiscreteState)
        x_act
        xhat
        est_err
        u_act
        y_act
        a_u
        a_y
        uatk
        yatk
        z
        z_mean
        P
        g
        chi_tst
        t
        threshold
        tau
    end

    % Pre-computed constants
    properties(Access = private)
        Tf = 200;
        %% esp
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

        %% trajectory tracking
        Ts = 0.1;
        A = [1.0000    0.1000;0    1.0000];
        B = [0.0050;0.1000];
        C = [1 0];
        D = [0];
        K = [16.0302    5.6622];  % settling time around 10
        L = [0.9902;0.9892];
        safex = [25,30];
        tolerance = [1,1];
        th = 2.15;              % new: changed it a bit so that without attack, the residue remains below threshold.
        sensorRange = [30000];
        actuatorRange = [36];
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function y = stepImpl(obj,u)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
           obj.t(i+1)=i*obj.Ts;
            obj.a_u(:,i+1)=au;
            obj.a_y(:,i+1)=ay;
            obj.x_act(:,i+1) = obj.A*obj.x_act(i)+obj.B*obj.uatk(i)+obj.proc_noise(i);
%             obj.xatk(i)= obj.A*obj.x_act(i)+obj.B*obj.u_act(i);
            obj.xhat(:,i+1) = obj.A*obj.xhat(i) + obj.B*obj.u_act(i) + obj.L*obj.z(i);
            obj.est_err(:,i+1)= obj.x_act(:,i+1)-obj.xhat(:,i+1);
            obj.u_act(:,i+1)= obj.K*obj.xhat(:,i+1);
            obj.uatk(:,i+1)= obj.u_act(:,i+1)+obj.a_u(:,i+1);
            obj.y_act(:,i+1)= obj.C*obj.x_act(i+1)+obj.meas_noise(i);
            obj.yatk(:,i+1)= obj.y_act(:,i+1)+obj.a_y(:,i+1);
            obj.z(:,i+1)= obj.yatk(:,i+1)-C*obj.xhat(:,i+1);
            obj.z_mean(:,i+1)= mean(obj.z);
            obj.P = cov(obj.z');
            obj.chi_tst(i+1)= z(:,i+1)'*inv(P)*z(:,i+1);
            obj.threshold(i+1)= th;
            obj.tau(i+1)= dtc_windw;
            obj.g(i+1) = 0;               
%             if i-tau+1 > 0                  % Calculate detector window starting index
%                 srt = i-tau+1;              % tau is the detector window length
%             else
%                 srt = 1;
%             end
            for k = max(1,i-obj.tau(i+1)+1):i+1
                obj.g(i+1) = obj.g(i+1) + obj.chi_tst(k);  % Calculate gk (chi2 test)
            end
            if obj.g(i+1) > obj.threshold(i+1)
                obj.u_act(:,i+1) = u_new;
                obj.uatk(:,i+1)= obj.u_act(:,i+1)+obj.a_u(:,i+1);
                isatk=1;
            else
                isatk=0;
            end
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
             obj.t(1)=0*obj.Ts;            
            obj.x_act(:,1) =[2*obj.safex(1,1)*randn(1)-obj.safex(1,1);2*obj.safex(1,2)*randn(1)-obj.safex(1,2)];
%             obj. xatk(1)= obj.x_act(1);
            obj.xhat(:,1)=[0;0];
            obj.est_err(:,1)= obj.x_act(:,1)-obj.xhat(:,1);
            obj.u_act(:,1)= obj.K*obj.xhat(:,1);
            obj.a_u(:,1)= zeros(size(obj.u_act(:,1)));
            obj.uatk(:,1)= obj.u_act(:,1)+obj.a_u(:,1);
            obj.y_act(:,1)= obj.C*obj.x_act(:,1);
            obj.a_y(:,1)= zeros(size(obj.y_act(:,1)));
            obj.yatk(:,1)= obj.y_act(:,1)+obj.a_y(:,1);
            obj.z(:,1)= obj.yatk(:,1)-obj.C*obj.xhat(:,1);
            obj.z_mean(:,1)= zeros(size(obj.z));
            obj.P= cov(obj.z');
            obj.g(1)= 0;
            obj.chi_tst(1)= 0;
            obj.threshold(1)= obj.th;
            obj.tau(1)= 1;
        end
    end
end
