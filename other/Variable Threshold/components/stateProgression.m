function [y,g,zmean] = stateProg(i,th,dtc_windw,u_new,ay,au,Ts)
            t(i+1)= i*Ts;
            a_u(:,i+1)=au;
            a_y(:,i+1)=ay;
            x_act(:,i+1) = A*x_act(i)+B*uatk(i)+proc_noise(i);
%             xatk(i)= A*x_act(i)+B*u_act(i);
            xhat(:,i+1) = A*xhat(i) + B*u_act(i) + L*z(i);
            est_err(:,i+1)= x_act(:,i+1)-xhat(:,i+1);
            u_act(:,i+1)= K*xhat(:,i+1);
            uatk(:,i+1)= u_act(:,i+1)+a_u(:,i+1);
            y_act(:,i+1)= C*x_act(i+1)+meas_noise(i);
            yatk(:,i+1)= y_act(:,i+1)+a_y(:,i+1);
            z(:,i+1)= yatk(:,i+1)-C*xhat(:,i+1);
            z_mean(:,i+1)= mean(z);
            P = cov(z');
            chi_tst(i+1)= z(:,i+1)'*inv(P)*z(:,i+1);
            threshold(i+1)= th;
            tau(i+1)= dtc_windw;
            g(i+1) = 0;               
%             if i-tau+1 > 0                  % Calculate detector window starting index
%                 srt = i-tau+1;              % tau is the detector window length
%             else
%                 srt = 1;
%             end
            for k = max(1,i-tau(i+1)+1):i+1
                g(i+1) = g(i+1) + chi_tst(k);  % Calculate gk (chi2 test)
            end
            if g(i+1) > threshold(i+1)
                u_act(:,i+1) = u_new;
                uatk(:,i+1)= u_act(:,i+1)+a_u(:,i+1);
                isatk=1;
            else
                isatk=0;
            end

        end