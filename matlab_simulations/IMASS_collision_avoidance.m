clc
clear all
close all

N = 8; %Number of agents

%Network structure

% A = [0 1 1 0 0 0 0 0; 1 0 1 0 0 0 0 0;
%      1 1 0 1 0 0 0 0; 0 0 1 0 1 0 0 0;
%      0 0 0 1 0 1 0 0; 0 0 0 0 1 0 1 1;
%      0 0 0 0 0 1 0 1; 0 0 0 0 0 1 1 0];
A = ones(N,N);
A = A - diag(sum(A,2));
L = diag(sum(A,2)) - A;
%Formation I

tau_I_x = [0 2 4 2 2 0 2 4]';
tau_I_y = [0 0 0 1.25 2.75 4 4 4]'+2;

%Formation M

tau_M_x = [0 3 5 4.5 0.5 1 2 4]';
tau_M_y = [0 2 0 2.0 2.0 4 2 4]'+2;

%Formation A
tau_A_x = [0 1 2 3 4 5 6 3]';
tau_A_y = [0 2 4 6 4 2 0 3]'+2;


%Formation S
% tau_S_x = [0 2 3 2 0 -1 0 2]';
% tau_S_y = [0 0 2 4 4 6 8 8]';

tau_S_x = [0 0 -1 0 2 3 2 2]';
tau_S_y = [0 4 5 8 8 2 0 4]';
%Initialize the locaton of the robots
x_ini = [0 1 2 3 4 5 6 7]';
y_ini = [0 0 0 0 0 0 0 0]';
yaw_ini = pi/2*ones(N,1);

x = x_ini;
y = y_ini;
yaw = yaw_ini;

%Step size
dt = 0.01;

err = 100;

num_formations = 4;
for l = 1:num_formations
    if l == 1
        tau_des_x  = tau_I_x;
        tau_des_y = tau_I_y;
    elseif l == 2
        tau_des_x = tau_M_x;
        tau_des_y = tau_M_y;
    elseif l == 3
        tau_des_x = tau_A_x;
        tau_des_y = tau_A_y;
    
    elseif l == 4
        tau_des_x = tau_S_x;
        tau_des_y = tau_S_y;
    end
    err_x = L*x - L*tau_des_x;
    err_y = L*y - L*tau_des_y;
    err = sum(sqrt(err_x.^2 + err_y.^2));
    while err > 4
        ux = zeros(N,1);
        uy = zeros(N,1);

        vx = zeros(N,1);
        vy = zeros(N,1);

        for i = 1:N
            collision = 0;
            for j = 1:N
                if (i ~= j)
                    D_min = 0.2;
                    d_ij = [tau_des_x(i) - tau_des_x(j); tau_des_y(i)-tau_des_y(j)];
                    l_ij = [x(i) - x(j); y(i)-y(j)];
                    norm(l_ij)
                    if (norm(l_ij) < D_min)
                        Collision = 1
                    end
                    %                     num_w = norm(d_ij - l_ij) - (norm(d_ij) - D_min);
                    %                     den_w = abs(norm(d_ij) - D_min - norm(d_ij - l_ij));
                    
                    if (norm(l_ij) < D_min+0.2)
                        num_w = 2*(norm(l_ij) - D_min);
                        den_w = (abs(norm(l_ij) - D_min))^2;
                        w_ij = num_w/den_w;
                        ux(i) = ux(i) + A(i,j)*w_ij*((x(i) - x(j))) + 3*randn;
                        uy(i)= uy(i) + A(i,j)*w_ij*((y(i) - y(j))) + 3*randn;
                        j = j+10;
                        
                    else
                        w_ij = 1;
                        ux(i) = ux(i) + A(i,j)*w_ij*((x(i) - x(j)) - (tau_des_x(i) -tau_des_x(j)));
                        uy(i)= uy(i) + A(i,j)*w_ij*((y(i) - y(j)) - (tau_des_y(i) -tau_des_y(j)));
                    end
                    
%                     w_ij = 1;
%                         ux(i) = ux(i) + A(i,j)*w_ij*((x(i) - x(j)) - (tau_des_x(i) -tau_des_x(j)));
%                         uy(i)= uy(i) + A(i,j)*w_ij*((y(i) - y(j)) - (tau_des_y(i) -tau_des_y(j)));
                end
            end
%             v_max = 5;
%             w_max = 90;
% 
%             R_i = [cos(yaw(i)) -sin(yaw(i));
%             sin(yaw(i)) cos(yaw(i))];
%             invR_i = inv(R_i);
%             vel = -invR_i* [ux(i);uy(i)];
%             vel(1) = vel(1)/abs(vel(1))*min(abs(vel(1)),v_max);
%             vel(2) = vel(2)/abs(vel(2))*min(abs(vel(2)),deg2rad(w_max));
%             dx = vel(1)*cos(yaw(i));
%             dy = vel(1)*sin(yaw(i));
%             d_yaw = vel(2);
%             x(i) = x(i) + dt*dx;
%             y(i)=  y(i) + dt*dy;
%             yaw(i) = yaw(i) + dt*d_yaw;
%             yaw(i) = atan2(sin(yaw(i)),cos(yaw(i)));
              ux_max = 10;
              uy_max = 10;
              ux_in = (ux(i)/abs(ux(i)))*min(abs(ux(i)),ux_max);
              uy_in = (uy(i)/abs(uy(i)))*min(abs(uy(i)),uy_max);
              x(i) = x(i) - dt*ux_in;
              y(i) = y(i) - dt*uy_in;
        end
        err_x = L*x - L*tau_des_x;
        err_y = L*y - L*tau_des_y;
        err = sum(sqrt(err_x.^2 + err_y.^2));
        hold off
        for i = 1:N

            scatter(x(i),y(i),'o','filled')
            hold on
            vx(i) = 0.5*cos(yaw(i));
            vy(i) = 0.5*sin(yaw(i));
            quiver(x(i),y(i),vx(i),vy(i));
            axis([-2 9 -5 5]);
        end
        pause(0.04)
    end

end







