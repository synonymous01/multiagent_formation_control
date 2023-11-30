clear all; close all;
loc1 = [0.5, 0];
loc2 = [1, 0];
loc3 = [0, 0];
locs = [loc1;loc2;loc3];
loc_set = [1, sqrt(3);
           2, 0;
           0, 0];
 dists = [0, norm(loc1 - loc2), norm(loc1 - loc3);
        norm(loc2 - loc1), 0, norm(loc2 - loc3);
        norm(loc3 - loc1), norm(loc3 - loc2), 0];

% D = [0, norm(locs_set(1,:) - locs_set(2,:)), norm(locs_set(1,:) - locs_set(3,:));
%      norm(locs_set(2,:) - locs_set(1,:)), 0, norm(locs_set(2,:) - locs_set(3,:));
%      norm(locs_set(3,:) - locs_set(1,:)), norm(locs_set(3,:) - locs_set(2,:)), 0];
vx = [0, 0, 0];
yaw = [0, 0, 0];
vinit = vx;
omega = [0, 0, 0];
err_bot_1 = -1*(((locs(1,:) - locs(2,:)) - (loc_set(1,:) - loc_set(2,:))) + ((locs(1,:) - locs(3,:)) - (loc_set(1,:) - loc_set(3,:))));
err_bot_2 = -1*(((locs(2,:) - locs(1,:)) - (loc_set(2,:) - loc_set(1,:))) + ((locs(2,:) - locs(3,:)) - (loc_set(2,:) - loc_set(3,:))));
err_bot_3 = -1*(((locs(3,:) - locs(2,:)) - (loc_set(3,:) - loc_set(2,:))) + ((locs(3,:) - locs(1,:)) - (loc_set(3,:) - loc_set(1,:))));

err_bots = [err_bot_1; err_bot_2; err_bot_3];

tol = 0.1;
v_max = 0.4;
dt = 0.05;
count = 0;

while 1
    count = count + 1;

    for i = 1:3
        a = err_bots(i,:);
        if norm(a) > tol
            vx(i) = cos(yaw(i))* a(1) + sin(yaw(i)) * a(2);
            if abs(vx(i)) > v_max
                vx(i) = sign(vx(i)) * v_max;
            end
            omega(i) = -sin(yaw(i)) * a(1) + cos(yaw(i))*a(2);
        else
            vx(i) = 0;
            omega(i) = 0; 
        end
        locs(i,1) = locs(i,1) + dt*vx(i)*cos(yaw(i));
        locs(i,2) = locs(i,2) + dt*vx(i)*sin(yaw(i));
        yaw(i) = yaw(i) + dt*omega(i);

    end

    for i = 1:3
        for j = 1:3
            if i == j
                continue;
            end
            dists(i,j) = norm(locs(i, :) - locs(j,:));
        end
    end 



err_bots(1,:) = -1*(((locs(1,:) - locs(2,:)) - (loc_set(1,:) - loc_set(2,:))) + ((locs(1,:) - locs(3,:)) - (loc_set(1,:) - loc_set(3,:))));
err_bots(2,:) = -1*(((locs(2,:) - locs(1,:)) - (loc_set(2,:) - loc_set(1,:))) + ((locs(2,:) - locs(3,:)) - (loc_set(2,:) - loc_set(3,:))));
err_bots(3,:) = -1*(((locs(3,:) - locs(2,:)) - (loc_set(3,:) - loc_set(2,:))) + ((locs(3,:) - locs(1,:)) - (loc_set(3,:) - loc_set(1,:))));
    
% clf;
    plot(locs(1, 1), locs(1, 2), 'b.', MarkerSize = 10)
    hold on
    plot(locs(2,1), locs(2,2), 'r.', MarkerSize = 10)
    plot(locs(3,1), locs(3,2), 'g.', MarkerSize = 10)

    if vx == vinit
        break
    end
    pause(0.01)
end