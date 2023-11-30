loc1 = [0, 0];
loc2 = [0.5, 0];
loc3 = [1, 0];
loc4 = [1.5, 0];
locs = [loc1;loc2;loc3;loc4];
loc_set = [0, 2;
           2, 2;
           2, 0;
           0, 0];
dists = [0, norm(loc1 - loc2), norm(loc1 - loc3), norm(loc1 - loc4);
       norm(loc2 - loc1), 0, norm(loc2 - loc3), norm(loc2 - loc4);
       norm(loc3 - loc1), norm(loc3 - loc2), 0, norm(loc3 - loc4);
       norm(loc4 - loc1), norm(loc4 - loc2), norm(loc4 - loc3), 0];

vx = [0, 0, 0, 0];
yaw = [0, 0, 0, 0];
vinit = vx;
omega = [0, 0, 0, 0];
err_bot_1 = -1*(((locs(1,:) - locs(2,:)) - (loc_set(1,:) - loc_set(2,:))) + ((locs(1,:) - locs(4,:)) - (loc_set(1,:) - loc_set(4,:))));
err_bot_2 = -1*(((locs(2,:) - locs(1,:)) - (loc_set(2,:) - loc_set(1,:))) + ((locs(2,:) - locs(3,:)) - (loc_set(2,:) - loc_set(3,:))));
err_bot_3 = -1*(((locs(3,:) - locs(2,:)) - (loc_set(3,:) - loc_set(2,:))) + ((locs(3,:) - locs(4,:)) - (loc_set(3,:) - loc_set(4,:))));
err_bot_4 = -1*(((locs(4,:) - locs(1,:)) - (loc_set(4,:) - loc_set(1,:))) + ((locs(4,:) - locs(3,:)) - (loc_set(4,:) - loc_set(3,:))));
err_bots = [err_bot_1; err_bot_2; err_bot_3; err_bot_4];

tol = 0.01;
v_max = 0.4;
dt = 0.05;
count = 0;

while 1
    count = count + 1;

    for i = 1:4
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

    for i = 1:4
        for j = 1:4
            if i == j
                continue;
            end
            dists(i,j) = norm(locs(i, :) - locs(j,:));
        end
    end 

    
    err_bots(1,:) = -1*(((locs(1,:) - locs(2,:))  - (loc_set(1,:) - loc_set(2,:))) + ((locs(1,:) - locs(3,:)) - (loc_set(1,:) - loc_set(3,:))));
    err_bots(2,:) = -1*(((locs(2,:) - locs(1,:))  - (loc_set(2,:) - loc_set(1,:))) + ((locs(2,:) - locs(3,:)) - (loc_set(2,:) - loc_set(3,:))));
    err_bots(3,:) = -1*(((locs(3,:) - locs(2,:))  - (loc_set(3,:) - loc_set(2,:))) + ((locs(3,:) - locs(1,:)) - (loc_set(3,:) - loc_set(1,:))));
    err_bots(4,:) = -1*(((locs(4,:) - locs(1,:)) - (loc_set(4,:) - loc_set(1,:))) + ((locs(4,:) - locs(3,:)) - (loc_set(4,:) - loc_set(3,:))));
    
    wide = 20;
    % clf;
    plot(locs(1, 1), locs(1, 2), 'b.', MarkerSize = wide)
    hold on
    plot(locs(2,1), locs(2,2), 'r.', MarkerSize = wide)
    plot(locs(3,1), locs(3,2), 'g.', MarkerSize= wide)
    plot(locs(4,1), locs(4,2), 'y.', MarkerSize= wide)

    if vx == vinit
        break
    end
    for i = 1:4
        for j = 1:4
            if i == j
                continue
            end

            if dists(i, j) < 0.35
                fprintf('collision detected between %i and %i', i, j)
            end
        end
    end


    pause(0.01)
end