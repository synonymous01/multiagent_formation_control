loc1 = [0, 0];
loc2 = [0.45, 0];
loc3 = [0.9, 0];
IMPOSTOR = [(loc1(1) + loc2(1) + loc3(1)) / 3, (loc1(2) + loc2(2) + loc3(2)) / 3];
% loc7 = [3, 0];
% loc8 = [3.5, 0];
locs = [loc1;loc2;loc3;IMPOSTOR];
 dists = zeros(4,4);
loc_set = [
    0.5*sqrt(3), 1.5;
    sqrt(3), 0;
    0, 0;
    0.5*sqrt(3), 0.5
];
% D = [0, norm(locs_set(1,:) - locs_set(2,:)), norm(locs_set(1,:) - locs_set(3,:));
%      norm(locs_set(2,:) - locs_set(1,:)), 0, norm(locs_set(2,:) - locs_set(3,:));
%      norm(locs_set(3,:) - locs_set(1,:)), norm(locs_set(3,:) - locs_set(2,:)), 0];
vx = zeros(1,4);
yaw = zeros(1,4);
goal_loc = [
    0, 3.75;
    0.25, 4;
    3.75, 4;
    4, 3.75;
    4, 0.25;
    3.75, 0;
    0, 0
    ];

vinit = zeros(1,4);
omega = zeros(1,4);
v_min = 0.1;

vx(4) = 0.4;
omega(4) = 0.25;

err_bots = zeros(4,2);



    for i = 1:4
        for j = 1:4
            if i == j
                continue;
            end
            dists(i,j) = norm(locs(i, :) - locs(j,:));
        end
    end

for i = 1:3
    err_bots(i,:) = [0 0];
    % if i == 1
    %     k = 4;
    % else
    %     k = 3;
    % end
    for j = 1:4
        err_bots(i,:) = err_bots(i,:) + ((locs(i,:) - locs(j ,:)) - (loc_set(i,:) - loc_set(j, :)));
    end
    err_bots(i,:) = -1 * err_bots(i,:);
end
count = 1;

err_bots(4,1) = -5.5*(locs(4,1) - goal_loc(count,1));
err_bots(4,2) = -5*(locs(4,2) - goal_loc(count,2));

tol = 0.3;
v_max = 1;
dt = 0.05;



while 1
    for i = 1:3
        a = err_bots(i,:);
        if norm(a) > tol
            vx(i) = cos(yaw(i))* a(1) + sin(yaw(i)) * a(2);
            if abs(vx(i)) > v_max
                vx(i) = sign(vx(i)) * v_max;
            elseif abs(vx(i)) < v_min
                vx(i) = 0;
            end
            omega(i) = -sin(yaw(i)) * a(1) + cos(yaw(i))*a(2);
        else
            vx(i) = 0;
            omega(i) = 0; 
        end

    end
    
    a = err_bots(4,:);
    if norm(locs(4,:) - goal_loc(count,:)) > tol
        vx(4) = cos(yaw(4)) * a(1) + sin(yaw(4)) * a(2);
        if abs(vx(4)) > v_max
            vx(4) = sign(vx(4)) * v_max;
        end
        omega(4) = -sin(yaw(4)) * a(1) + cos(yaw(i)) * a(2);
    else
        count = count + 1
        count = mod(count, 7) + 1
    end

    for i = 1:4
        yaw(i) = yaw(i) + dt*omega(i);
        locs(i,1) = locs(i,1) + dt*vx(i)*cos(yaw(i));
        locs(i,2) = locs(i,2) + dt*vx(i)*sin(yaw(i));
    end 
    % locs(4,1) = ((locs(1,1) + locs(2,1) + locs(3,1)) / 3) + dt*vx(4)*cos(yaw(4));
    % locs(4,2) = ((locs(1,2) + locs(2,2) + locs(3,2)) / 3) + dt*vx(4)*sin(yaw(4));

    % locs(4,1) = goal_loc(count,1);
    % locs(4,2) = goal_loc(count, 2);

for i = 1:3
    err_bots(i,:) = [0 0];
    % if i == 1
    %     k = 4;
    % else
    %     k = 3;
    % end
    for j = 1:4
        if i == j
            continue
        end
        err_bots(i,:) = err_bots(i,:) + ((locs(i,:) - locs(j ,:)) - (loc_set(i,:) - loc_set(j, :)));
    end
    err_bots(i,:) = -1 * err_bots(i,:);
end
err_bots(4,1) = -5.5*(locs(4,1) - goal_loc(count,1));
err_bots(4,2) = -5*(locs(4,2) - goal_loc(count,2));

 
% clf;
    plot(locs(1, 1), locs(1, 2), 'b.', MarkerSize = 10)
    hold on
    plot(locs(2,1), locs(2,2), 'r.', MarkerSize = 10)
    plot(locs(3,1), locs(3,2), 'g.', MarkerSize = 10)
    plot(locs(4,1), locs(4,2), 'k.', MarkerSize = 10)
    axis([-10 10 -10 10])
    % if vx == vinit
    %     break
    % end
    
    pause(0.01)
end