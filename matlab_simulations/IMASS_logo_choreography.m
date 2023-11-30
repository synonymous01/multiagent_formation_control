loc1 = [0, 0];
loc2 = [0.45, 0];
loc3 = [0.9, 0];
loc4 = [1.35, 0];
loc5 = [1.8, 0];
loc6 = [2.25, 0];
% loc7 = [3, 0];
% loc8 = [3.5, 0];
locs = [loc1;loc2;loc3;loc4;loc5;loc6];
loc_set_I = [
    0,0;
    1,1;
    0,4;
    1,3;
    2,4;
    2,0
];
loc_set_M = [
    0,0;
    1, 2.5;
    0,4;
    2, 2.5;
    3,4;
    3,0
];
loc_set_A = [
    0,0;
    1,2;
    2,4;
    2,2;
    3,2;
    4,0;
];
loc_set_S = [
    0, 0.5;
    0.5, 3;
    1.5, 3.5;
    1, 2;
    2, 1;
    1, 0
];

 dists = zeros(6,6);

% D = [0, norm(locs_set(1,:) - locs_set(2,:)), norm(locs_set(1,:) - locs_set(3,:));
%      norm(locs_set(2,:) - locs_set(1,:)), 0, norm(locs_set(2,:) - locs_set(3,:));
%      norm(locs_set(3,:) - locs_set(1,:)), norm(locs_set(3,:) - locs_set(2,:)), 0];
vx = zeros(1,6);
yaw = zeros(1,6);
vinit = zeros(1,6);
omega = zeros(1,6);
v_min = 0.1;

err_bots = zeros(3,2);

for k = 1:4
    if k == 1
        fprintf('making I\n')
        loc_set = 0.5 * loc_set_I;
    elseif k == 2
        fprintf('making M\n')
            loc_set = 0.5* loc_set_M;
    elseif k == 3
        fprintf('making A\n')
            loc_set = 0.5* loc_set_A;
    elseif k == 4
        fprintf('making S\n')
            loc_set = 0.5*loc_set_S;
    end

    for i = 1:6
        for j = 1:6
            if i == j
                continue;
            end
            dists(i,j) = norm(locs(i, :) - locs(j,:));
        end
    end

for i = 1:6
    err_bots(i,:) = [0 0];
    for j = 1:6
        err_bots(i,:) = err_bots(i,:) + ((locs(i,:) - locs(j ,:)) - (loc_set(i,:) - loc_set(j, :)));
    end
    err_bots(i,:) = -1 * err_bots(i,:);
end

tol = 0.2;
v_max = 0.4;
dt = 0.05;
count = 0;

while 1
    count = count + 1;

    for i = 1:6
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
        yaw(i) = yaw(i) + dt*omega(i);
        locs(i,1) = locs(i,1) + dt*vx(i)*cos(yaw(i));
        locs(i,2) = locs(i,2) + dt*vx(i)*sin(yaw(i));

    end

    for i = 1:6
        for j = 1:6
            if i == j
                continue;
            end
            dists(i,j) = norm(locs(i, :) - locs(j,:));
            if dists(i,j) < 0.35
                fprintf('collision b/w %i & %i\n', i, j)
            end
        end
    end 


for i = 1:6
    err_bots(i,:) = [0 0];
    for j = 1:6
        if i == j
            continue
        end
        err_bots(i,:) = err_bots(i,:) + ((locs(i,:) - locs(j ,:)) - (loc_set(i,:) - loc_set(j, :)));
    end
    err_bots(i,:) = -1 * err_bots(i,:);
end
    
clf;
    plot(locs(1, 1), locs(1, 2), 'b.', MarkerSize = 10)
    hold on
    plot(locs(2,1), locs(2,2), 'r.', MarkerSize = 10)
    plot(locs(3,1), locs(3,2), 'g.', MarkerSize = 10)
    plot(locs(4,1), locs(4,2), 'y.', MarkerSize = 10)
    plot(locs(5,1), locs(5,2), 'c.', MarkerSize = 10)
    plot(locs(6,1), locs(6,2), 'k.', MarkerSize = 10)

    if vx == vinit
        break
    end
    
    pause(0.01)
end
pause(1)
end