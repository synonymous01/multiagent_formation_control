clear all; close all;
loc1 = [0.5, 0];
loc2 = [1, 0];
loc3 = [0, 0];
locs = [loc1;loc2;loc3];
loc_set = [1, 1*sqrt(3);
           2, 0;
           0, 0];
 dists = [0, norm(loc1 - loc2), norm(loc1 - loc3);
        norm(loc2 - loc1), 0, norm(loc2 - loc3);
        norm(loc3 - loc1), norm(loc3 - loc2), 0];

l = [0, norm(loc_set(1,:) - loc_set(2,:)), norm(loc_set(1,:) - loc_set(3,:));
     norm(loc_set(2,:) - loc_set(1,:)), 0, norm(loc_set(2,:) - loc_set(3,:));
     norm(loc_set(3,:) - loc_set(1,:)), norm(loc_set(3,:) - loc_set(2,:)), 0];
vx = [0, 0, 0];
yaw = [0, 0, 0];
vinit = vx;
omega = [0, 0, 0];
delta = 0.4;
err_bots = zeros(3, 2);
weights = zeros(3);

for i = 1:3
    for j = 1:3
        if i == j
            continue
        end
        weights(i,j) = 1;
        % weights(i,j) = (2*(dists(i,j) - delta) - (dists(i,j) - l(i,j)))/(dists(i,j) - delta - (l(i,j) - dists(i,j)));
        % weights(i,j) = (2*(dists(i,j) - delta) - norm(loc_set(i,:) - loc_set(j,:) - locs(i,:) - locs(j,:)))/(dists(i,j) - delta - norm(loc_set(i,:) - loc_set(j,:) - locs(i,:) - locs(j,:)))^2;
    end
end


neighbours = [2 3; 1 3; 2 1];

for i = 1:3
    err_bots(i,:) = [0 0];
    for j = 1:2
        err_bots(i,:) = err_bots(i,:) + weights(i,j) * ((locs(i,:) - locs(neighbours(i,j) ,:)) - (loc_set(i,:) - loc_set(neighbours(i,j), :)));
    end
    err_bots(i,:) = -1 * err_bots(i,:);
end

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
            if dists(i,j) < 0.3
                fprintf('collision detected between robot %i and %i', i, j)
            end
        end
    end 

    for i = 1:3
    for j = 1:3
        if i == j
            continue
        end
        weights(i,j) = 1;
        % weights(i,j) = (2*(dists(i,j) - delta) - (dists(i,j) - l(i,j)))/(dists(i,j) - delta - (l(i,j) - dists(i,j)));
        % weights(i,j) = (2*(dists(i,j) - delta) - norm(loc_set(i,:) - loc_set(j,:) - locs(i,:) - locs(j,:)))/(dists(i,j) - delta - norm(loc_set(i,:) - loc_set(j,:) - locs(i,:) - locs(j,:)))^2;
    end
    end

for i = 1:3
    err_bots(i,:) = [0 0];
    for j = 1:2
        err_bots(i,:) = err_bots(i,:) + weights(i,j) * ((locs(i,:) - locs(neighbours(i,j) ,:)) - (loc_set(i,:) - loc_set(neighbours(i,j), :)));
    end
    err_bots(i,:) = -1 * err_bots(i,:);
end
% err_bots(1,:) = -1*(((locs(1,:) - locs(2,:)) - (loc_set(1,:) - loc_set(2,:))) + ((locs(1,:) - locs(3,:)) - (loc_set(1,:) - loc_set(3,:))));
% err_bots(2,:) = -1*(((locs(2,:) - locs(1,:)) - (loc_set(2,:) - loc_set(1,:))) + ((locs(2,:) - locs(3,:)) - (loc_set(2,:) - loc_set(3,:))));
% err_bots(3,:) = -1*(((locs(3,:) - locs(2,:)) - (loc_set(3,:) - loc_set(2,:))) + ((locs(3,:) - locs(1,:)) - (loc_set(3,:) - loc_set(1,:))));
    
clf;
    plot(locs(1, 1), locs(1, 2), 'b.', MarkerSize = 20)
    hold on
    plot(locs(2,1), locs(2,2), 'r.', MarkerSize = 20)
    plot(locs(3,1), locs(3,2), 'g.', MarkerSize = 20)
    % axis([-5 5 -5 5])

    if vx == vinit
        break
    end
    pause(0.01)
end