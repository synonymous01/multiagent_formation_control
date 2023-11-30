clc
clear all 
%close all
locs=[[0,0];
       [0.45,0];
       [0.9,0]];

des=[[0,0];
       [2,0];
       [1,sqrt(3)]];

L=[[2,-1,-1];
   [-1,2,-1];
   [-1,-1,2]]


yaw=[0,0,0];
dt=0.1;

x=[0,0.45,0.9];
y=[0,0,0];
marker_size=500;
% figure;

hold off
scatter(x(1), y(1),marker_size,"red",'filled');
hold on
scatter(x(2), y(2),marker_size,"blue",'filled');
scatter(x(3), y(3),marker_size,"green",'filled');
axis([-3 3 -2 2.5])
pause(0.1)
hold off


edge_dis=2;
while 1

    dist1=pdist2(locs(1,:), locs(2,:));
    dist2=pdist2(locs(2,:), locs(3,:));
    dist3=pdist2(locs(3,:), locs(1,:));

    
    scatter(x(1), y(1),marker_size,"red",'filled');
    hold on
    scatter(x(2), y(2),marker_size,"blue",'filled');
    scatter(x(3), y(3),marker_size,"green",'filled');
    axis([-3 3 -2 2.5])
    hold off
    pause(1)
    weights=[,dist2+dist1-1,dist3+dist2-1]
    u=-L*(locs-des);
    %u=-L*locs;
    for k=1:3
        u(k,:)=u(k,:).*weights(k);  
    
    end
    vx=[0,0,0];
    w=[0,0,0];
    for i=1:3
        vx(i)=cos(yaw(i))*u(i,1)+sin(yaw(i))*u(i,2);
        if abs(vx(i))>1
            vx(i)=sign(vx(i))*1;
        end
        w(i)=-sin(yaw(i))*u(i,1)+cos(yaw(i))*u(i,2);

    end


    yaw=yaw+w.*dt;
    % disp('next')
    % disp(vx)
    % disp(w)
    locs(:, 1)=locs(:,1)+vx'.*cos(yaw').*dt;
    locs(:, 2)=locs(:,2)+vx'.*sin(yaw').*dt;

    %disp(locs)

    x= locs(:, 1);
    y= locs(:, 2);

    
    if (abs(dist1-edge_dis)<0.1 && abs(dist2-edge_dis)<0.1 && abs(dist3-edge_dis)<0.1)
        hold off
        scatter(x(1), y(1),marker_size,"red",'filled');
        hold on
        scatter(x(2), y(2),marker_size,"blue",'filled');
        scatter(x(3), y(3),marker_size,"green",'filled');
        axis([-3 3 -2 2.5])
        a=[x',x(1)];
        b=[y',y(1)];
        %disp(size(a))
        plot(a', b', 'b-');
        break
    end

    
end


plot(x,y,'b-')




