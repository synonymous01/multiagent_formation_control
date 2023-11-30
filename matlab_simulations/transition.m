clc
clear all 
close all
locs=[[0,0];
       [3,0];
       [5,0]
       [1,1]];

des1=[[0,0];
       [2,0];
       [2,2];
       [0,2]];



des2=[[0,0];
       [2,0];
       [1,sqrt(3)];
       [1,sqrt(3)/3]];

L=[[3,-1,-1,-1];
   [-1,3,-1,-1];
   [-1,-1,3,-1];
   [-1,-1,-1,3]];

des={des1,des2};

d1=[2,2,2*sqrt(2),2,2,2*sqrt(2),2,2,2*sqrt(2)];
d2=[2,2/sqrt(3),2,2,2,2/sqrt(3),2,2/sqrt(3),2];
d={d1,d2};

yaw=[0,0,0,0];
dt=0.1;

x=[0,3,5,1];
y=[0,0,0,1];

figure;
scatter(x(1), y(1),100,"red",'filled');
hold on
scatter(x(2), y(2),100,"blue",'filled');
scatter(x(3), y(3),100,"green",'filled');
scatter(x(4), y(4),100,"yellow",'filled');

axis([0 5 -1 4])
hold off
j=1;
while j<3
    

    while 1
    
    
        path=des{j};

        u=-L*(locs-path);
        vx=[0,0,0,0];
        w=[0,0,0,0];
        for i=1:length(w)
            vx(i)=cos(yaw(i))*u(i,1)+sin(yaw(i))*u(i,2);
            w(i)=-sin(yaw(i))*u(i,1)+cos(yaw(i))*u(i,2);
    
        end
    
        yaw=yaw+w.*dt;
        locs(:, 1)=locs(:,1)+vx'.*cos(yaw').*dt;
        locs(:, 2)=locs(:,2)+vx'.*sin(yaw').*dt;
    
        x= locs(:, 1);
        y= locs(:, 2);
        
    
        scatter(x(1), y(1),100,"red",'filled');
        hold on
        scatter(x(2), y(2),100,"blue",'filled');
        scatter(x(3), y(3),100,"green",'filled');
        scatter(x(4), y(4),100,"yellow",'filled');
            axis([0 5 -1 4])
            hold off
        
            pause(0.1)
        
            dist1=pdist2(locs(1,:), locs(2,:));
            dist2=pdist2(locs(1,:), locs(4,:));
            dist3=pdist2(locs(1,:), locs(3,:));
            dist4=pdist2(locs(2,:), locs(1,:));
            dist5=pdist2(locs(2,:), locs(3,:));
            dist6=pdist2(locs(2,:), locs(4,:));
            dist7=pdist2(locs(3,:), locs(2,:));
            dist8=pdist2(locs(3,:), locs(4,:));
            dist9=pdist2(locs(3,:), locs(1,:));
            dis=d{j};
        
            
            if (abs(dist1-dis(1))<0.1 && abs(dist2-dis(2))<0.1 && abs(dist3-dis(3))<0.1 && abs(dist4-dis(4))<0.1 && abs(dist5-dis(5))<0.1  && abs(dist6-dis(6))<0.1  && abs(dist7-dis(7))<0.1  && abs(dist8-dis(8))<0.1  && abs(dist9-dis(9))<0.1)
                hold on
        
                scatter(x(1), y(1),100,"red",'filled');
                scatter(x(2), y(2),100,"blue",'filled');
                scatter(x(3), y(3),100,"green",'filled');
                scatter(x(4), y(4),100,"yellow",'filled');
                axis([0 5 -1 4])
                x=[x',x(1)];
                y=[y',y(1)];
                plot(x, y, 'b-');
                break
            end

    
    end
plot(x,y,'b-')
j=j+1;

pause(2)

end






