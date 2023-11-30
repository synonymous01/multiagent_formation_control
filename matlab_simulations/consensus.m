loc1=[0,0,0];
loc2=[2,2,0];
loc3=[-4,-4,0];
%loc4=[-3,-5,0];
tol=0.1;
dist1=norm(loc1(1:2)+loc1(1:2)-loc2(1:2)-loc3(1:2));
dist2=norm(loc2(1:2)+loc2(1:2)-loc1(1:2)-loc3(1:2));
dist3=norm(loc3(1:2)+loc3(1:2)-loc1(1:2)-loc2(1:2));
v_max=0.4;
K = [-1, -1];
u1 = [0,0];
u2 = [0,0];
u3=[0,0];
dt=0.05;
while (dist1>tol || dist2>tol || dist3>tol) 
    dist1=norm(loc1(1:2)+loc1(1:2)-loc2(1:2)-loc3(1:2));
    dist2=norm(loc2(1:2)+loc2(1:2)-loc1(1:2)-loc3(1:2));
    dist3=norm(loc3(1:2)+loc3(1:2)-loc1(1:2)-loc2(1:2));
    e1=loc1+loc1-loc2-loc3;
    e2=loc2+loc2-loc3-loc1;
    e3=loc3+loc3-loc2-loc1;
    u1=K.*e1(1:2);
    u2=K.*e2(1:2);
    u3=K.*e3(1:2);
    vx1=cos(loc1(3))*u1(1)+sin(loc1(3))*u1(2);
    if abs(vx1)>v_max
      vx1=sign(vx1)*v_max;
    end
    vx2=cos(loc2(3))*u2(1)+sin(loc2(3))*u2(2);
    if abs(vx2)>v_max
      vx2=sign(vx2)*v_max;
    end
    vx3=cos(loc3(3))*u3(1)+sin(loc3(3))*u3(2);
    if abs(vx3)>v_max
      vx3=sign(vx3)*v_max;
    end
    w1=-sin(loc1(3))*u1(1)+cos(loc1(3))*u1(2);
    w1=atan2(sin(w1),cos(w1));
    w2=-sin(loc2(3))*u2(1)+cos(loc2(3))*u2(2);
    w2=atan2(sin(w2),cos(w2));
    w3=-sin(loc3(3))*u3(1)+cos(loc3(3))*u3(2);
    w3=atan2(sin(w3),cos(w3));
    loc1(1)=loc1(1)+dt*vx1*cos(loc1(3));
    loc1(2)=loc1(2)+dt*vx1*sin(loc1(3));
    loc1(3)=loc1(3)+dt*w1;
    loc2(1)=loc2(1)+dt*vx2*cos(loc2(3));
    loc2(2)=loc2(2)+dt*vx2*sin(loc2(3));
    loc2(3)=loc2(3)+dt*w2;
    loc3(1)=loc3(1)+dt*vx3*cos(loc3(3));
    loc3(2)=loc3(2)+dt*vx3*sin(loc3(3));
    loc3(3)=loc3(3)+dt*w3;
    %loc1=loc1(1:2)+dt*u1;
    %loc2=loc2(1:2)+dt*u2;
    %loc3=loc3(1:2)+dt*u3;
    plot(loc1(1),loc1(2), 'r.')
    hold on
    plot(loc2(1),loc2(2), 'b.')
    hold on
    plot(loc3(1),loc3(2), 'g.')
    hold on
    xlabel('xposition')
    ylabel('yposition')
    pause(0.01)
end
