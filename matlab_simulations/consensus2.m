loc1=[0,0];
loc2=[1,1];
loc3=[1,-2];
loc4=[-3,2];
loc5=[4,-6];
locs=[loc1;loc2;loc3;loc4;loc5];
yaw=[0,0,0,0,0];
vx=[0,0,0,0,0];
vinit=vx;
w=[pi,-pi,1.5*pi,0.5*pi,0];
l=length(yaw);
err=zeros(l,2);
dists=zeros(1,l);
u=zeros(l,2);
for i=1:l
    p=locs(i,:); 
    arr=locs;
    arr(i,:)=[];
    j=length(arr);
    point=p+p;
    for k=1:j
        point=point-arr(k,:);
    end
    err(i,:)=-point;
    dists(i)=norm(point);
end
tol=0.1;
tolv=0.1*ones(1,l);
v_max=0.4;
dt=0.05;
while 1
    convergence=false;
    for i=1:l
        a=err(i,:);
        vx(i)=cos(yaw(i))*a(1)+sin(yaw(i))*a(2);
        if abs(vx(i))>v_max
            vx(i)=sign(vx(i))*v_max;
        end
        w(i)=-sin(yaw(i))*a(1)+cos(yaw(i))*a(2);
        other_locs=locs;
        other_locs(i,:)=[];
        a=locs(i,:);
        l2=l-1;
        for k=1:l2
            if norm(a-other_locs(k,:))<0.15
                vx(i)=0;
                w(i)=0;
                convergence=true;
                conv_point=a;
            end
        end
        locs(i,1)=locs(i,1)+dt*vx(i)*cos(yaw(i));
        locs(i,2)=locs(i,2)+dt*vx(i)*sin(yaw(i));
        yaw(i)=yaw(i)+dt*w(i);
        %locs(i,1)=locs(i,1)+dt*err(i,1);
        %locs(i,2)=locs(i,2)+dt*err(i,2);
    end
    for i=1:l
        p=locs(i,:);
        arr=locs;
        arr(i,:)=[];
        j=length(arr);
        point=p+p;
        for k=1:j
            point=point-arr(k,:);
        end
        err(i,:)=-point;
        if convergence==true
            err(i,:)=-p+conv_point;
        end
        dists(i)=norm(point);
    end
    r='r.';
    b='b.';
    g='g.';
    y='y.';
    p='k.';
    for i=1:l
        a=locs(i,:);
        if i==1
            c=r;
        elseif i==2
            c=b;
        elseif i==3
            c=g;
        elseif i==4
            c=y;
        elseif i==5
            c=p;
        end
        plot(a(1),a(2),c)
        hold on
        xlabel('xposition')
        ylabel('yposition')
    end
    if vx==vinit
        break
    end
    pause(0.01)
end
