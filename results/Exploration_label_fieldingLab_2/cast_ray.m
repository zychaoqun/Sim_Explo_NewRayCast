function [ points ] = cast_ray(start_points, end_points,map_size, Resolution,map)
%CAST_RAY Summary of this function goes here
%   Detailed explanation goes here
dx=abs(end_points(1)-start_points(1)); dy=abs(end_points(2)-start_points(2));
x=start_points(1);
y=start_points(2);
error=dx-dy;
coll_flag=0;
coll_size=5;

if end_points(1)>start_points(1)
    x_inc=Resolution ;
else
    x_inc=-Resolution;
end
if end_points(2)>start_points(2)
    y_inc=Resolution;
else
    y_inc=-Resolution;
end

dx=dx*2;
dy=dy*2;

points=zeros(1000,2);
n=1;
while x>=1 && x<map_size(2) && y>=1 &&y<map_size(1)
    if x==end_points(1) && y==end_points(2)
        break    
    end
    
    if map(y,x)~=0&&coll_flag>0
        break
    elseif map(y,x)==0&&coll_flag<coll_size
        coll_flag=coll_flag+1;
    end
    
    points(n,:)=[x, y];
    n=n+1;
    
    if map(y,x)==0&&coll_flag==coll_size
        break
    end
   
    if error>0
        x=x+x_inc;
        error=error-dy;
    else
        y=y+y_inc;
        error=error+dx;
    end
    
end

points(n:end,:) = [];
end
