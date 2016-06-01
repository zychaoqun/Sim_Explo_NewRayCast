function [ map_out ] = InverseSensorModel( RoboPosi, SensorRange, OccupancyMap_in ,Resolution, map)
%INVERSESENSORMODEL Summary of this function goes here
%   Detailed explanation goes here

map_size=size(OccupancyMap_in);
sensor_angle_inc=deg2rad(0.6);
sensor_angle_range=2*pi;

start_angle=deg2rad(RoboPosi(3))-sensor_angle_range/2;
end_angle=start_angle+sensor_angle_range;

known=zeros(500000,2);
n=1;

for angle=start_angle:sensor_angle_inc:end_angle
    ray_end=round(RoboPosi(1:2)'+SensorRange*[cos(angle),sin(angle)]);
    [point]=cast_ray(RoboPosi(1:2)', ray_end,map_size, Resolution,map);
    [mp,~]=size(point);
    known(n:n+mp-1,:)=point;
    n=n+mp;
end

known=unique(known(known(:,1)~=0,:),'rows');

map_out=OccupancyMap_in;
for i=1:size(known)
map_out(known(i,2),known(i,1))=map(known(i,2),known(i,1));
end
% map_out(known(:,2),known(:,1))=map(known(:,2),known(:,1));

end

