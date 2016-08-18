function [ map_out, cur_free_xy ] = InverseSensorModel( RoboPosi, SensorRange, OccupancyMap_in ,Resolution, map )
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
% cur_free_xy = ones(size(map))*127;
counter = 1;
cur_free_xy = zeros(size(map,1)*size(map,2),2);
for i=1:size(known)
    map_out(known(i,2),known(i,1))=map(known(i,2),known(i,1));
    if map(known(i,2),known(i,1)) == 255
        cur_free_xy(counter,:) = [known(i,2) known(i,1)];
        counter = counter + 1;
    end
end
cur_free_xy(counter:end,:) = [];

end

