
NumOfRun=1;
clearvars -except NumOfRun; close all; clc;
time = clock;
% FolderName = strcat(['Explo_Sobol_GP_10_' num2str(NumOfRun) 'thRun_' num2str(time(1)) '_' num2str(time(2)) '_' num2str(time(3)) '_' num2str(time(4)) '' num2str(time(5))]);
% mkdir(FolderName); SaveImgPath = [pwd '/' FolderName '/'];
disp('Load data...');
load('RobotInitSet.mat');

% setup map
map=map_setup('map.png');
maxX=size(map,2);
maxY=size(map,1);
Resolution=1;
[t1, t2] = meshgrid(1:maxX, 1:maxY);
t = [t1(:) t2(:)];
Map_OccuPts=t(map==0,:);
OP_MAP=ones(maxY,maxX)*127;

% setup robot
Po_id=randi([1 15], 1, 1);
RobotInit(1:2,1) = RobotInitSet(1:2,Po_id);
RobotInit(3,1)=90;
RoboPosi = round(RobotInit);
SensorRange = 80;
SearchRange = SensorRange/5;
RobotStep = SensorRange/2;
RobotStep_long=SensorRange;

% setup GP Regression


[ OP_MAP, cur_free ] = InverseSensorModel( RoboPosi, SensorRange, OP_MAP, Resolution, map); 
Entropy_1 = length(find(OP_MAP==127));


tic
RoboPosi = [118 135 90]'; % [x  y  yaw_deg]
[ OP_MAP, cur_free ] = InverseSensorModel( RoboPosi, SensorRange, OP_MAP, Resolution, map); 
Entropy_2 = length(find(OP_MAP==127))





toc
figure(2); imshow(OP_MAP,[0 255]);



 
 

 
 
 
 



 
 