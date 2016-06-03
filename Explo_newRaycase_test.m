
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
RndSet = sobolset(2);
RndSet = ceil(RndSet(100:100000,:).*2*SensorRange);
RndPat = zeros(SensorRange*2,SensorRange*2,'uint32');
for i=1:length(RndSet)
    if(RndPat(RndSet(i,1),RndSet(i,2)) == 0)
        RndPat(RndSet(i,1),RndSet(i,2)) = i;
    end
end
% setup GP Regression


[ OP_MAP, cur_free ] = InverseSensorModel( RoboPosi, SensorRange, OP_MAP, Resolution, map); 

tic
RoboPosi = [118 135 90]'; % [x  y  yaw_deg]
[ OP_MAP, cur_free ] = InverseSensorModel( RoboPosi, SensorRange, OP_MAP, Resolution, map); 


SelPat = zeros(SensorRange*2,SensorRange*2,'uint32');
offset = [RoboPosi(1)-SensorRange RoboPosi(2)-SensorRange ];
for i=1:length(cur_free)
    SelPat(cur_free(i,1)-offset(1),cur_free(i,2)-offset(2)) = 1;
end

SelPts = RndPat.*SelPat;
figure(3); imshow(SelPts,[0 1]);

SelPts(SelPts==0) = max(SelPts(:)) + 1;
% SelPts = SelPts - min(SelPts(:)) + 1;

[I,J] = find(SelPts>-inf,10,'last')

toc
figure(2); imshow(OP_MAP,[0 255]);



 
 

 
 
 
 



 
 