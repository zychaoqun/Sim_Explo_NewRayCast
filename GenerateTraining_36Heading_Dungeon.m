close all;
clear all;
TrainFolderDungeon = 'TrainImg_Dungeon_5663';
TestFolderDungeon = 'TestImg_Dungeon_5663';
mkdir(TestFolderDungeon); TestImgPath = [pwd '/' TestFolderDungeon '/'];
mkdir(TrainFolderDungeon); TrainImgPath = [pwd '/' TrainFolderDungeon '/'];
disp('Load map names...');
pngFolder = 'DungeonMaps_5663';
pnglist = dir([pngFolder '/*.png']);

labelStatic = zeros(36,1);

offset = 0;
for NumOfRun=offset+1:offset+length(pnglist)
close all;
% setup map
img = rgb2gray(imread([pngFolder '/' pnglist(NumOfRun).name]));
[Y,X]=find(img==203);
RobotInit(1:2,1) = [mean(X); mean(Y)];
figure(1000); imshow(img,[0 255]);

map=map_setup([pngFolder '/' pnglist(NumOfRun).name]);

maxX=size(map,2);
maxY=size(map,1);
Resolution=1;
[t1, t2] = meshgrid(1:maxX, 1:maxY);
t = [t1(:) t2(:)];
Map_OccuPts=t(map==0,:);
OP_MAP=ones(maxY,maxX)*127;

% setup robot
% Po_id=randi([1 15], 1, 1);
% Po_id = NumOfRun-offset;
% RobotInit(1:2,1) = RobotInitSet_cur(1:2,Po_id);
% RobotInit(1:2,1) = ginput(1)';

RobotInit(3,1)=90;
RoboPosi = round(RobotInit);
SensorRange = 80;
% SearchRange = SensorRange/5;
RobotStep = SensorRange/2;
RobotStep_long=SensorRange;
Step_Counter = 1;
max_MI = 1000;

% setup GP Regression

% Start the run
while(max_MI > 250)

    trainFileID = fopen('TrainImg_Dungeon_5663.txt','a');
    testFileID = fopen('TestImg_Dungeon_5663.txt','a');

tic
[ OP_MAP, cur_free ] = InverseSensorModel( RoboPosi, SensorRange, OP_MAP, Resolution, map); 

% Select Candidates
sensor_angle_inc = 10;
counter = 1;
OP_MAP_1 = OP_MAP; 

    for angle=0:sensor_angle_inc:360-sensor_angle_inc
        ray_end=round(RoboPosi(1:2)'+RobotStep*[cosd(angle),sind(angle)]);
        [point]=cast_ray(RoboPosi(1:2)', ray_end,size(map), Resolution,map);
        occu_count = 0;
        for i = 1:length(point)
            if (map(point(i,2),point(i,1)) == 255 ) 
                occu_count = occu_count + 0;
            else
                occu_count = occu_count + 1;
                break;
            end
        end
        if occu_count == 0 
            Candidate(counter,:) = [point(i,1),point(i,2)];
            OP_MAP_1(point(i,2),point(i,1)) = 100;
            counter = counter + 1;
        end
    end

% Evaluate MI
MI = zeros(length(Candidate),1);
parfor i = 1:length(Candidate)
    OP_MAP_MI= OP_MAP;
    RoboPosi = [Candidate(i,:) 90]';  
    [ OP_MAP_MI, cur_free ] = InverseSensorModel( RoboPosi, SensorRange, OP_MAP_MI, Resolution, map); 
    MI(i) = length(find(OP_MAP==127)) - length(find(OP_MAP_MI==127));
end

% Pick the Next Step
[max_MI,np_idx] = max(MI);
if(max_MI < 250)
    continue;
end

% Save the image
local_img = ones(2*SensorRange,2*SensorRange,'uint8').*127;

row_lb = min(RoboPosi(2)-1,SensorRange-1);
row_ub = min(size(map,1)-RoboPosi(2),SensorRange);

col_lb = min(RoboPosi(1)-1,SensorRange-1);
col_ub = min(size(map,2)-RoboPosi(1),SensorRange);

im_row = RoboPosi(2);
im_col = RoboPosi(1);

trainORtest = rand<0.95;  % 1: train

if(trainORtest)
    disp('train sample +1');
    if ( (row_lb + row_ub >= 2 * SensorRange ) &&  (col_lb + col_ub >= 2 * SensorRange ))
        local_img = OP_MAP(im_row-SensorRange+1:im_row+SensorRange,im_col-SensorRange+1:im_col+SensorRange);
        imwrite(local_img, strcat([TrainImgPath num2str(NumOfRun) '_'  num2str(Step_Counter) '|' num2str(np_idx-1) ]) , 'jpg');
        fprintf(trainFileID,'%d_%d|%d %d\n', NumOfRun, Step_Counter, np_idx-1, np_idx-1);
    else
        local_img(SensorRange-row_lb:SensorRange+row_ub , SensorRange-col_lb:SensorRange+col_ub) = OP_MAP(im_row-row_lb:im_row+row_ub , im_col-col_lb:im_col+col_ub);
        imwrite(local_img, strcat([TrainImgPath num2str(NumOfRun) '_'  num2str(Step_Counter) '|' num2str(np_idx-1) ]) , 'jpg');
        fprintf(trainFileID,'%d_%d|%d %d\n', NumOfRun, Step_Counter, np_idx-1, np_idx-1);
        disp('robot getting too close to boundaries');
    end
else
    disp('test sample +1');
    if ( (row_lb + row_ub >= 2 * SensorRange ) &&  (col_lb + col_ub >= 2 * SensorRange ))
        local_img = OP_MAP(im_row-SensorRange+1:im_row+SensorRange,im_col-SensorRange+1:im_col+SensorRange);
        imwrite(local_img, strcat([TestImgPath num2str(NumOfRun) '_'  num2str(Step_Counter) '|' num2str(np_idx-1) ]) , 'jpg');
        fprintf(testFileID,'%d_%d|%d %d\n', NumOfRun, Step_Counter, np_idx-1, np_idx-1);
    else
        local_img(SensorRange-row_lb:SensorRange+row_ub , SensorRange-col_lb:SensorRange+col_ub) = OP_MAP(im_row-row_lb:im_row+row_ub , im_col-col_lb:im_col+col_ub);
        imwrite(local_img, strcat([TestImgPath num2str(NumOfRun) '_'  num2str(Step_Counter) '|' num2str(np_idx-1) ]) , 'jpg');
        fprintf(testFileID,'%d_%d|%d %d\n', NumOfRun, Step_Counter, np_idx-1, np_idx-1);
        disp('robot getting too close to boundaries');
    end
end

% update statics about labels
labelStatic(np_idx) = labelStatic(np_idx) + 1;

% write to next step
RoboPosi = [Candidate(np_idx,:) 90]';
Step_Counter = Step_Counter + 1;

toc
fclose(trainFileID);
fclose(testFileID);
clear Candidate; 

end

% r_img = imrotate(local_img,-1,'bilinear','crop');
% figure(2); imshow(OP_MAP,[0 255]);

end

save('labelStatics_DungeonMaps_5663','labelStatic');
 

 
 
 
 



 
 