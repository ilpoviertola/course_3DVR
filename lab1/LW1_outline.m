% Your implementation should run by executing this m-file ("run LW1.m"), 
% but feel free to create additional files for your own functions
% Make sure it runs without errors after unziping
% This file is for guiding the work. You are free to make changes that suit
% you.

% Fill out the information below

% Group members: Petri Vainio, Ilpo Viertola, Henrik Lauronen
% Tasks Completed: 


%% Task A:  Apply transformation on point and visualize  [mandatory]

%1.
%create point cloud 
Points=pointCloud([0 0 3; 5 0 5; 2.5 2.5 0]);
% %visualize point cloud
pcshow(Points)
% 
% %2.
% %create transformation
% PointsMoved=pointCloud([]);
% 
% %combined rotation matrix
% syms y b a
% R = [
%     cos(b)*cos(y) sin(a)*sin(b)*cos(y)-cos(a)*sin(y) cos(a)*sin(b)*cos(y)+sin(a)*sin(y);
%     cos(b)*sin(y) sin(a)*sin(b)*sin(y)+cos(a)*sin(y) cos(a)*sin(b)*sin(y)-sin(a)*cos(y);
%     -sin(b) sin(a)*cos(b) cos(a)*cos(b)
%     ];
% 
% for i = 1 : Points.Count
%     point = Points.Location(i,:);
%     
% end
    
z = deg2rad(20);
y = deg2rad(50);
x = deg2rad(40);

Rx=[1 0 0;
    0 cos(x) -sin(x);
    0 sin(x) cos(x)];
Ry=[cos(y) 0 sin(y);
    0 1 0;
    -sin(y) 0 cos(y)];
Rz=[cos(z) -sin(z) 0;
    sin(z) cos(z) 0;
    0 0 1];

R=Rz*Ry*Rx;

PointsMoved=pointCloud(rigidTransform(Points.Location, R, [0 2 3]'));
%Visualize the point cloud piar
f2=figure;pcshowpair(Points,PointsMoved, 'VerticalAxis','Y', 'VerticalAxisDir', 'down','MarkerSize',200)
offset=.2;
hold on, text(Points.Location(:,1)+offset,Points.Location(:,2)+offset ,Points.Location(:,3)+offset,num2str([1:Points.Count]'))
hold on, text(PointsMoved.Location(:,1)+offset, PointsMoved.Location(:,2)+offset ,PointsMoved.Location(:,3)+offset,num2str([1:PointsMoved.Count]'))

title('Original and Transformed points')
xlabel('X (unit)')
ylabel('Y (unit)')
zlabel('Z (unit)')

%% Task B: Estimate homogenous transformation [rotation and translation] between original and transformed point cloud of task A [mandatory]
% First run task A to get data for this task B

%data from task A
pts=Points.Location; %reference points
ptsMoved=PointsMoved.Location; % Points to align to reference

% Estimate the transformation [R,t]
[R, t] = estimateRT_pt2pt(pts, ptsMoved);

% Transform 
rotatedPoints=inv(R)*transpose(ptsMoved)-inv(R)*t;
ptsAlligned=pointCloud(transpose(rotatedPoints));

% Visualize
figure,pcshowpair(Points,ptsAlligned, 'VerticalAxis','Y', 'VerticalAxisDir', 'down','MarkerSize',200)
hold on, text(Points.Location(:,1),Points.Location(:,2) ,Points.Location(:,3),num2str([1:Points.Count]'))
hold on, text(ptsAlligned.Location(:,1), ptsAlligned.Location(:,2) ,ptsAlligned.Location(:,3),num2str([1:ptsAlligned.Count]'))
title('tranformed and merged Point clouds')

% Find the error (RMSE)
err = Points.Location - ptsAlligned.Location;
err = err .* err;
err = sum(err(:));
rmse = sqrt(err/ptsAlligned.Count);


%% Task C: Create a function to iteratively allign bunny ptsMoved point cloud  to the reference [mandatory]

%load dataset
addpath('Data and Demos')
load('bunny.mat')

% extract points
pts=bunny.Location;%reference points
ptsMoved=bunnyMoved.Location; %Points to align to reference

% Set parameters
DownsampleStep=0.0015; % can be changed
visualize=true;

%Perform ICP
[bunny_estR,bunny_estt,err]=ICP(bunny, bunnyMoved, DownsampleStep, 0.9, 200, [0 0], false,0);

% Visualize Seperately
 bunnyAlligned=pointCloud(rigidTransform(ptsMoved,bunny_estR,bunny_estt));
 figure,pcshowpair(bunny,bunnyAlligned, 'VerticalAxis','Y', 'VerticalAxisDir', 'down','MarkerSize',100)

%% Task D: Add an adaptive Stop Criterion to task C [+1]

%load dataset
load('bunny.mat')

% extract points
pts=bunny.Location;%reference points
ptsMoved=bunnyMoved.Location; %Points to align to reference

% Set parameters
DownsampleStep=0.0015; % can be changed
tolerance=[0.001, 0.001];  % can be changed
visualize=true;

%Perform ICP
[bunny_estR,bunny_estt,err]=ICP(bunny, bunnyMoved, DownsampleStep, 0.9, 300, tolerance, false,0);

% Visualize Seperately
 bunnyAlligned=pointCloud(rigidTransform(ptsMoved,bunny_estR,bunny_estt));
 figure,pcshowpair(bunny,bunnyAlligned, 'VerticalAxis','Y', 'VerticalAxisDir', 'down','MarkerSize',100)



%% Task E:	Registration and Stitching of multiple Point Clouds [+1]

%load dataset
load('FabLab_scans.mat')

% Set parameters
DownsampleStep=0.001;
mergeSize=0.01;  %sets the parameter for pcmerge function to merge 2 points if they are assumed to be same.
tolerance=[0.01, 0.01];
visualize=true;

%visualize first pointcloud 
Map=FabLabm{1};

%open up a figure to display and update
f=figure;
hAxes = pcshow(Map, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
title('Stiched Scene')

% Set the axes property for faster rendering
hAxes.CameraViewAngleMode = 'auto';
hScatter = hAxes.Children;

% To initialize the pipeline
newPtCloud=FabLabm{1};

Rs=eye(3);
ts=[0 0 0]';

for i = 2:length(FabLabm)
    % Use previous  point cloud as reference.
    referencePtCloud = newPtCloud;
    
    % get new point cloud which you want to register to the previous point cloud
    newPtCloud = FabLabm{i};

    % Apply ICP registration.
    [estR,estt,err]=ICP(referencePtCloud, newPtCloud, DownsampleStep, 0.8, 300, tolerance, false, 0);
    
    %Accumulate the transformations as shown in Task A and as used inside the ICP function
    Rs(:,:,i) = estR;
    ts(:,i) = estt';

    % Transform the current/new point cloud to the reference coordinate system
    % defined by the first point cloud using the accumulated transformation.  
    ptCloudAligned= pointCloud(rigidTransform(newPtCloud.Location, Rs, ts));
    ptCloudAligned.Color=newPtCloud.Color;
    
    % Merge the newly alligned point cloud into the global map to update
    Map = pcmerge(Map, ptCloudAligned, mergeSize);

    % Visualize the world scene.
    hScatter.XData = Map.Location(:,1);
    hScatter.YData = Map.Location(:,2);
    hScatter.ZData = Map.Location(:,3);
    hScatter.CData = Map.Color;
    drawnow('limitrate')

end
    figure(f)
    
%% Task F: Create a function to iteratively alligns bunny ptsMoved based on distance and colour [+1]

%load dataset
load('slab.mat')

% extract points
pts=slab1.Location;%reference points
ptsMoved=slab2.Location; %Points to align to reference

figure,pcshowpair(slab1,slab2, 'VerticalAxis','Y', 'VerticalAxisDir', 'down','MarkerSize',100)
% Set parameters
DownsampleStep=0.3;
tolerance=[0.001, 0.001];
visualize=true;

% For testing here, we donot use colour as input. The default distance based ICP is used
useColour=false;
[slab_estR,slab_estt,err]=ICP(slab1, slab2, DownsampleStep, 0.90, 600, tolerance, useColour,0); % colour input only used for visualization

slabAlligned=pointCloud(rigidTransform(ptsMoved,slab_estR,slab_estt));
slabAlligned.Color=slab1.Color;

figure,pcshowpair(slab1,slabAlligned, 'VerticalAxis','Y', 'VerticalAxisDir', 'down','MarkerSize',100)
% Use colour assisted ICP
useColour=true;
alpha=0.003;
[slab_estR,slab_estt,err]=ICP(slab1, slab2, DownsampleStep, 0.75, 1000, tolerance, useColour, alpha);% colour used both for visualization and estimation

slabAlligned=pointCloud(rigidTransform(ptsMoved,slab_estR,slab_estt));
slabAlligned.Color=slab2.Color;

figure,pcshowpair(slab1,slabAlligned, 'VerticalAxis','Y', 'VerticalAxisDir', 'down','MarkerSize',100)
%% Task G: Create a function to iteratively allign  bunny ptsMoved using point-2-plane metric [+1]
%load dataset
load('bunny.mat')

% extract points
pts=bunny.Location;%reference points
ptsMoved=bunnyMoved.Location; %Points to align to reference

% Set parameters
tolerance=[0.001, 0.001];
DownsampleStep=0.0015;
useColour=false;
visualize=true;

% compare the convergence both metrics in terms of iterations and final error
disp('Using Point2Point');
[bunny_estR,bunny_estt,err_pt]=ICP(bunny, bunnyMoved, DownsampleStep, 0.9, 200, tolerance, useColour, 0); % point-to-point method
disp(['delta_t ' num2str(err_pt(1)) '; delta_R ' num2str(err_pt(2))]);
disp('Using Point2Plane');
[bunny_estR_pl,bunny_estt_pl,err_pl]=ICP(bunny, bunnyMoved, DownsampleStep, 0.9, 200, tolerance, useColour, 0, 1); % point-to-plane method
disp(['delta_t ' num2str(err_pl(1)) '; delta_R ' num2str(err_pl(2))]);

bunnyAlligned_pt=pointCloud(rigidTransform(ptsMoved, bunny_estR, bunny_estt));
figure,pcshowpair(bunny, bunnyAlligned_pt, 'VerticalAxis','Y', 'VerticalAxisDir', 'down','MarkerSize',100)
title('Point2Point')

bunnyAlligned_pl=pointCloud(rigidTransform(ptsMoved,bunny_estR_pl,bunny_estt_pl));
figure,pcshowpair(bunny, bunnyAlligned, 'VerticalAxis','Y', 'VerticalAxisDir', 'down','MarkerSize',100)
title('Point2Plane')


