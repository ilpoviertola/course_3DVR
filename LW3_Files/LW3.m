%--------------------------------------------------------------------------
%   COMP.SGN.320 3D and Virtual Reality
%   
% Your implementation should run by executing this m-file ("run LW3.m"), 
% but feel free to create additional files for your own functions
% Make sure it runs without errors after unzipping
%
% Fill out the information below
%
% Group members:
% Additional tasks completed (5, 6, 7):
%
%--------------------------------------------------------------------------
clc; clear; close all;

% Global options
DEBUG = 1;  % Draw extra information for debug purposes
OPENGL = 1; % Device backend (OpenGL vs. MATLAB Painters)
ANTIALIASING = 1; % Apply antialiasing filter
DISPLAYPOINTCLOUD = 0; % Draw mesh or point cloud (for "3DView" window)
WIN_WIDTH = 800; % Window width
WIN_HEIGHT = 600; % Window height
FULLSCREEN = 0; % Fullscreen window

% Libraries and assets
addpath('./RenderingPipeline/'); % Mini lib to help with the rendering pipeline and virtual content
addpath('./ply/'); % Folder with (ASCII) .ply assets
%% DO NOT EDIT THIS PART
% Create main window(s)
if(OPENGL)
    wh1 = Window(WIN_WIDTH, WIN_HEIGHT, '3DView', FULLSCREEN, 'opengl', ANTIALIASING);
    wh2 = Window(WIN_WIDTH, WIN_HEIGHT, 'XRView', FULLSCREEN, 'opengl', ANTIALIASING);
else
    wh1 = Window(WIN_WIDTH, WIN_HEIGHT, '3DView', FULLSCREEN, 'painters', ANTIALIASING);
    wh2 = Window(WIN_WIDTH, WIN_HEIGHT, 'XRView', FULLSCREEN, 'painters', ANTIALIASING);
end

%% Virtual content - Task 2.1

% Example using the "RenderingPipeline" lib:
% Create default (empty) scene:
sceneObj = Scene();

% Load 3D objects from ply files and them add to sceneObj:
sceneObj = sceneObj.LoadObject3D('cube');

% Create 3D object from ply file:
mesh1 = Object3D('pyramid');

% Change some properties of the mesh and add it to sceneObj: 
mesh1 = mesh1.SetScale([1.5, 1.5, 1.5]);
mesh1 = mesh1.SetPosition([0, 0, 1.5]);
mesh1 = mesh1.SetRotationX(-90);
sceneObj = sceneObj.AddObject3D(mesh1);

% Create a custom mesh:
mesh2 = Object3D();

% Define properties for custom mesh:
% Name
mesh2.Name = 'My custom mesh';

% Vertices
mesh2.XYZ = [0 0  0  10 0  10 10 10; ... %X
             0 0  10 0  10 0  10 10; ... %Y
             0 10 0  0  10 10 0  10];    %Z  

% Number of vertices
mesh2.VerticesCount = size(mesh2.XYZ, 2);

% Vertices color
mesh2.VerticesColor = [1   0   0   1   1   0   0   1; ... %R
                       0   1   0   1   0   1   0   1; ... %G
                       0   0   1   0   1   1   0   1; ... %B
                       1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0];    %A

% List of triangle connections
mesh2.TriangleNodes = [1  2  1  2  2  2  3  5  1  1  4  6; ... 
                       2  3  2  4  5  6  5  7  3  4  6  7; ...
                       3  5  4  6  8  8  7  8  7  7  7  8]; 
 
% Triangle number   #1  #2  #3  #4 
mesh2.FacesColor = [1   1   0   0   0   0   1   1   0   0   1   1; ... %R
                    0   0   1   1   0   0   1   1   1   1   0   0; ... %G
                    0   0   0   0   1   1   0   0   1   1   1   1; ... %B
                    1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0];  %A

% Number of faces
mesh2.FacesCount = size(mesh2.FacesColor, 2);

% Set material
mesh2.Material.SetMaterial(0.3, 0.3, 1.0, 25, 0.5);

% Apply some transformations: 
mesh2 = mesh2.SetScale([0.1, 0.1, 0.1]);
mesh2 = mesh2.SetPosition([2, 0, 2.5]);

% Add custom object to sceneObj:
sceneObj = sceneObj.AddObject3D(mesh2);

% Change main camera view (3DView window)
sceneObj.MainCamera.Yaw = sceneObj.MainCamera.Yaw - 145; 
sceneObj.MainCamera.Pitch = sceneObj.MainCamera.Pitch + 10;
sceneObj.MainCamera.FoV = 60;

% Change light(s) attributes
sceneObj.ListOfLightSources{1}.Type = 'local';
sceneObj.ListOfLightSources{1}.Position = [0, 0, 1];

% Preview 3D model(s)
 figure,
 for i=1:length(sceneObj.ListOfObjects3D)
    hold on;
     for c = 1:size(sceneObj.ListOfObjects3D{i}.TriangleNodes, 2)
         patch('Faces', [1 2 3], ...
             'Vertices', [sceneObj.ListOfObjects3D{i}.XYZ(:,sceneObj.ListOfObjects3D{i}.TriangleNodes(1, c)), ...
             sceneObj.ListOfObjects3D{i}.XYZ(:,sceneObj.ListOfObjects3D{i}.TriangleNodes(2, c)), ...
             sceneObj.ListOfObjects3D{i}.XYZ(:,sceneObj.ListOfObjects3D{i}.TriangleNodes(3, c))]', ...
             'FaceColor', sceneObj.ListOfObjects3D{i}.FacesColor(1:3,c), ...
             'FaceAlpha', sceneObj.ListOfObjects3D{i}.FacesColor(4,c), ...
             'EdgeColor', 'none');
     end
 end
 hold off;
 axis equal;

%% Real-world content - Task 2.2

load synthdata.mat
distMap = double(Depth);
colorImg = im2double(Image);

Dparam.fp = Dparam.fx / Dparam.pixelsize;
Cparam.fp = Cparam.f / Cparam.pixelsize;
KD = Dparam;
KC = Cparam;

% Convert distance map to depth:
depthImg = Dist2Depth(distMap, KD);

% Mapping colour data to depth image plane:
[resampledColorImage, colorCamDepth, XYZCam] = MapColor(KC, KD, ...
                                              colorImg, depthImg, ...
                                              R,T);

% Discard background objects using depth information:
valToDiscard = 12;
Rch = resampledColorImage(:,:,1); Rch(depthImg > valToDiscard) = 0;
Gch = resampledColorImage(:,:,2); Gch(depthImg > valToDiscard) = 0;
Bch = resampledColorImage(:,:,3); Bch(depthImg > valToDiscard) = 0;
resampledColorImage = cat(3, Rch, Gch, Bch);
% Assign nan to depth values that we don't care:
depthImg(depthImg > valToDiscard) = nan;

figure, imshow(resampledColorImage);
title('Foreground objects on color data mapped to depth');


%% 3D scene reconstruction - Task 2.3 / Task 2.6

%tri = delaunayTriangulation(u(:),v(:));
%tri = delaunayTriangulation(X(:),Y(:),Z(:));

% Preview triangulated mesh:
% faceColorR = resampledColorImage(:,:,1); faceColorR = faceColorR(:);
% faceColorG = resampledColorImage(:,:,2); faceColorG = faceColorG(:);
% faceColorB = resampledColorImage(:,:,3); faceColorB = faceColorB(:);
% faceColor = [faceColorR, faceColorG, faceColorB];
%trisurf(tri.ConnectivityList, u(:), v(:), Z(:), 'FaceVertexCData', faceColor, 'EdgeColor', 'none');


%% Rendering pipeline - Task 2.4 / 2.5 / 2.6
deltaTime = 0;
lastFrameTime = 0;
clock = tic; % start timer
frameCount = 0;

while (ishandle(wh1) && ishandle(wh2))
    
    % Update deltatime (DO NOT EDIT)
    time = toc(clock);
    deltaTime = time - lastFrameTime;
    lastFrameTime = time;
    frameCount = frameCount + 1;
    if(DEBUG && mod(frameCount, 2) == 1)      
        fprintf("Delta time: %.5f s, FPS: %.2f\n",deltaTime, 1/deltaTime); 
    end
    
    % Update mouse movement (3DView window) (DO NOT EDIT)
    if(wh1.UserData.MouseInput.Button1Down)
        sceneObj.MainCamera = sceneObj.MainCamera.Rotate(-wh1.UserData.MouseInput.X * 5, -wh1.UserData.MouseInput.Y * 5);
    end
    
    %% Add more 3D models, add animations or/and change attributes - Task 2.4
    %Rotate object:
    %obj1 = sceneObj.ListOfObjects3D{2};
    %angleY = 1;
    %obj1 = obj1.Rotate([0, angleY * deltaTime, 0]); % Rotate ~1 degree on every iteration
    %sceneObj.ListOfObjects3D{2} = obj1;


    % Render 3DView (DO NOT EDIT EXCEPT FOR THE AXES LIMIT)
    if(DEBUG)
        set(0,'CurrentFigure',wh1);
        sceneObj.RenderScene(DISPLAYPOINTCLOUD, DEBUG);
        axis on;
        % Axes limit (swapped order because of Matlab plot)
        xlim([-1, 2]); % Fix Z limit
        ylim([-2, 2]); % Fix X limit
        zlim([-2, 2]); % Fix Y limit
    end


    %% Collisions - Task 2.6


    %% Fused data - Task 2.5

    set(0,'CurrentFigure',wh2); % Activate XRView window
    cla; % Clear content
    ax = gca; % Get handle to current axes and change some attributes
    ax.Color = 'k';
    ax.GridColor = 'w';
    ax.XColor = 'w';
    ax.YColor = 'w';
    ax.ZColor = 'w';
    ax.Box = true;
    xlabel('u (pixels)');
    ylabel('v (pixels)');
    hold on;
    
   
    % Draw virtual content and real-world content:


    % "Render" light source:
    %lt = light(gca);
    %lt.Color = sceneObj.ListOfLightSources{1}.Color;
    %lt.Style = sceneObj.ListOfLightSources{1}.Type;
    %lt.Position = ...

    hold off;
    
    axis on;
    axis auto;
    set(gca, 'ZDir','reverse'); % fix axis direction Matlab plot
    set(gca,'YDir','reverse'); % fix axis direction Matlab plot
    xlim([1, resX]); % XRView image resolution width
    ylim([1, resY]); % XRView image resolution height
    zlim([0.01, 20]); % near / far clip - XRView camera
    pbaspect([1, resY/resX, 1]); %Fix aspect ratio of axis box (DO NOT EDIT)

    
    drawnow();
end


%--------------------------------------------------------------------------
% Functions
%--------------------------------------------------------------------------
%Distance map to Depth image
function depth = DistToDepth(dist, K)
% Input:
%   dist - distance map
%   K - calibration matrix
%
% Output:
%   depth - depth map


end

function [resampledColorImg, colorCamDepth, XYZCam] = GetMappedColor(KC, KD, ...
                                              colorImg, depthImg, ...
                                              R,T)
% Input:
%   dist - distance map
%   KC - calibration matrix colour camera
%   KD - calibration matrix depth camera
%   colorImg - colour image
%   depthImg - depth image
%   R - relative rotation between color camera and depth camera
%   T - relative translation between color camera and depth camera
%
% Output:
%   resampledColorImg - resampled color image (same resolution than depth image)
%   colorCamDepth - respective depth from color camera viewpoint
%   XYZCam - transformed XYZ values, after applying R and T 

end
