%--------------------------------------------------------------------------
% COMP.SGN.320 3D and Virtual Reality 
% Capture Color + Depth data from Kinect v2 
%--------------------------------------------------------------------------
%   Kinect 2 Interface
%   Stream options:
%        ___________________________________________
%       | STREAM OPTION |           OUTPUT          |
%       |_______________|___________________________|                   
%       | OPTION = 0    | Color                     |   
%       | OPTION = 1    | Depth                     |   
%       | OPTION = 2    | IR                        |   
%       | OPTION = 3    | Color & Depth             |   
%       | OPTION = 4    | Color & IR                |   
%       | OPTION = 5    | Depth & IR                |   
%       | OPTION = 6    | Color & Depth & IR        |   
%       | OPTION = 7    | Body Track                |   
%       | OPTION = 8    | Depth & Face Track        |
%       | OPTION = 9    | Body Track & Color & Depth|
%       |_______________|___________________________|
%
%   Usage: cam = KinectInterface(<stream_option>)
%--------------------------------------------------------------------------
clear all; close all; clc;
%% Add Kinect lib and calibration lib
addpath('./../CaptureCalibrationData/KinectInterface/');
addpath('./TOOLBOX_calib');
%% Program parameters
outputImageFormat = 'tif';  %Format of the output image file. 
                            %Uncompressed and 16-bit support
%% Capture a new scene (color image + depth image) by pressing 's'
streamOption = 3;

%Initialize Kinect
kinectObj = KinectInterface(streamOption);
kinectObj = kinectObj.Init();

%Grab data from Kinect
h = figure(1);
%Setup keyboard I/O
set(h,'WindowKeyPressFcn',@keyPressCallback);
key = [];
key.grabFrame = false;
key.exitLoop = false;
h.UserData = key;

while (ishandle(h))
    
    %Get data
    kinectObj = kinectObj.GetData();
    colorFrame = kinectObj.colourImg;
    depthFrame = kinectObj.depthImg;
    
    
    if(~isempty(colorFrame) && ~isempty(depthFrame))
        %Show data
        subplot(121)
        imagesc(colorFrame);
        axis image;
        
        subplot(122)
        imagesc(depthFrame);
        colormap gray; 
        cb = colorbar;
        cb.FontSize = 20;
        axis image; 
        
        drawnow();
    end
    
    %Process I/O
    if(ishandle(h))
        key = h.UserData;
        figure(h); %Keep figure focus
    end
    if(key.grabFrame)
        
        %Wait for the next valid frame
        if(~isempty(colorFrame) && ~isempty(depthFrame))
            
            %Save image(s)
            colorTxt = sprintf(['./Colour.' outputImageFormat]);
            depthTxt = sprintf(['./Depth.' outputImageFormat]);
            
            imwrite(colorFrame,colorTxt);
            imwrite(depthFrame,depthTxt);
            
            %Reset save image trigger
            disp("Saving image...");
            key.grabFrame = false;
            h.UserData = key;
        end
    end
    
    if(key.exitLoop)
        break;
    end
    
end

%% Close Kinect and free memory
kinectObj.Close();
clear mex;

%% Functions
%--------------------------------------------------------------------------
function keyPressCallback(source,eventdata)

%Determine the key that was pressed
keyPressed = eventdata.Key;

%If "s", save image
if(strcmp(keyPressed, 's'))
    key = source.UserData;
    key.grabFrame = true;
    source.UserData = key;
%If "q", quit loop
elseif(strcmp(keyPressed, 'q') || strcmp(keyPressed, 'escape'))
    key = source.UserData;
    key.exitLoop = true;
    source.UserData = key;
end

end



