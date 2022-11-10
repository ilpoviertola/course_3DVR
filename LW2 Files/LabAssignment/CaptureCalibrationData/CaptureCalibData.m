%--------------------------------------------------------------------------
% COMP.SGN.320 3D and Virtual Reality 
% Grab frames from Kinect v2 and save them
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
%% Add Kinect lib
addpath('./KinectInterface/');
%% Program parameters
automaticCapture = 0;       %Automatic or manual capture/trigger mode
outputImageFormat = 'tif';  %Format of the output image file. 
                            %Uncompressed and 16-bit support

%Automatic trigger:
numMaxCalibImgs = 15;       %Maximum number of images to be captured 
delayBetweenEachShot = 7;   %Delay between each shot (in seconds)

%Manual trigger:
%Press 's' to save image while figure is in focus
%Press 'q' or 'esc' to close figure

%% Grab frames from Kinect 
streamOption = 4;

%Initialize Kinect
kinectObj = KinectInterface(streamOption);
kinectObj = kinectObj.Init();

%Get Kinect data
savedFramesCount = 0;
h = figure(1);

%Automatic trigger/capture
if(automaticCapture)
    tic;
    trigger = 0;
    currentCount = 0;
    while (ishandle(h) && savedFramesCount<numMaxCalibImgs)
        
        %Get data
        kinectObj = kinectObj.GetData();
        colorFrame = kinectObj.colourImg;
        irFrame = kinectObj.irImg;
        
        %Update trigger after n time
        n = toc;
        if(~trigger && n >= delayBetweenEachShot)
            trigger = 1;
        end
        
        %Show valid data from Kinect
        if(~isempty(colorFrame) && ~isempty(irFrame))
            %Show data
            subplot(121)
            imagesc(colorFrame);
            axis image;
            
            subplot(122)
            imagesc(irFrame);
            axis image;
            
            drawnow();
        end
        
        %Show count down (at each second)
        if(~trigger && floor(n) == currentCount)
            fprintf("Countdown: %d\n", delayBetweenEachShot - floor(n));
            currentCount = currentCount + 1;
        end
        
        %Check if frames are valid and if trigger is on
        if(trigger && ~isempty(colorFrame) && ~isempty(irFrame))
            
            %Save image(s)
            colorTxt = sprintf(['./colourImg_%03.f.' outputImageFormat], savedFramesCount);
            irTxt = sprintf(['./irImg_%03.f.' outputImageFormat], savedFramesCount);
            
            imwrite(colorFrame,colorTxt);
            imwrite(irFrame,irTxt);
            
            savedFramesCount = savedFramesCount + 1;
                
            %Reset trigger and timers    
            disp("Saving image...");    
            trigger = 0;
            currentCount = 0;
            tic;
        end
    end
%Manual trigger/capture
else 
    
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
        irFrame = kinectObj.irImg;
        
        if(~isempty(colorFrame) && ~isempty(irFrame))
            %Show data
            subplot(121)
            imagesc(colorFrame);
            axis image;
            
            subplot(122)
            imagesc(irFrame);
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
            if(~isempty(colorFrame) && ~isempty(irFrame))
            
                %Save image(s)
                colorTxt = sprintf(['./colourImg_%03.f.' outputImageFormat], savedFramesCount);
                irTxt = sprintf(['./irImg_%03.f.' outputImageFormat], savedFramesCount);
                
                imwrite(colorFrame,colorTxt);
                imwrite(irFrame,irTxt);
                
                savedFramesCount = savedFramesCount + 1;
                
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

