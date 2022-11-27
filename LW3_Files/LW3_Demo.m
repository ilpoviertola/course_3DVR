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
% Additional tasks completed (5, 6, 7, 8):
%
%--------------------------------------------------------------------------
clc; clear; close all;

% Global options
DEBUG = 1;              % Draw extra information for debug purposes
OPENGL = 1;             % Device backend (OpenGL vs. MATLAB Painters)
ANTIALIASING = 1;       % Apply antialiasing filter
DISPLAYPOINTCLOUD = 0;  % Draw mesh or point cloud (for "3DView" window)
WIN_WIDTH = 800;        % Window width
WIN_HEIGHT = 600;       % Window height
FULLSCREEN = 0;         % Fullscreen window
SAMPLES = 4;            % 3D reconstruction sampling factor: 1,2,4,6,8,...

% Run demo
RunLW3Demo;
