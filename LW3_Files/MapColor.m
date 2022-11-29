function [resampledColorImage, colorCamDepth, XYZCam] = MapColor( ...
    KC, KD, colorImg, depthImg, R, T)
% Generate X and Y coords of the depth map
[X,Y] = meshgrid(1:size(depthImg,2), 1:size(depthImg,1));
X = X - KD.cx;  % Center the X plane
Y = Y - KD.cy;  % Center the Y plane

% Global 3D coordinates
X = depthImg.*X / KD.fp;
Y = depthImg.*Y / KD.fp;
Z = depthImg;

% From global 3D to camera coordinates
u = zeros(size(depthImg,1),size(depthImg,2));
v = zeros(size(depthImg,1),size(depthImg,2));
z = zeros(size(depthImg,1),size(depthImg,2));

for x = 1:1:size(depthImg,2)
    for y = 1:1:size(depthImg,1)
        k=[R T]*[double(X(y,x)); double(Y(y,x)); double(Z(y,x)); double(1)];
        u(y,x)=KC.fp*(k(1)/k(3))+KC.cx;
        v(y,x)=KC.fp*(k(2)/k(3))+KC.cy;
        z(y,x)=k(3);
    end
end
colorCamDepth = z;
XYZCam = cat(3,u,v,z);

% Mapping of color data to depth image plane.
resampledColorImage = zeros(size(depthImg,1),size(depthImg,2), 3);
[x_grid, y_grid] = meshgrid(1:1:size(colorImg,2),1:1:size(colorImg,1));
resampledColorImage(:,:,1) = interp2(x_grid, y_grid, double(colorImg(:,:,1)), u, v);
resampledColorImage(:,:,2) = interp2(x_grid, y_grid, double(colorImg(:,:,2)), u, v);
resampledColorImage(:,:,3) = interp2(x_grid, y_grid, double(colorImg(:,:,3)), u, v);
end

