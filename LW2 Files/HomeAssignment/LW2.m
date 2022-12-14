%--------------------------------------------------------------------------
% COMP.SGN.320 3D and Virtual Reality 
%
%
% Your implementation should run by executing this m-file ("run LW2.m"), 
% but feel free to create additional files for your own functions
% Make sure it runs without errors after unzipping
%
% Fill out the information below
%
% Group members: Petri Vainio, Ilpo Viertola, Henrik Lauronen
% Additional tasks completed (5, 6, 7, 8): 5, 6, 7, 8
%
% Fill in your implementation at the assigned slots. You can use the 
% existing drawing scripts or draw your own figures. 
% To give an impression of the scope of each task, 
% the number of lines of code in the reference implementation is given
% This is highly subjective depending on coding style, and should not
% be considered as a requirement. Use as much code as you need, but if you
% start having 10 times the lines, you may consider that there might be an
% easier way to do it.
%--------------------------------------------------------------------------

% Load synthetic data:
%load synthdata

% Load real data:
%   .Depth from Kinect is in "mm";
%   .Translation vector is in "mm";
%   .Intrinsic parameters are in "pixels";

kinect=false;
if kinect
    load KinectData.mat
    Image = imread('Colour_rect.tif');
    Depth = imread('Depth_rect.tif'); 
else
    load synthdata
end
  
% load KinectData.mat 
% Image = imread('Colour_rect.tif');
% Depth = imread('Depth_rect.tif'); 

%% Task 1: Plotting global point cloud (8 lines of code)
subplot(1,2,1), imshow(Image), subplot(1,2,2), imshow(rescale(Depth), [0 1])

% Back projection from PMD image plane to global space

[X,Y] = meshgrid(-size(Depth,2)/2+0.5:1:size(Depth,2)/2-0.5, -size(Depth,1)/2+0.5:1:size(Depth,1)/2-0.5);
%[X,Y] = meshgrid(1:1:size(Depth,2), 1:1:size(Depth,1));
Z=zeros(size(Depth,1),size(Depth,2));

for x = 1:1:size(Depth,2)
    for y = 1:1:size(Depth,1)
        if kinect
            Z=Depth;
            X(y,x)=(X(y,x))/Dparam.fx*double(Z(y,x));
            Y(y,x)=(Y(y,x))/Dparam.fy*double(Z(y,x));
        else
            Z(y,x)=(Depth(y,x) ...
            *((Dparam.f/Dparam.pixelsize) ...
            /sqrt(((X(y,x)-1)^2+(Y(y,x)-1)^2)+(Dparam.f/Dparam.pixelsize)^2)));
            X(y,x)=((X(y,x)*Z(y,x))/(Dparam.fx/Dparam.pixelsize));
            Y(y,x)=((Y(y,x)*Z(y,x))/(Dparam.fy/Dparam.pixelsize));
        end
        
    end
end

% Plotting
figure; hold on;
scatter3(X(:), Y(:), Z(:), 10, Z(:));
colormap jet; colorbar;
scatter3(0, 0, 0, 500, 'gx', 'LineWidth', 2)
title('Task 1: Point cloud in global (x,y,z) space');
set(gca,'YDir','reverse');
set(gca,'ZDir','reverse');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal
drawnow;
%% Task 2: Projection to color camera image plane (5 lines of code)

u_colorcam=zeros(size(Depth,1),size(Depth,2));
v_colorcam=zeros(size(Depth,1),size(Depth,2));
z_colorcam=zeros(size(Depth,1),size(Depth,2));

for x = 1:1:size(Depth,2)
    for y = 1:1:size(Depth,1)
        k=[R T]*[double(X(y,x)); double(Y(y,x)); double(Z(y,x)); double(1)];
        if kinect
            u_colorcam(y,x)=Cparam.fx*(k(1)/k(3))+Cparam.cx;
            v_colorcam(y,x)=Cparam.fy*(k(2)/k(3))+Cparam.cy;
        else
            u_colorcam(y,x)=Cparam.fx/Cparam.pixelsize*(k(1)/k(3))+Cparam.cx;
            v_colorcam(y,x)=Cparam.fy/Cparam.pixelsize*(k(2)/k(3))+Cparam.cy;
        end
        z_colorcam(y,x)=k(3);
    end
end

% Plotting
figure; axis equal
imshow(Image, []); hold on; %#ok<*NODEF>

% Only drawing the objects in front to check alignment
if kinect
    objectmask = z_colorcam>400;
else
    objectmask = z_colorcam<13;
end
sc = scatter(u_colorcam(objectmask), v_colorcam(objectmask), 10, z_colorcam(objectmask), 'filled');
sc.MarkerEdgeAlpha = 0.2;
sc.MarkerFaceAlpha = 0.2;
title( 'Task 2: Global depth points projected on image plane of the color camera');
drawnow;

%% Task 3: Resampling projected data (3 lines of code)

z_colorcam_reg = zeros(size(Image,1),size(Image,2));
F = scatteredInterpolant(double(u_colorcam(:)), double(v_colorcam(:)), double(z_colorcam(:)), 'nearest');
for i = 1:size(Image,1)
    for j = 1:size(Image,2)
        z_colorcam_reg(i,j) = F(j,i);
    end
end

% Plotting
figure;
subplot( 131); imshow( Image, []); title('Task 3: Original color image')
subplot( 132); imshow( z_colorcam_reg, []); title('Task 3: Resampled depth image');
subplot( 133); imshowpair( Image, z_colorcam_reg); title('Task 3: Resampled depth on original color')

%% Task 4: Visualizing combined depth/color data

% Well, actually, this one is just plotting so you're done already
figure; 

if kinect
    %take just important parts to make calculation easier
    surf(z_colorcam_reg(65:845,560:1480), double(Image(65:845,560:1480)), 'EdgeColor', 'none')
else
    surf(z_colorcam_reg, double(Image), 'EdgeColor', 'none')
end

set(gca,'ZDir','reverse');
set(gca,'YDir','reverse');
title('Task 4: 3D mesh generated from resampled depth')
drawnow;

%% Task 5: Artifact removal (6 lines of code)

% Just plotting here, add your implementation to the edgeRemoval.h function
figure; 

if kinect
    %take just important parts to make calculation easier
    h = surf(z_colorcam_reg(65:845,560:1480), double(Image(65:845,560:1480)), 'EdgeColor', 'none');
else
    h = surf(z_colorcam_reg, double(Image), 'EdgeColor', 'none');
end

set(gca,'ZDir','reverse');
set(gca,'YDir','reverse');
title( 'Task 5: 3D mesh generated from resampled depth with edge artifacts removed')
edgeRemoval(h);



%% Task 6: Color resampling (4 lines of code)
resampledColorImage = zeros(size(Depth,1),size(Depth,2), 3);

[x_grid, y_grid] = meshgrid(1:1:size(Image,2),1:1:size(Image,1));

resampledColorImage(:,:,1) = interp2(x_grid, y_grid, double(Image(:,:,1)), u_colorcam, v_colorcam);
resampledColorImage(:,:,2) = interp2(x_grid, y_grid, double(Image(:,:,2)), u_colorcam, v_colorcam);
resampledColorImage(:,:,3) = interp2(x_grid, y_grid, double(Image(:,:,3)), u_colorcam, v_colorcam);

z = rescale(Depth);

% Plotting
figure;  

subplot( 231); imshow( Image, []); title('Task 3: Original color image')
subplot( 232); imshow( z_colorcam_reg, []); title('Task 3: Resampled depth image');
subplot( 233); imshowpair( Image, z_colorcam_reg); title('Task 3: Resampled depth on original color')

subplot( 234); imshow( z, []); title('Task 6: Original depth image');
subplot( 235); imshow( resampledColorImage, []); title('Task 6: Resampled color image')
subplot( 236); imshowpair( resampledColorImage, z); title('Task 6: Resampled color on original depth')
drawnow;


%% Task 7: Z-buffering (19 lines of code)
if kinect
    u_colorcam=u_colorcam(objectmask);
    v_colorcam=v_colorcam(objectmask);
    z_colorcam=z_colorcam(objectmask);
    points=[u_colorcam(:), v_colorcam(:), z_colorcam(:)/max(z_colorcam(:))];
else
    points=[u_colorcam(:), v_colorcam(:), z_colorcam(:)];
end

img_dist_0=[u_colorcam(:), v_colorcam(:), zeros((size(z_colorcam,1)*size(z_colorcam,2)),1)];
Idx = knnsearch(points,img_dist_0,"K",1,"Distance","euclidean");
uvz_cam=points(Idx,:);

if kinect
    F = scatteredInterpolant(double(uvz_cam(:,1)), double(uvz_cam(:,2)), double(uvz_cam(:,3)*max(z_colorcam(:))), 'nearest');
else
    F = scatteredInterpolant(double(uvz_cam(:,1)), double(uvz_cam(:,2)), double(uvz_cam(:,3)), 'nearest');
end

z_colorcam_reg_zbuf = zeros(size(Image,1),size(Image,2));
for i = 1:size(Image,1)
    for j = 1:size(Image,2)
        z_colorcam_reg_zbuf(i,j) = F(j,i);
    end
end

[uc,vc] = meshgrid(1:size(Image,2),1:size(Image,1));

% Plotting
figure;
subplot(131);
scatter(u_colorcam(:)', v_colorcam(:)', 10, z_colorcam(:)')
ylim([0 size(Depth, 1)]); xlim([0 size(Depth, 2)]);
title( 'Irregular'); 
set(gca,'YDir','reverse'); 
axis image; 
drawnow;

subplot(132);
axis equal
scatter(uc(:), vc(:), 10, z_colorcam_reg(:))
ylim([0 size(Depth, 1)]); xlim([0 size(Depth, 2)]);
title( 'Regular'); 
set(gca,'YDir','reverse'); 
axis image; 
drawnow;

subplot(133);
axis equal
scatter(uc(:), vc(:), 10, z_colorcam_reg_zbuf(:))
ylim([0 size(Depth, 1)]); xlim([0 size(Depth, 2)]);
title( 'Regular z-buffered');
set(gca,'YDir','reverse'); 
axis image; 
drawnow;

figure; 
subplot(231); imshow( z_colorcam_reg, []);
title( 'Task 7: Depth data resampled into a regular grid ');
subplot(234); imshow( z_colorcam_reg_zbuf, []);
title( 'Task 7: Depth data resampled into a regular grid after Z-buffering');

subplot(2, 3, [2 3 5 6]); 
if kinect
    %take just important parts to make calculation easier
    h = surf(z_colorcam_reg_zbuf(65:845,560:1480), double(Image(65:845,560:1480)), 'EdgeColor', 'none');
else
    h = surf(z_colorcam_reg_zbuf, double(Image), 'EdgeColor', 'none');
end
set(gca,'ZDir','reverse');
set(gca,'YDir','reverse');
title( 'Task 7: Z-buffering 3D mesh generated from resampled depth');
edgeRemoval(h);
drawnow;
%% Task 8: Occlusion handling (14 lines of code)
z_colorcam_flat = z_colorcam(:);
u_colorcam_flat = u_colorcam(:);
v_colorcam_flat = v_colorcam(:);

% Identify occlusions by searching for the nearest pixel
[Idx,D] = knnsearch([u_colorcam_flat v_colorcam_flat],...
    [uc(:) vc(:)],'K',2,'Distance', 'euclidean');
D_n = zeros(size(D,1),1);
for i=1:size(D,1)
    D_n(i,1) = min(D(i,:));
end

% If distance to nearest pixel is over 1 count these as missing
Idx = Idx(D_n >= 1.5665);
pts = [u_colorcam_flat v_colorcam_flat z_colorcam_reg_zbuf(:)];
uvz_missing = pts(Idx,:);
u_missing = uvz_missing(:,1);
v_missing = uvz_missing(:,2);
z_missing = uvz_missing(:,3);

if ~kinect
    ptCloud = pointCloud([u_colorcam_flat v_colorcam_flat z_colorcam_reg_zbuf(:)]);
    planeModel = pcfitplane(ptCloud, 1);
end

% Plotting
figure;
scatter3(u_colorcam(:), v_colorcam(:), z_colorcam(:), 10, z_colorcam(:));
hold on;
plot(planeModel)
scatter3(u_missing, v_missing, z_missing, 50, 'gx');
set(gca,'YDir','reverse');
set(gca,'ZDir','reverse');
title('UVZ-point cloud with the plane fit (red) and missing pixels (green)')
drawnow;

z = uvz_cam(:,3);
% z = (-au - bv - d)/c
a = planeModel.Parameters(1);
b = planeModel.Parameters(2);
c = planeModel.Parameters(3);
d = planeModel.Parameters(4);
z(Idx,:) = (-a*u_colorcam_flat(Idx) - b*v_colorcam_flat(Idx) - d) / c;

uvz = [uvz_cam(:,1) uvz_cam(:,2) z];

if kinect
    F = scatteredInterpolant(double(uvz(:,1)), double(uvz(:,2)), double(uvz(:,3)*max(z_colorcam(:))), 'nearest');
else
    F = scatteredInterpolant(double(uvz(:,1)), double(uvz(:,2)), double(uvz(:,3)), 'nearest');
end

z_colorcam_reg_zbuf_filled = zeros(size(Image,1),size(Image,2));
for i = 1:size(Image,1)
    for j = 1:size(Image,2)
        z_colorcam_reg_zbuf_filled(i,j) = F(j,i);
    end
end

figure; 
subplot(231); imshow( z_colorcam_reg_zbuf, []);
title( 'Task 7: Depth data resampled into a regular grid after Z-buffering ');
subplot(234); imshow( z_colorcam_reg_zbuf_filled, []);
title( 'Task 7: Depth data resampled into a regular grid after Z-buffering and occlusion filling');

subplot(2, 3, [2 3 5 6]); h = surf(z_colorcam_reg_zbuf_filled, double(Image), 'EdgeColor', 'none');
set(gca,'ZDir', 'reverse')
set(gca,'YDir','reverse');
title( 'Task 8: Z-buffering 3D mesh generated from resampled depth')
edgeRemoval(h);
drawnow;





