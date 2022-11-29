function Z = Dist2Depth(distMap,KD)
% Generate X and Y coords of the depth map
[X,Y] = meshgrid(1:size(distMap,2), 1:size(distMap,1));
X = X - KD.cx;  % Center the X plane
Y = Y - KD.cy;  % Center the Y plane

Z = KD.fp * distMap ./ sqrt(KD.fp^2 + X.^2 + Y.^2);

end