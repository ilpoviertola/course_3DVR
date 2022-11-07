function [R,t] = estimateRT_pt2pl(pts, ptsMoved)
    n = pcnormals(pointCloud(ptsMoved));
    amountOfPoints = size(pts);
    C = zeros(amountOfPoints(1), 6);
    b = zeros(amountOfPoints(1), 1);
    for i=1:amountOfPoints(1)
        dx = ptsMoved(i, 1);
        dy = ptsMoved(i, 2);
        dz = ptsMoved(i, 3);
        nx = n(i, 1);
        ny = n(i, 2);
        nz = n(i, 3);
        sx = pts(i, 1);
        sy = pts(i, 2);
        sz = pts(i, 3);
        
        a1 = (nz * sy) - (ny * sz);
        a2 = (nx * sz) - (nz * sx);
        a3 = (ny * sx) - (nx * sy);
        
        C(i, :) = [a1, a2, a3, nx, ny, nz];
        b(i, 1) = (nx * dx) + (ny * dy) + (nz * dz) - (nx * sx) - (ny * sy) - (nz * sz);
    end
    X = pinv(C)*b;  % same as X = C \ b
    
    x = X(1,1);
    y = X(2,1);
    z = X(3,1); 
    Rx = [1 0 0;
          0 cos(x) -sin(x);
          0 sin(x) cos(x)];
    Ry = [cos(y) 0 sin(y);
          0 1 0;
          -sin(y) 0 cos(y)];
    Rz = [cos(z) -sin(z) 0;
          sin(z) cos(z) 0;
          0 0 1];
    R = Rz * Ry * Rx;
    
    t = transpose([X(4,1), X(5,1) X(6,1)]);    
end

