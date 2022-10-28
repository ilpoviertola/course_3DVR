function transformedPointCloud = rigidTransform(z, y, x, t)
    Points=pointCloud([0 0 3; 5 0 5; 2.5 2.5 0]);
    
    R = [
        cos(z)*cos(y) cos(z)*cos(y)*sin(x)-sin(z)*cos(x) cos(z)*sin(y)*cos(x)+sin(z)*sin(x);
        cos(z)*cos(y) cos(z)*cos(y)*sin(x)+sin(z)*cos(x) cos(z)*sin(y)*cos(x)-sin(z)*sin(x);
        -sin(y) cos(y)*sin(x) cos(y)*cos(x)
    ];
    transformedPoints = zeros(Points.Count, 3);
    for i = 1 : Points.Count
        point = Points.Location(i,:);
        transformedPoint = (point * R) + t;
        transformedPoints(i,:) = transformedPoint;
    end
    transformedPointCloud = pointCloud(transformedPoints);
end
