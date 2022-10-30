function transformedPoints = rigidTransform(Points, R, t)

    transformedPoints = zeros(size(Points,1), 3);
    for i = 1 : size(Points,1)
        point = Points(i,:);
        transformedPoint = (point * R) + t;
        transformedPoints(i,:) = transformedPoint;
    end
end