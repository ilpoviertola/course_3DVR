function transformedPoints = rigidTransform(Points, R, t)
    transformedPoints = zeros(size(Points,1), 3);
    for j = 1 : size(t(1, :),2)
        for i = 1 : size(Points,1)
            if(j==1)
                point = Points(i,:);
            else
                point=transformedPoints(i,:);
            end
            transformedPoint = (point * R(:,:,size(t(1, :),2)-j+1)) + transpose(t(:,size(t(1, :),2)-j+1));
            transformedPoints(i,:) = transformedPoint;
        end
    end
end