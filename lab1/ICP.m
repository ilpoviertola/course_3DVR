function [out_estR,out_estt] = ICP(pts, ptsMoved, downsampleStep, correspondences, maxIterations, tolerance)

    % Initialize R and t
    out_estR = eye(3,3);
    out_estt = zeros(1,3);
    
    %Downsample points
    pts_ds = pcdownsample(pts, 'gridAverage', downsampleStep);
    pts_ds = pts_ds.Location;

    ptsMoved_ds = pcdownsample(ptsMoved, 'gridAverage', downsampleStep);
    ptsMoved_ds = ptsMoved_ds.Location;

    for k=1:maxIterations
        ptsAligned = rigidTransform(ptsMoved_ds, out_estR, out_estt);
        
        % Search corresponding nearest points
        [IDX,D] = knnsearch(pts_ds, ptsAligned);

        nearestPoints = zeros(size(IDX,1),3);
        for i=1:size(IDX,1)
            nearestPoints(i,:) = pts_ds(IDX(i),:);
        end

        % Sort points by distance D
        A = [D nearestPoints ptsAligned];
        pts_sorted = sortrows(A, 1);

        % Filter correspondences based on distance
        n = ceil(size(pts_sorted,1)*correspondences);
        nearest_Filtered = pts_sorted(1:n,2:4);
        ptsAligned_filtered = pts_sorted(1:n,5:7);

        % Calculate R,t with function from B
        [R,t] = estimateRT_pt2pt(nearest_Filtered, ptsAligned_filtered);
                
        out_estR = out_estR*R;
        out_estt = out_estt-t';
        
        % Adaptive stop criterion
        delta_t = sqrt(sum(t.^2));
        delta_R = rad2deg(acos((R(1,1)+R(2,2)+R(3,3)-1)/2));
        
        if (delta_R <= tolerance(1)) && (delta_t <= tolerance(2))
            break;
        end
    end
    disp([num2str(k) ' iterations'])
end

