function [out_estR,out_estt,err] = ICP(pts, ptsMoved, downsampleStep, correspondences, maxIterations, tolerance, useColour, alpha, usePt2Pl)
    % set point-to-point as default
    if(nargin < 9)
        usePt2Pl = 0;
    end

    vector_size=3;
    % Initialize R and t
    out_estR = eye(3,3);
    out_estt = zeros(3,1);
    
    %Downsample points
    pts_ds = pcdownsample(pts, 'gridAverage', downsampleStep);
    ptsMoved_ds = pcdownsample(ptsMoved, 'gridAverage', downsampleStep);
    
    if(useColour)
        lab1 = rgb2lab(pts_ds.Color)*alpha;
        lab2 = rgb2lab(ptsMoved_ds.Color)*alpha;
    end

    pts_ds = pts_ds.Location;
    ptsMoved_ds = ptsMoved_ds.Location;
    
    for k=1:maxIterations
        ptsAligned = rigidTransform(ptsMoved_ds, out_estR, out_estt);
        if(useColour)
            if(k==1)
                pts_ds = [pts_ds lab1];
                vector_size=6;
            end
            ptsAligned = [ptsAligned lab2];
        end
        
        % Search corresponding nearest points
        [IDX,D] = knnsearch(pts_ds, ptsAligned);

        nearestPoints = zeros(size(IDX,1),vector_size);
        for i=1:size(IDX,1)
            nearestPoints(i,:) = pts_ds(IDX(i),:);
        end

        % Sort points by distance D
        A = [D nearestPoints ptsAligned];
        pts_sorted = sortrows(A, 1);

        % Filter correspondences based on distance
        n = ceil(size(pts_sorted,1)*correspondences);
        nearest_Filtered = pts_sorted(1:n,2:4);
        
        if(useColour)
            ptsAligned_filtered = pts_sorted(1:n,8:10);
        else
            ptsAligned_filtered = pts_sorted(1:n,5:7);
        end
        
        % Calculate R,t with function from G/B
        if(usePt2Pl)
            [R,t] = estimateRT_pt2pl(nearest_Filtered, ptsAligned_filtered);
        else
            [R,t] = estimateRT_pt2pt(nearest_Filtered, ptsAligned_filtered);
        end
            
        out_estR = out_estR*R;
        out_estt = out_estt-t;
        
        % Adaptive stop criterion
        delta_t = sqrt(sum(t.^2));
        delta_R = rad2deg(acos((R(1,1)+R(2,2)+R(3,3)-1)/2));
        
        if (delta_R <= tolerance(1)) && (delta_t <= tolerance(2))
            break;
        end
    end
    err = [delta_t delta_R];
    disp(['usePt2Pl ' num2str(usePt2Pl) '; ' num2str(k) ' iterations'])
end

