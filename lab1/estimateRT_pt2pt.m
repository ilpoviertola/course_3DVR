function [R,t] = estimateRT_pt2pt(pts, ptsMoved)
    % Calculate centroids
    ptsCentroid=mean(pts);
    ptsMovedCentroid=mean(ptsMoved);

    % Re-center datasets to origin
    ptsOrigin=pts-ptsCentroid;
    ptsMovedOrigin=ptsMoved-ptsMovedCentroid;
    
    % Covariance
    covariance=transpose(ptsOrigin)*ptsMovedOrigin;
    
    % Calculate rotation with SVD
    [U,S,V]=svd(covariance);
    R=V*transpose(U);
    % Check if negative
    if(det(R)<0)
        R=V*([1 0 0; 0 1 0; 0 0 -1]*transpose(U));
    end
    
    % Find translation
    centroidRotated=R*transpose(ptsCentroid);
    t=transpose(ptsMovedCentroid)-centroidRotated;

end

