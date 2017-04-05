function [ pcroi ] = camROI(pc)
    
    x = pc(1, :);%converting to xs and ys could be avoided if there 
                %was a way to return a subset of a matrix as a matrix
                %not a vector
    y = pc(2, :);
    z = pc(3, :);
    
    roixSize = 0.1;%m
    roizSize = 0.1;
    zOffset = 0.1;
    badZ = -10;
    
    xCriteria = false(1, numel(pc(1, :)));
    zCriteria = xCriteria;
    minZ = min(z(z ~= badZ));
    
    xCriteria = x > -roixSize & x < roixSize;
    zCriteria = z > (minZ + zOffset) & z < (minZ + zOffset + roizSize) & z ~= badZ;
    
    roi = xCriteria & zCriteria;
    
    pcroi = [x(roi); y(roi); z(roi)];
end

