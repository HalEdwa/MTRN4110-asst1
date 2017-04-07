function [ pcroi ] = camROI(pc)
    
    x = pc(1, :);%converting to xs and ys could be avoided if there 
                %was a way to return a subset of a matrix as a matrix
                %not a vector
    y = pc(2, :);
    z = pc(3, :);
    
    roiySize = 0.1;%m
    roixSize = 0.1;
    xOffset = 0.1;
    badX = -10;
    floorNoise = 0.1;
    
    yCriteria = false(1, numel(pc(1, :)));
    xCriteria = yCriteria;
    minX = min(x(x ~= badX));
    
    yCriteria = y > -roiySize & y < roiySize;
    xCriteria = x > (minX + xOffset) & x < (minX + xOffset + roixSize) & x ~= badX;
    zCriteria = z < (min(z) + floorNoise);
    roi = yCriteria & xCriteria & zCriteria;
    
    pcroi = [x(roi); y(roi); z(roi)];
end

