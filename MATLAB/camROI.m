function [ pcroi ] = camROI(pc)
    
    x = pc(1, :);%converting to xs and ys could be avoided if there 
                %was a way to return a subset of a matrix as a matrix
                %not a vector
    y = pc(2, :);
    z = pc(3, :);
    
    roiySize = 0.1;%m
    roixSize = 0.1;
    xOffset = 0.2;
    badX = -10;
    
    minX = min(x(x ~= badX));
    
    yCriteria = y > -roiySize & y < roiySize;
    xCriteria = x > (minX + xOffset) & x < (minX + xOffset + roixSize) & x ~= badX;
    roi = yCriteria & xCriteria;
    
    pcroi = [x(roi); y(roi); z(roi)];
end

