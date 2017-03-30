function [ roix, roiy, roiz ] = camROI( x, y, z )
    
    
    roiW = 50;
    roiH = 50;

    %it's a lot easier to extract the ROI and then discard bad points:
    imgSize = [160 120];
    badZ = -10;%the value z is set to for a bad point;
    
    x = reshape(x, imgSize);
    y = reshape(y, imgSize);
    z = reshape(z, imgSize);
    
    roi = zeros(imgSize(x));
    roi( 0:roiH, (imgSize(roi, 2)/2 - roiW/2):(imgSize(roi, 2) + roiW/2))
    
    x = x(roi);
    y = y(roi);
    y = z(roi);
    
    roix = reshape(x, [1, numel(x)]);
    yroi = reshape(y, [1, numel(y)]);
    roiz = reshape(z, [1, numel(z)]);
    
    roix = roix(z ~= badZ);
    yroi = yroi(z ~= badZ);
    roiz = roiz(z ~= badZ);
end

