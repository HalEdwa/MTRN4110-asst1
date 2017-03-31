function [ pcroi ] = camROI(pc)
    
    
    roiW = 50;
    roiH = 50;

    %it's a lot easier to extract the ROI and then discard bad points:
    imgSize = [160 120];
    badZ = -10;%the value z is set to this when it's bad
    
    x = reshape(pc(1, :), imgSize);
    y = reshape(pc(2, :), imgSize);
    z = reshape(pc(3, :), imgSize);
    
    roi = false(imgSize);
    roi( 1:roiH, (size(roi, 2)/2 - roiW/2):(size(roi, 2)/2 + roiW/2)) = true;
    
    x = x(roi);
    y = y(roi);
    z = z(roi);
    
    roix = reshape(x, [1, numel(x)]);
    roiy = reshape(y, [1, numel(y)]);
    roiz = reshape(z, [1, numel(z)]);
    
    roix = roix(roiz ~= badZ);
    roiy = roiy(roiz ~= badZ);
    roiz = roiz(roiz ~= badZ);
%     figure(3);clf();
%     xlabel('x'); ylabel('y'); zlabel('z');
%     hold on; axis equal; grid on;
%     scatter3(roix, roiy, roiz, 'g.');
    pcroi = [roix; roiy; roiz];
end

