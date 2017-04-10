function [cTran] = cloudTransform(c, n)
    %%usage: c = [xVals; yVals; zVals], n = normal vector(orientation)
%     [~, n, ~] = getOrientation(c(1, :), c(2, :), c(3, :));

    n = n / norm(n);
    pitch = asin(n(1));
    roll = asin( -n(2) / cos(pitch));
    yaw = 0;%can't be calculated from a plane

    rx = roll;
    ry = pitch;
    rz = yaw;
    
    pitchDeg = pitch*180/pi
    rollDeg = roll*180/pi
    %matrix taken from lecture notes
    rotX = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)];
    rotY = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)];
    rotZ = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1];
    rot3Dpt = rotZ * rotY * rotX;
    
    cTran = rot3Dpt*c;
%     for i = 1:size(c, 2)
%         c(:, i) = rot3Dpt * c(:, i);
%     end
    
%     cTran = c;
end