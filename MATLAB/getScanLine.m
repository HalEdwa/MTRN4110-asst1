function [ line ] = getScanLine( c, lineHeight )
%GETSCANLINE returns a line 

roi = abs(c(3, :)) < lineHeight;

x = c(1,:);%I hate doing this
y = c(2,:);
z = c(3,:);

line = [x(roi); y(roi); z(roi)];
end

