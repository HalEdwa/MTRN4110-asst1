clear
point = [rand() rand() rand()];%[1 2 3];%
normal = [rand() rand() rand()];%[0 1 1];%
noise = 0.3;

p = generateFakePlane(point, normal, noise);
[pFit, nFit, fitresult] = getOrientation(p);
x = p(:, :, 1);
y = p(:, :, 2);
z = p(:, :, 3);
[xData, yData, zData] = prepareSurfaceData( p(:, :, 1), p(:, :, 2), p(:, :, 3) );

%generate normal line:
nLength = 3;
nLine = [point; point + nLength*normal]';
nLineFit = [pFit; pFit - nLength*nFit]';

nFit = nFit / norm(nFit);
pitch = asin(nFit(1));
roll = asin( -nFit(2) / cos(pitch));
yaw = 0;%for now

pitch_ = pitch*180/pi;
roll_ = roll*180 / pi;

rx = roll;
ry = pitch;
rz = yaw;
rotX = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)];
rotY = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)];
rotZ = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1];
rot3Dpt = rotZ * rotY * rotX;

figure(1);clf();
subplot(2, 1, 1)
hold on; axis equal; grid on;
for i = 1:length(p(1,:,1))
    plot3(x(:, i), y(:, i), z(:, i), 'b*')
end
plot( fitresult, [xData, yData], zData );
plot3(pFit(1), pFit(2), pFit(3), 'r*');%point on fitted plane
plot3(nLineFit(1, :), nLineFit(2, :), nLineFit(3, :), 'r-*');
% plot3(nLine(1, :), nLine(2, :), nLine(3, :), 'g-*');
xlabel('x'); ylabel('y'); zlabel('z');
view( -92.5, 6.0 );

subplot(2, 1, 2)
axis equal; grid on; hold on;
tPoints = [xData'; yData'; zData'];

%now transform points:
for i = 1:length(xData)
    tPoints(:, i) = rot3Dpt * tPoints(:, i) - pFit';
end

plot3(tPoints(1, :), tPoints(2, :), tPoints(3, :), 'b*')

[floorX, floorY] = ndgrid(-5:5, -5:5);
floorZ = zeros(size(floorX));
surf(floorX, floorY, floorZ);
plot3(0, 0, 0, 'r*')
xlabel('x'); ylabel('y'); zlabel('z');
view(45, 0);
