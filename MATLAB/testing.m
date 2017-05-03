%%
%	THIS DOCUMENT IS FOR SANDBOXING CHANGES, NOT SERIOUS WORK
%	NOTHING IN IT IS SAFE 
%%
%%loaded from file:
close all
load('recordedCameraData_20170407.mat')
width = 160;
height = 120;
MaxDist = 3;

%% 
subplot(1, 2, 1)
hold on;zoom on ; grid on; axis equal;
guiH.Vertices = plot3(0,0,0,'.');
guiH.roi = scatter3(0, 0, 0, 'g');
guiH.normalLine = plot3(0, 0, 0, 'r');
xlabel('x'); ylabel('y'); zlabel('z');
title('untransformed point cloud data');
axis([0 1 -0.7 0.7 -0.7 0.7]);
view(225, 15);

% figure(4); 
subplot(1, 2, 2)
hold on; axis equal; zoom on;grid on;
guiH.pct = scatter3(0, 0, 0, 'b.');
guiH.roit = scatter3(0, 0, 0, 'g');
guiH.scanLine = scatter3(0, 0, 0, 'r');
xlabel('x'); ylabel('y'); zlabel('z');
title('transformed pts');
view(90, 0);

fig3 = figure(3); hold on; axis equal
guiH.DepthScan = scatter(0,0,25,[0 0 0]);   %Depth map at horizon scatterplot handle
guiH.Marker = scatter(0,0,'r*');  %Object of interest marker overlay Handle
set(fig3, 'position', [30 30 800 800])
axis([0 1 -0.5 0.5]);
title('scan of middle row');
xlabel('x'); ylabel('y');
%% 
set(gcf,'currentchar',' ');         % set a dummy character
    
%i = 39 breaks the fitting tool for some reason
for i = 72;%52:99 %i=31 is a frams with no bad points
    
    x = recordedData(1, :, i);
    y = recordedData(2, :, i);
    z = recordedData(3, :, i);
    
    y = y(x ~= -10);
    z = z(x ~= -10);
    x = x(x ~= -10);
    
%     
    x = x/1000; y = y/1000; z = z/1000;  %Convert from mm to m
    x(x < 0) = -10;    %Negative depths to be disregarded
    
    %why does the below line cause the data to go bananas?
       x(x > MaxDist) = -10;   %Value for depth too far away disregarded
%     
%     %plot depthmap before bad points are removed
%     DepthMap = reshape(z,[160,120]);
%     %flip image upside down and rotate 90 deg
%     DepthMap = flipud(DepthMap');   
% %     set(guiH.DepthVisualisation, 'CData', DepthMap);
%     
    y = y(x ~= -10);
    z = z(x ~= -10);
    x = x(x ~= -10);
    
    pc = [x; y; z];
    roi = camROI(pc);
        % plotting and transformation of live camera data:
    set(guiH.Vertices, 'xdata', x, 'ydata', y, 'zdata', z);

    if numel(roi) < 20*3
        pause(0.1);
        disp('not enough pts');
        continue
    end
    [~, n, ~] = getOrientation(roi);
    pct = cloudTransform(pc, n);
    roit = cloudTransform(roi, n);
    sl = getScanLine(pct, 0.005);

    % plotting and transformation of live camera data:
    set(guiH.Vertices, 'xdata', x, 'ydata', y, 'zdata', z);
    xScan = x(y==0);
    zScan = z(y==0);
%     set(guiH.DepthScan, 'xdata', sl(1, :), 'ydata', sl(2, :));
    OOIs = ExtractOOIs_cam(sl(1, :), sl(2, :), guiH.DepthScan);
    set(guiH.Marker, 'xdata', OOIs.centers.x, 'ydata', OOIs.centers.y);

    %%
    set(guiH.scanLine, 'xdata', sl(1, :), 'ydata', sl(2, :), 'zdata', sl(3, :));
    %create a line to visualise n:
    nLine = [roi(:, 1), roi(:, 1) + n'*0.2/(norm(n))];
    set(guiH.normalLine, 'xdata', nLine(1, :), 'ydata', nLine(2, :), 'zdata', nLine(3, :));
%     plot3(nLine(1, :), nLine(2, :), nLine(3, :), 'linewidth', 10);
    
    set(guiH.pct, 'xdata', pct(1, :), 'ydata', pct(2, :), 'zdata', pct(3, :));
%     scatter3(pct(1, :), pct(2, :), pct(3, :), 'b.');
    set(guiH.roi, 'xdata', roi(1, :), 'ydata', roi(2, :), 'zdata', roi(3, :));
    set(guiH.roit, 'xdata', roit(1, :), 'ydata', roit(2, :), 'zdata', roit(3, :))
%     scatter3(roit(1, :), roit(2, :), roit(3, :), 'r*');
 
%     pause
    if get(gcf,'currentchar')~=' '
        break;
    end
    
    
    
    
end

%% generated randomly:
%pass into generateFakePlane:
%%This version generates a fake plane, based on the specified point and normal
%%vector for use when testing algorithms
% point = [rand() rand() rand()];%[1 2 3];%
% normal = [rand() rand() rand()];%[0 1 1];%
% noise = 0.3;
% p = generateFakePlane(point, normal, noise);








