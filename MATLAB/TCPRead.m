clear
 
height = 120;%received image dimensions
width = 160;

ip_address = '127.0.0.1';
remote_port = 15000;
MaxTimeout = 1000; 
Timer = 0;
MaxDist = 3;
MaxRecordSize = 100;

t = tcpip(ip_address,remote_port);%Initiate TCP connection
t.ByteOrder = 'littleEndian';%Set Endian to convert

fopen(t);
pause(1)

close all;
% figure(1); hold on;
% guiH.DepthVisualisation = imagesc();
% colorbar;
% caxis([0 1]);
% axis([0 160 0 120]);

figure(2); hold on;zoom on ; grid on; axis equal;
guiH.Vertices = plot3(0,0,0,'.');
guiH.roi = scatter3(0, 0, 0, 'g');
guiH.normalLine = plot3(0, 0, 0, 'r');
xlabel('x'); ylabel('y'); zlabel('z');
title('untransformed point cloud data');
axis([0 1 -0.7 0.7 -0.7 0.7]);
view(225, 15);


figure(3); hold on;
guiH.DepthScan = plot(0,0,'b.');   %Depth map at horizon scatterplot handle
guiH.Marker = plot(0,0,'g.','MarkerSize',20);  %Object of interest marker overlay Handle
title('scan of middle row');

figure(4); hold on; axis equal; zoom on;grid on;
guiH.pct = scatter3(0, 0, 0, 'b.');
guiH.roit = scatter3(0, 0, 0, 'g');
guiH.scanLine = scatter3(0, 0, 0, 'r');
xlabel('x'); ylabel('y'); zlabel('z');
title('transformed pts');
view(90, 0);

rosbagXYZ = repmat(struct('x', [], 'y', [], 'z', []),1,150);
rosbagFrame = 0;

while t.BytesAvailable == 0 %Wait for incoming bytes
    pause(1)
    disp('waiting for initial bytes...');
end

disp('Connected to server');

while ((Timer < MaxTimeout) || (get(t, 'BytesAvailable') > 0))         
    if(t.BytesAvailable > 0)    %if connected
        Timer = 0; %reset timer
    end
    
    buff = fread(t, height*width*3, 'int16');
    Timer = Timer + 1;
    rosbagFrame = rosbagFrame + 1;
    
    y = buff(1:19200);%Y X Z
    z = buff(19201:38400);
    x = buff(38401:57600);
    
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
    
    pc = [x'; y'; z'];
    roi = camROI(pc);
    if numel(roi) < 20*3
        pause(0.01);
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
    set(guiH.DepthScan, 'xdata', sl(1, :), 'ydata', sl(2, :));
%     PlotOOIs(OOIs, guiH.Marker);

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
 
    
    %%
    %record a rosbag
%     if (rosbagFrame < MaxRecordSize)
%         rosbagXYZ(rosbagFrame).x = x;
%         rosbagXYZ(rosbagFrame).y = y;
%         rosbagXYZ(rosbagFrame).z = z;
%     else 
%         disp('Rosbag Full');
%     end
    pause(0.1);    %~10ms delay
end

%program end
pause(1)
fclose(t);
delete(t);
clear t