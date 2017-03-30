clear

height = 120;%received image dimensions
width = 160;

ip_address = '127.0.0.1';
remote_port = 15000;
MaxTimeout = 1000; 
Timer = 0;
MaxDist = 1;
MaxRecordSize = 100;

t = tcpip(ip_address,remote_port);%Initiate TCP connection
t.ByteOrder = 'littleEndian';%Set Endian to convert
set(t,'InputBufferSize', width*height*3*2);

fopen(t);
pause(1)

figure(1);
guiH.DepthVisualisation = imagesc();
colorbar;
caxis([0 1]);
axis([0 160 0 120]);

figure(2); clf(); 
guiH.Vertices = plot3(0,0,0,'.', 'MarkerSize', 2);
axis([0 1 -0.4 0.4 -0.4 0.4]);
xlabel('Z Depth'); ylabel('X Horizontal'); zlabel('Y Vertical');
zoom on ; grid on;

figure(3); clf(); hold on;
guiH.DepthScan = plot(0,0,'b.');   %Depth map at horizon scatterplot handle
guiH.Marker = plot(0,0,'g.','MarkerSize',20);  %Object of interest marker overlay Handle
axis([-0.4 0.4 0 1]);    %in meters

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
    
    x = buff(1:19200);
    y = buff(19201:38400);
    z = buff(38401:57600);
    
    x = x/1000; y = y/1000; z = z/1000;  %Convert from mm to m
    z(z < 0) = -10;    %Negative depths to be disregarded
    z(z > MaxDist) = -10;   %Value for depth too far away disregarded
    
    %record a rosbag
    if (rosbagFrame < MaxRecordSize)
        rosbagXYZ(rosbagFrame).x = x;
        rosbagXYZ(rosbagFrame).y = y;
        rosbagXYZ(rosbagFrame).z = z;
    else 
        disp('Rosbag Full');
    end
    
    DepthMap = reshape(z,[160,120]);
    %rotate image by 90 degree
    DepthMap = DepthMap';
    %flip image upside down
    DepthMap = flipud(DepthMap);   
    
    xScan = x(y==0);
    zScan = z(y==0);
    
    OOIs = ExtractOOIs_cam(xScan, zScan);
    
    %Display necessary plots
    set(guiH.DepthVisualisation, 'CData', DepthMap);
    set(guiH.Vertices, 'xdata', z, 'ydata', x, 'zdata', y);
    set(guiH.DepthScan, 'xdata', xScan, 'ydata', zScan);
    PlotOOIs(OOIs, guiH.Marker);
    
    pause(0.01);    %~10ms delay
end

%program end
pause(1)
fclose(t);
delete(t);
clear t