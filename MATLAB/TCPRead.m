clear

height = 120;%received image dimensions
width = 160;
fovx = 74*pi/180;%horizontal field of view of the camera

ip_address = '127.0.0.1';
remote_port = 15000;
MaxTimeout = 1000; 
Timer = 0;

MinX = -0.5; MaxX = 0.5; MinDist = 0; MaxDist = 1;

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
axis([0 1 -1 1 -1 1]);
xlabel('Z Depth'); ylabel('X Horizontal'); zlabel('Y Vertical');
zoom on ; grid on;

figure(3); clf(); hold on;
guiH.DepthScan = plot(0,0,'b.');   %Depth map at horizon scatterplot handle
guiH.Marker = plot(0,0,'g.','MarkerSize',20);  %Object of interest marker overlay Handle
axis([MinX, MaxX, MinDist, MaxDist]);    %in meters

while t.BytesAvailable == 0 %Wait for incoming bytes
    pause(1)
    disp('waiting for initial bytes...');
end

disp('Connected to server');

while ((Timer < MaxTimeout) || (get(t, 'BytesAvailable') > 0))         
    if(t.BytesAvailable > 0)    %if connected
        Timer = 0; %reset timer
    end
    
    Timer = Timer + 1;
    
    %buff = fread(t, height*width*3, 'int16');
     
    %x = buff(1:19200)
    %y = buff(19201:38400)
    %z = buff(38401:57600)
    
    z = fread(t,width*height,'int16');  %Read one depthmap frame
    
    temp = [-79:80]/80;
    temp1 = [-59:60]/60;
    x = repmat(temp', 120, 1);
    y = repmat(temp1, 160, 1);
    y = y(:);
    
    z = z/1000;  %Convert from mm to m
    z(z < 0) = -10;    %Negative depths to be disregarded
    z(z > MaxDist) = -10;   %Value for depth too far away disregarded
    
    DepthMap = reshape(z,[160,120]);
    %rotate image by 90 degree
    DepthMap = DepthMap';
    %flip image upside down
    DepthMap = flipud(DepthMap);   
    
    zScan = (z((160*60 + 1):(160*61)))';
    %estimate world x-coordinates of pixels:
    xScan = zScan*tan(fovx/2).*(-width/2 : (width - 1)/2)/width;;%width - 1 for off-by one error
    
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