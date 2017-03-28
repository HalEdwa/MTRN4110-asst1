clear

height = 120;%received image dimensions
width = 160;
Range = 31999;
fovx = 74*pi/180;%horizontal field of view of the camera

ip_address = '127.0.0.1';
remote_port = 15000;
Timer = 0;
MaxTimeout = 1000; 
colourRange = 255;

MinX = -1; MaxX = 1; MinDist = 0; MaxDist = 3;

t = tcpip(ip_address,remote_port);%Initiate TCP connection
t.ByteOrder = 'littleEndian';%Set Endian to convert
set(t,'InputBufferSize', (2*240*320));%*width*height));
fopen(t);
pause(1)

figure(1);
%DepthVisualisation = imshow(zeros(120,160), []);    %Depth visualisation in GrayScale
DepthVisualisation = imshow(zeros(height,width),'Colormap',jet(colourRange)); %Depth visualisation in Colormap

figure(2);
DepthHorizon = imshow(zeros(1,width), 'Colormap', jet(colourRange));%center row of depthMap for now will be horizon

figure(3); clf(); hold on;
guiH.DepthScan = plot(0,0,'b.');   %Depth map at horizon scatterplot handle
guiH.OOI = plot(0,0,'r*');  %Object of interest marker overlay Handle
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
    
    DataReceivedXYZ = fread(t,width*height,'uint16');  %Read one depthmap frame
    
    counter = 1;
    
    for i = 1 : height
        for j = 1: width
          DepthMap(i,j) = DataReceivedXYZ(counter); %Capture depth for respective pixel
          counter = counter + 1;
          
          if i == height/2
              yDist(j) = DataReceivedXYZ(counter);    %Capture depth at horizon row and column j
          end
        end
    end
    
    yDist = yDist/1000;  %Convert from mm to m
    yDist(yDist < 0) = -1;    %Negative depths to be disregarded
    yDist(yDist > MaxDist) = -1;   %Value for depth too far away disregarded
    
    %estimate world x-coordinates of pixels:
    xDist = yDist*tan(fovx/2).*(-width/2 : (width - 1)/2)/width;%width - 1 for off-by one error
    
    DepthMap(DepthMap < 0) = 0; %Negative depths to be disregarded
    OOIs = ExtractOOIs_cam(xDist, yDist);

    %Display necessary plots
    set(DepthVisualisation, 'CData', DepthMap/Range);   %divide by range to scale between 0-1 for colormpa
    set(DepthHorizon, 'CData', yDist);
    set(guiH.DepthScan, 'xdata', xDist, 'ydata', yDist);
    PlotOOIs(OOIs, guiH.OOI);
    
    pause(0.01);    %~10ms delay
end

%program end
pause(1)
fclose(t);
delete(t);
clear t