clear

t = tcpip('127.0.0.1',15000);
t.ByteOrder = 'littleEndian';
set(t,'InputBufferSize', (2*76800));
fopen(t);
pause(1)
i = 0;

figure(1);
%Image = imshow(zeros(120,160), []);
Image = imshow(zeros(120,160));%, 'Colormap',jet(255));

figure(3);
ImageCenter = imshow(zeros(1,160), 'Colormap', jet(255));

figure(2); clf(); hold on;
Point = plot(0,0,'b.');
guiH.OOI = plot(0,0,'r*');
axis([-1, 1, 0, 3]);


while t.BytesAvailable == 0
    pause(1)
    disp('waiting for initial bytes...');
end

height = 120;
width = 160;
fovx = 74*pi/180;%the horizontal fov of the camera
Timeout = 0;

while ((Timeout < 100000) || (get(t, 'BytesAvailable') > 0)) 
    if(t.BytesAvailable > 0)
        Timeout = 0;
    end
    
    DataReceivedXYZ = fread(t,76800,'uint16');
    
    k = 19280;
    counter = 1;
    
    for i = 1 : height
        for j = 1: width
          if counter > 160
                 k = k + 160;
                 counter = 1;
          end

          imageArray(i,j) = DataReceivedXYZ(k);
          k = k + 1;
          counter = counter + 1;
          
          if i == height/2
              yDist(j) = DataReceivedXYZ(k);
          end
        end
    end
    
    yDist = yDist/1000;  
    yDist(yDist < 0) = -1;    %Valued to be disregarded
    yDist(yDist > 3) = -1;   %Value for depth too far away
    %now to calculate actual x coordinates of pixels:

    xDist = yDist*tan(fovx/2).*(-width/2 : (width - 1)/2)/width;%width - 1 for off-by one error
    
    imageArray(imageArray < 0) = 0;
    OOIs = ExtractOOIs_cam(xDist, yDist);

    set(Image, 'CData', imageArray/31999);
    set(Point, 'xdata', xDist, 'ydata', yDist);
    set(ImageCenter, 'CData', yDist);
    
    PlotOOIs(OOIs, guiH.OOI);
    
    pause(0.01);
end

pause(1)

fclose(t);
delete(t);
clear t