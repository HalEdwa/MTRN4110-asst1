clear

t = tcpip('127.0.0.1',15000);
t.ByteOrder = 'littleEndian';
set(t,'InputBufferSize', (2*76800));
fopen(t);
pause(1)
i = 0;

figure(1);
%Image = imshow(zeros(120,160), []);
Image = imshow(zeros(120,160), [0 31999])%, 'Colormap',jet(255));

figure(3);
ImageCenter = imshow(zeros(1,160), 'Colormap', jet(255));

figure(2); clf(); hold on;
%OOI = plot(0, 0)
Point = plot(0,0,'b.');
guiH.OOI = plot(0,0,'g.', 'MarkerSize', 30);
axis([0, 160, 0, 3]);


while t.BytesAvailable == 0
    pause(1)
end

height = 120;
width = 160;

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
              ooiArray(j) = DataReceivedXYZ(k);
          end
        end
    end

    imageArray(imageArray < 0) = 0;
    %imageArray(imageArray > 30000) = -10
    ooiArray = ooiArray/1000;  
    X = 1:length(ooiArray);
    set(Image, 'CData', imageArray);
    ooiArray(ooiArray < 0) = -10;    %Valued to be disregarded
    ooiArray(ooiArray > 3) = -10;   %Value for depth too far away
    set(Point, 'xdata', X, 'ydata', ooiArray);
    %figure(4);
    %plot(X, ooiArray, 'b.')
    set(ImageCenter, 'CData', ooiArray);
    %ooiArray(75:85) %Print Center section of center row
    
    OOI = ExtractOOIs(ooiArray);
    PlotOOIs(OOI, guiH);
    
    pause(0.01);
end

pause(1)

fclose(t);
delete(t);
clear t

function r = ExtractOOIs(ranges)
    PoleDia = 10;%0.2;
    poleDiaTol = 95;
    r.N = 0;
    r.Centers.x = [];
    r.Centers.y = [];
    r.Sizes   = [];
    clusterEndPts = [1];%have to initialise

    X = [1:length(ranges)];
    Y = ranges;
    
    %find all clusters of points:
    for i = 2:length(ranges) - 1
        if (X(i) - X(i-1))^2 + (Y(i) - Y(i-1))^2 > PoleDia^2
            if (X(i) - X(i+1))^2 + (Y(i) - Y(i+1))^2 > PoleDia^2
                clusterEndPts = [clusterEndPts, i - 1, i + 1];
                i = i + 1;
                disp('yis')
            else 
                disp('awyis')
                clusterEndPts = [clusterEndPts, i - 1, i];
            end
        end
    end

    clusterEndPts
    
    lineColor = [0 1 1];
    for i = 1:2:length(clusterEndPts) - 1
%         if ~any(find(brightPts > clusterEndPts(i) & brightPts < clusterEndPts(i+1)))
%             continue
%         end
        objectSize = pdist([X(clusterEndPts(i)),  Y(clusterEndPts(i)); X(clusterEndPts(i+1)),  Y(clusterEndPts(i+1))]);
        if  (objectSize > (PoleDia*(1 + poleDiaTol )))||( objectSize < (PoleDia*(1 - poleDiaTol)))
            continue
        end
        r.N = r.N + 1;
        r.Centers.x = [r.Centers.x mean(X(clusterEndPts(i):clusterEndPts(i+1)))];
        r.Centers.y = [r.Centers.y mean(Y(clusterEndPts(i):clusterEndPts(i+1)))];
        r.Sizes = [r.Sizes pdist( [X(clusterEndPts(i)), Y(clusterEndPts(i));
                           X(clusterEndPts(i+1)), Y(clusterEndPts(i+1))])];
%         plot(X(clusterEndPts(i):clusterEndPts(i+1)), Y(clusterEndPts(i):clusterEndPts(i+1)), 'Color', hsv2rgb(lineColor), 'Marker', '+')%, 'LineStyle', 'none'
%         lineColor(1) = mod(lineColor(1) + pi, 1);
%         axis([-10,10,0,20]);
    %     pause(0.1)
    end
%     plot(X, Y, 'k.')
%     plot(X(brightPts), Y(brightPts), 'r.')
return;
end

function PlotOOIs(OOIs, guiHandle)
    if OOIs.N<1, return ; end;
    
    set(guiHandle.OOI, 'xdata', OOIs.Centers.x(:), 'ydata', OOIs.Centers.y(:));
    OOIs.Centers.x(:)
    OOIs.Centers.y(:)
%     theta = 0:0.1:2*pi;
%     
%     for i = 1:OOIs.N
%         circX = OOIs.Centers.x(i) + OOIs.Sizes(i)*cos(theta);
%         circY = OOIs.Centers.y(i) + OOIs.Sizes(i)*sin(theta);
%         set(guiHandle.handle1, 'xdata', circX, 'ydata', circY);
%     end
    
return;
end