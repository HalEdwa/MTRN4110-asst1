clear

t = tcpip('127.0.0.1',15000);
t.ByteOrder = 'littleEndian';
set(t,'InputBufferSize', (2*76800));
fopen(t);
pause(1)
i = 0;

figure(1);
%Image = imshow(zeros(120,160), []);
Image = imshow(zeros(120,160))%, 'Colormap',jet(255));

figure(3);
ImageCenter = imshow(zeros(1,160), 'Colormap', jet(255));

figure(2);
%OOI = plot(0, 0)
guiH.OOI = plot(0,0,'.b');
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
    imageArray(imageArray > 30000) = -10
    ooiArray = ooiArray/1000;  
    X = 1:length(ooiArray);
    set(Image, 'CData', imageArray/31999);
    ooiArray(ooiArray < 0) = -10;    %Valued to be disregarded
    ooiArray(ooiArray > 3) = -10;   %Value for depth too far away
    set(guiH.OOI, 'xdata', X, 'ydata', ooiArray);
    set(ImageCenter, 'CData', ooiArray);
    ooiArray(75:85) %Print Center section of center row
    
    pause(0.01);
end

pause(1)

fclose(t);
delete(t);
clear t