function TCPRead()
    clear
    
    load('recordedCameraData_20170503.mat')
    
    height = 120;%received image dimensions
    width = 160;
    Live = 0;

    ip_address = '127.0.0.1';
    remote_port = 15000;
    MaxTimeout = 1000; 
    Timer = 0;
    MaxDist = 5;
    MaxRecordSize = 100;

    if (Live == 1)
        t = tcpip(ip_address,remote_port);%Initiate TCP connection
        t.ByteOrder = 'littleEndian';%Set Endian to convert
        set(t,'InputBufferSize', width*height*3*2);

        fopen(t);
    end
    
    pause(1)
    close all;
    
    figure(1); hold on;
    guiH.DepthVisualisation = imagesc();
    colorbar;
    caxis([0 1]);
    axis([0 160 0 120]);

     figure(2); 
    % subplot(1, 2, 1)
    hold on;zoom on ; grid on; axis equal;
    guiH.Vertices = plot3(0,0,0,'.');
    guiH.roi = scatter3(0, 0, 0, 'g');
    guiH.normalLine = plot3(0, 0, 0, 'r');
    xlabel('x'); ylabel('y'); zlabel('z');
    title('untransformed point cloud data');
    axis([0 1 -0.7 0.7 -0.7 0.7]);
    view(225, 15);

    figure(4); 
    % subplot(1, 2, 2)
    hold on; axis equal; zoom on;grid on;
    guiH.pct = scatter3(0, 0, 0, 'b.');
    guiH.roit = scatter3(0, 0, 0, 'g');
    guiH.scanLine = scatter3(0, 0, 0, 'r');
    xlabel('x'); ylabel('y'); zlabel('z');
    axis([-1 1 -1 1 -0.5 0.5]);
    title('transformed pts');
    view(90, 0);

    fig3 = figure(3); hold on; axis equal
    guiH.DepthScan = scatter(0,0,25,[0 0 0]);   %Depth map at horizon scatterplot handle
    guiH.Marker = scatter(0,0,'r*');  %Object of interest marker overlay Handle
    set(fig3, 'position', [30 30 800 800])
    axis([0 4 -2 2]);
    title('scan of horizon');
    xlabel('x'); ylabel('y');
    
    figure(6); hold on; axis equal
    GlobalMap.Landmarks = plot(0,0,'r*');   %Depth map at horizon scatterplot handle
    title('Global Map');
    xlabel('x'); ylabel('y');
    
    %recordedData = zeros(3, 19200, 200);   % Turn on if recording
    
    idx = 1;

    if (Live == 1)
        while t.BytesAvailable == 0 %Wait for incoming bytes
            pause(1)
            disp('waiting for initial bytes...');
        end
    end
    
    disp('Connected to server');
    figure(5);
    guiH.og = surf(zeros(60));
    og = OccupancyGrid(3, 3, 0.05);
    xlabel('x'); ylabel('y'); zlabel('z');

    X = [0;0;pi/2];    % Vehicle Pose
    
    if (Live == 1)
        RunCondition = (Timer < MaxTimeout) || (get(t, 'BytesAvailable') > 0);
    else
        RunCondition = 1;
        iteration = 1;
    end
    
    while (RunCondition)
        if (Live == 1)
            if(t.BytesAvailable > 0)    %if connected
                Timer = 0; %reset timer
            end

            buff = fread(t, height*width*3, 'int16');
            Timer = Timer + 1;
        
            %Live Data
            y = buff(1:19200);%Y X Z
            z = buff(19201:38400);
            x = buff(38401:57600);
        else
            %Play recorded data
            y = (recordedData(2,:,iteration))';
            z = (recordedData(3,:,iteration))';
            x = (recordedData(1,:,iteration))';
        
            iteration = iteration + 1;
        end
        
        %uncomment to record some camera data:
    %     recordedData(1, :, idx) = x;
    %     recordedData(2, :, idx) = y;
    %     recordedData(3, :, idx) = z;
    %     idx = idx + 1;
    %     if idx == length(recordedData(1, 1, :))
    %         break;
    %     end

        x = x/1000; y = y/1000; z = z/1000;  %Convert from mm to m
        x(x < 0) = -10;    %Negative depths to be disregarded

        %why does the below line cause the data to go bananas?
        x(x > MaxDist) = -10;   %Value for depth too far away disregarded
        
        % Create image to display live camera feed
        DepthMap = reshape(x,[160,120]);
        %rotate image by 90 degree
        DepthMap = DepthMap';
        %flip image upside down
        DepthMap = flipud(DepthMap);   
        
        y = y(x ~= -10);
        z = z(x ~= -10);
        x = x(x ~= -10);
        
        if (Live == 1)
            pc = [x; y; z];
        else
            pc = [x'; y'; z'];
        end
        
        roi = camROI(pc);
        
        set(guiH.Vertices, 'xdata', x, 'ydata', y, 'zdata', z);
        set(guiH.DepthVisualisation, 'CData', DepthMap);

        if numel(roi) < 20*3
            pause(0.1);
            disp(strcat('Not enough pts: ', num2str(rand())));
            set(guiH.normalLine, 'xdata', 0, 'ydata', 0, 'zdata', 0);
            set(guiH.roi, 'xdata', 0, 'ydata', 0, 'zdata', 0);

            continue
        end
        
        [~, n, ~] = getOrientation(roi);
        pct = cloudTransform(pc, n);
        roit = cloudTransform(roi, n);
        sl = getScanLine(pct, 0.005);
        
        % Plot Local frame depth camera scan data and transform
        ExtractOOIs_cam(sl(1, :), sl(2, :), guiH.DepthScan);
        
        %create a line to visualise n:
        nLine = [roi(:, 1), roi(:, 1) + n'*0.2/(norm(n))];
        set(guiH.normalLine, 'xdata', nLine(1, :), 'ydata', nLine(2, :), 'zdata', nLine(3, :));
    %     plot3(nLine(1, :), nLine(2, :), nLine(3, :), 'linewidth', 10);

        set(guiH.pct, 'xdata', pct(1, :), 'ydata', pct(2, :), 'zdata', pct(3, :));
    %     scatter3(pct(1, :), pct(2, :), pct(3, :), 'b.');
        set(guiH.roi, 'xdata', roi(1, :), 'ydata', roi(2, :), 'zdata', roi(3, :));
        set(guiH.roit, 'xdata', roit(1, :), 'ydata', roit(2, :), 'zdata', roit(3, :))
    %     scatter3(roit(1, :), roit(2, :), roit(3, :), 'r*');
   
        %transform sl according to position before adding observations
        % Form rotation matrixes
        theta = X(3) - pi/2;
        rotationMatrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];

        % Apply rotation matrix to align data with heading
        V = rotationMatrix*[sl(1, :); sl(2, :)];
        xVals = V(1,:);
        yVals = V(2,:) + X(2);
        
        xVals = xVals + X(1);
        yVals = yVals + X(2);
        
        %set(guiH.scanLine, 'xdata', sl(1, :), 'ydata', sl(2, :), 'zdata', sl(3, :));
        set(guiH.scanLine, 'xdata', xVals, 'ydata', yVals, 'zdata', sl(3, :));
        
        og.addObservations(xVals, yVals);
        %og.addObservations(sl(1, :), sl(2, :));
        og.visualise(guiH.og);
        og.decrement();
        
        OOIs = og.FindOOIs();
        
        %Global Frame Plot of Vehicle and Landmarks
        set(GlobalMap.Landmarks, 'xdata', OOIs.centers.x(:), 'ydata', OOIs.centers.y(:));
        
        X = Localise(OOIs, X)  %update pose based on "known" locations of OOIS
        
        %%
        
        if (Live == 0)
            RunCondition = (Timer < MaxTimeout) || (get(t, 'BytesAvailable') > 0);
            pause(0.2);    %~200ms delay
        end
    end

    %program end
    if (Live == 0)
        pause(1)
        fclose(t);
        delete(t);
        clear t
    end
end

function X = Localise(OOIs, X_Last)
        % Transform by heading
        Landmarks.x = OOIs.centers.x + X_Last(1);    % Global x of landmark
        Landmarks.y = OOIs.centers.y + X_Last(2);    % Global y of landmark
        Landmarks.n = OOIs.N;  % Number of landmarks
        
        for i = 1:OOIs.N
            Landmarks.r(i) = sqrt(OOIs.centers.y(i).^2 + OOIs.centers.x(i).^2); % Local Range
            
            Landmarks.theta(i) = pi/2 + atan2(OOIs.centers.y(i),OOIs.centers.x(i));   % Local Azimuth Bearing
        end
        
        options = optimset('MaxFunEvals',1000);
        
        if (Landmarks.n > 1)   %Triangulate and localise
            X = fminsearch(@(X) Triangulate(X,Landmarks),[X_Last(1),X_Last(2),X_Last(3)], options); % Triangulate and return x, y, phi
        else
            X = X_Last;
        end
        
        Pose = [X(1); X(2); rad2deg(X(3))];
end

function F = Triangulate(X,OOI)
    F = zeros(OOI.n * 2, 1);
    u = 1;
    
    for i = 1:OOI.n
        F(u) = sqrt((OOI.x(i) - X(1))^2 + (OOI.y(i) - X(2)^2)) - OOI.r(i);
        F(u + 1) = atan2(OOI.y(i) - X(2), OOI.x(i) - X(1)) - X(3) - OOI.theta(i);
        u = u + 2;
    end
    F = sum(F.^2);
end