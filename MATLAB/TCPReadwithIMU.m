function TCPRead()
    
    %-------------------------------------------------------------------------
    % Set up program variables and initialisers
    %-------------------------------------------------------------------------
    
    clear
    
    % TCP Variables
    ip_address = '127.0.0.1';
    remote_port_cam = 15000;
    remote_port_imu = 14500;
    
    % Camera Variables
    height = 120;
    width = 160;
    MaxDist = 5;
    Live = 1;
    %MaxRecordSize = 100;
    %recordedData = zeros(3, 19200, 200);   % Turn on if recording
    %idx = 1;
    
    %IMU Variables
    Bias.Ax = -0.0175; Bias.Ay = -0.0868; Bias.Az = -1.0134;
    Bias.Gx = -1.8285; Bias.Gy = 2.5944; Bias.Gz = 1.3713;
    %Bias = EstimateBias(t);
    IMU_data = struct('Attitude_G',[0, 0, 0],'Attitude_A',[0, 0],'Accel',[0, 0, 0],'Gyros',[0, 0, 0],'Dt',0,'TimeStamp',0);
    
    X = [0;0;pi/2];    % Vehicle Pose
    
    counter = 1;
    close all;
    
    %-------------------------------------------------------------------------
    % Set up plot handles
    %-------------------------------------------------------------------------
    
    figure(1); hold on;
    guiH.DepthVisualisation = imagesc();
    colorbar;
    caxis([0 1]);
    axis([0 160 0 120]);

    figure(2);
    hold on;zoom on ; grid on; axis equal;
    guiH.Vertices = plot3(0,0,0,'.');
    guiH.roi = scatter3(0, 0, 0, 'g');
    guiH.normalLine = plot3(0, 0, 0, 'r');
    xlabel('x'); ylabel('y'); zlabel('z');
    title('untransformed point cloud data');
    axis([0 1 -0.7 0.7 -0.7 0.7]);
    view(225, 15);

    fig3 = figure(3); hold on; axis equal
    guiH.DepthScan = scatter(0,0,25,[0 0 0]);   %Depth map at horizon scatterplot handle
    guiH.Marker = scatter(0,0,'r*');  %Object of interest marker overlay Handle
    set(fig3, 'position', [30 30 800 800])
    axis([0 4 -2 2]);
    title('scan of horizon');
    xlabel('x'); ylabel('y');
    
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
    
    figure(5);
    guiH.og = surf(zeros(60));
    og = OccupancyGrid(3, 3, 0.05);
    xlabel('x'); ylabel('y'); zlabel('z');
    
    figure(6); hold on; axis equal
    GlobalMap.Landmarks = plot(0,0,'r*');   %Depth map at horizon scatterplot handle
    title('Global Map');
    xlabel('x'); ylabel('y');
    
%     figure(7); clf(); hold on;
%     Handles.Wx = plot(0,0,'r');
%     Handles.Wy = plot(0,0,'g');
%     Handles.Wz = plot(0,0,'b');
%     Handles.Title_Gyros = title('');
%     ylim([-800,800]);
%     xlabel('Time (Seconds)')
%     ylabel('Rate of Yaw (Degrees/Sec)')
%     zoom on; grid on;
%     
%     figure(8); clf(); hold on;
%     Handles.Ax = plot(0,0,'r');
%     Handles.Ay = plot(0,0,'g');
%     Handles.Az = plot(0,0,'b');
%     Handles.Title_Accel = title('');
%     ylim([-15,15]);
%     xlabel('Time (Seconds)')
%     ylabel('Linear Acceleration (M/Sec^2)')
%     zoom on; grid on;
    
    figure(9); clf(); hold on;
    Handles.Roll_G = plot(0,0,'r');
    Handles.Pitch_G = plot(0,0,'g');
    Handles.Yaw_G = plot(0,0,'b');
    Handles.Title_Attitude_G = title('');
    ylim([-100, 100]);
    xlabel('Time (Seconds)')
    ylabel('Attitude (Degrees)')
    zoom on; grid on;
    
    figure(10); clf(); hold on;
    Handles.Roll_A = plot(0,0,'r');
    Handles.Pitch_A = plot(0,0,'g');
    Handles.Title_Attitude_A = title('');
    ylim([-100, 100]);
    xlabel('Time (Seconds)')
    ylabel('Attitude (Degrees)')
    zoom on; grid on;
    
    %-------------------------------------------------------------------------
    % Connect to Servers
    %-------------------------------------------------------------------------
    
    % Connect to Camera server
    if (Live == 1)
        t = tcpip(ip_address,remote_port_cam);  %Initiate cam TCP connection
        t.ByteOrder = 'littleEndian';   %Set Endian to convert
        set(t,'InputBufferSize', width*height*3*2);
        fopen(t);
    end
    
    % Connect to IMU server
    p = tcpip(ip_address, remote_port_imu);
    p.ByteOrder = 'littleEndian';   %Set Endian to convert
    fopen(p);   pause(1); 
    
    % Wait for incoming bytes from IMU TCP
    while p.BytesAvailable == 0
        pause(1)
        disp('waiting for initial imu bytes...');
    end
        
    % Wait for incoming bytes from Camera TCP
    if (Live == 1)   
        while t.BytesAvailable == 0 
            pause(1)
            disp('waiting for initial cam bytes...');
        end
    end
    
    disp('Connected to Servers');
    
    %-------------------------------------------------------------------------
    % Begin main program loop
    %-------------------------------------------------------------------------
    
    if (Live == 0)
        load('recordedCameraData_20170503.mat')
        iteration = 1;
    end
    
    while ((get(p, 'BytesAvailable') > 0 && ((get(t, 'BytesAvailable') > 0)||(~Live)))||(1))
        %----------------------------------------------------------------------
        %Receive and save incoming IMU data
        %----------------------------------------------------------------------
        
        if (get(p, 'BytesAvailable') > 0) 
            disp("Reading IMU...");
 
            IMU = Parse_IMU_Serial(p);   % Read over TCP and save IMU values
            
            % Save data into a history buffer applying bias removal
            IMU_data.Accel(counter, 1) = IMU.Ax - Bias.Ax;    
            IMU_data.Accel(counter, 2) = IMU.Ay - Bias.Ay;
            IMU_data.Accel(counter, 3) = IMU.Az - Bias.Az;
            IMU_data.Gyros(counter, 1) = (IMU.Gx - Bias.Gx)*pi/180;
            IMU_data.Gyros(counter, 2) = (IMU.Gy - Bias.Gy)*pi/180;
            IMU_data.Gyros(counter, 3) = (IMU.Gz - Bias.Gz)*pi/180;
            IMU_data.Dt(counter) = IMU.Dt;

            if (counter > 1)
               IMU_data.TimeStamp(counter) = IMU_data.TimeStamp(counter - 1) + IMU.Dt;
            else 
               IMU_data.TimeStamp(counter) = 0;
            end
 
            %Calculate new attitude (Using Gyroscope + Using Accelometer)
            IMU_data.Attitude_G(counter + 1,:) = ProcessAttitude_Gyros(IMU_data.Gyros(counter,:), IMU_data.Dt(counter), IMU_data.Attitude_G(counter,:));
            IMU_data.Attitude_A(counter + 1,:) = ProcessAttitude_Accel(IMU_data.Accel(counter,:), IMU_data.Attitude_A(counter,:));
            
            Plot_IMU(Handles, IMU_data, counter);
            
            counter = counter + 1;
        end

        %----------------------------------------------------------------------
        %Receive and save incoming Camera data
        %----------------------------------------------------------------------
        
        if (Live == 0)             
            y = (recordedData(2,:,iteration))';
            z = (recordedData(3,:,iteration))';
            x = (recordedData(1,:,iteration))';

            iteration = iteration + 1;
        elseif (get(t, 'BytesAvailable') > 0)
            disp("Reading Camera...");
            buff = fread(t, height*width*3, 'int16');

            y = buff(1:19200); % Y X Z
            z = buff(19201:38400);
            x = buff(38401:57600);
        
            %uncomment to record some camera data:
        %     recordedData(1, :, idx) = x;
        %     recordedData(2, :, idx) = y;
        %     recordedData(3, :, idx) = z;
        %     idx = idx + 1;
        %     if idx == length(recordedData(1, 1, :))
        %         break;
        %     end

            %----------------------------------------------------------------------
            %Process and Plot Camera data
            %----------------------------------------------------------------------

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

            pc = [x'; y'; z'];
            roi = camROI(pc);

            set(guiH.Vertices, 'xdata', x, 'ydata', y, 'zdata', z);
            set(guiH.DepthVisualisation, 'CData', DepthMap);

            if numel(roi) < 20*3
                disp('Not enough pts to estimate floor');
                set(guiH.normalLine, 'xdata', 0, 'ydata', 0, 'zdata', 0);
                set(guiH.roi, 'xdata', 0, 'ydata', 0, 'zdata', 0);
            else
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
            end
   
        %----------------------------------------------------------------------
        % Process Occupancy Grid and Process Vehicle Localisation
        %----------------------------------------------------------------------
        
%         theta = X(3) - pi/2;
%         rotationMatrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
% 
%         % Apply rotation matrix to align data with heading
%         V = rotationMatrix*[sl(1, :); sl(2, :)];
%         xVals = V(1,:);
%         yVals = V(2,:) + X(2);
%         
%         xVals = xVals + X(1);
%         yVals = yVals + X(2);
%         
%         %set(guiH.scanLine, 'xdata', sl(1, :), 'ydata', sl(2, :), 'zdata', sl(3, :));
%         set(guiH.scanLine, 'xdata', xVals, 'ydata', yVals, 'zdata', sl(3, :));
%         
%         og.addObservations(xVals, yVals);
        og.addObservations(sl(1, :), sl(2, :));
        og.visualise(guiH.og);
        og.decrement();
%         
%         OOIs = og.FindOOIs();
%         
%         %Global Frame Plot of Vehicle and Landmarks
%         set(GlobalMap.Landmarks, 'xdata', OOIs.centers.x(:), 'ydata', OOIs.centers.y(:));
%         
%         X = Localise(OOIs, X);  %update pose based on "known" locations of OOIS 
        end
        pause(0.01);
    end

    %-------------------------------------------------------------------------
    % Close main program loop
    %-------------------------------------------------------------------------
    
    disp('Closing Connections to Servers');
    
    % Close and clear connection to IMU
    pause(1)
    fclose(p); 
    delete(p);
    clear p
    
    % Close and clear connection to Camera
    if (Live == 0)
        pause(1)
        fclose(t);
        delete(t);
        clear t
    end
end

%-------------------------------------------------------------------------
% Functions
%-------------------------------------------------------------------------

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


function imuRaw = Parse_IMU_Serial(t) 
    imuRaw = struct('Ax', 0, 'Ay', 0, 'Az',0, 'Gx',0,'Gy',0,'Gz',0, 'Dt', 0);
    
    aRange = 19.6;
    aResolution = 10;
    gScale = 14.375;
    
    convert_a = aRange/2^(aResolution-1);
    convert_g = (1/gScale);
    
    recvBuff = fread(t,8,'int16');
    
    if(recvBuff(1) == 18500)
        %Acceleration in m/s^2
        imuRaw.Ax = recvBuff(2)*convert_a;
        imuRaw.Ay = recvBuff(3)*convert_a;
        imuRaw.Az = recvBuff(4)*convert_a;
        
        %Angular Rates in Degrees/s
        imuRaw.Gx = recvBuff(5)*convert_g;
        imuRaw.Gy = recvBuff(6)*convert_g;
        imuRaw.Gz = recvBuff(7)*convert_g;
        imuRaw.Dt = recvBuff(8)/1000;
    end
    return;
end

function NewAttitude = ProcessAttitude_Gyros(gyros, dt, CurrentAttitude)
    wx = gyros(1);  r = CurrentAttitude(1); 
    wy = gyros(2);  p = CurrentAttitude(2); 
    wz = gyros(3);  y = CurrentAttitude(3);
    
    if (~isnan(wx) && ~isnan(wy) && ~isnan(wz)) % If data is valid
        roll = r + dt*(wx + (wy*sin(r) + wz*cos(r))*tan(p)); 
        pitch = p + dt*(wy*cos(r) - wz*sin(r));
        yaw = y + dt*((wy*sin(r) + wz*cos(r))/cos(p));
        NewAttitude = [roll, pitch, yaw]; %new global Roll, Pitch, Yaw (at time t+dt)
    else
        NewAttitude = CurrentAttitude;
    end
end

function NewAttitude = ProcessAttitude_Accel(accel, CurrentAttitude)
    ax = accel(1);  
    ay = accel(2);
    az = accel(3);
 
    if (~isnan(ax) && ~isnan(ay) && ~isnan(az)) % Is data valid
        pitch = atan(-ax/az);
        roll = atan(ay/sqrt(ax^2 + az^2));

        NewAttitude = [roll, pitch]; %new global Roll, Pitch (at time t+dt)
    else
         NewAttitude = CurrentAttitude;
    end
end

function Plot_IMU(mh, imu, i)  
%     set(mh.Wx, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Gyros(1:i,1)*(180/pi));
%     set(mh.Wy, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Gyros(1:i,2)*(180/pi));
%     set(mh.Wz, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Gyros(1:i,3)*(180/pi));
%     s = sprintf('Gyroscope Plot: x(r), y(g), z(b)');
%     set(mh.Title_Gyros, 'string', s);
%     
%     set(mh.Ax, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Accel(1:i,1));
%     set(mh.Ay, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Accel(1:i,2));
%     set(mh.Az, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Accel(1:i,3));
%     s = sprintf('Accelerometer Plot: x(r), y(g), z(b)');
%     set(mh.Title_Accel, 'string', s);
    
    set(mh.Roll_G, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Attitude_G(1:i,1)*4*(180/pi));
    set(mh.Pitch_G, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Attitude_G(1:i,2)*4*(180/pi));
    set(mh.Yaw_G, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Attitude_G(1:i,3)*4*(180/pi));
    s = sprintf('Attitude_G Plot: Roll(r), Pitch(g), Yaw(b)');
    set(mh.Title_Attitude_G, 'string', s);
    
    set(mh.Roll_A, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Attitude_A(1:i,1)*(180/pi));
    set(mh.Pitch_A, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Attitude_A(1:i,2)*(180/pi));
    s = sprintf('Attitude_A Plot: Roll(r), Pitch(g)');
    set(mh.Title_Attitude_A, 'string', s);
return;
end