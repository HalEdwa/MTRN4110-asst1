function TCPRead()
    
    %-------------------------------------------------------------------------
    % Set up program variables and initialisers
    %-------------------------------------------------------------------------
    
    clear   % Perform an initial clear
    
    % TCP Variables
    ip_address = '127.0.0.1';
    remote_port_cam = 15000;
    remote_port_imu = 14500;
    remote_port_hex = 14000;
    
    t_hex = tcpip('127.0.0.1', 14000);
    t_hex.ByteOrder = 'littleEndian';%Set Endian to convert
    
    fopen(t_hex);
    
    % Camera Variables
    width = 160;
    height = 120;
    Live = 1;
    CameraRead = 0;
    
    % IMU Variables
    Bias.Ax = -0.0175; Bias.Ay = -0.0868; Bias.Az = -1.0134;
    Bias.Gx = -1.8285; Bias.Gy = 2.5944; Bias.Gz = 1.3713;
    IMU_data = struct('Attitude_G',[0, 0, pi/2],'Attitude_A',[0, 0],'Accel',[0, 0, 0],'Gyros',[0, 0, 0],'Dt',0,'TimeStamp',0);
    
    % Landmark map Variables
    global DA_Landmarks;
%     LandmarkMap.x = [-1,0,1,-0.5,0.5,-1,0,1,-0.5,0.5,-1,0,1];
%     LandmarkMap.y = [0.5,0.5,0.5,1,1,1.5,1.5,1.5,2,2,2.5,2.5,2.5];
%     LandmarkMap.N = numel(LandmarkMap.x);
    LandmarkMap.x = [-1,    -0.5,   0,      0.5,    1,      1,      -1];
    LandmarkMap.y = [1,   1,    1,    1,    1,    0.5,      0.5];
    LandmarkMap.N = numel(LandmarkMap.x);
    % Localisation Variables
    X = [0;0;pi/2];
    k = 1;
    
    it = 1;
    labels = text(0,0,' ');
    
    global destination
    destination.x = 0;
    destination.y = 0;
    %-------------------------------------------------------------------------
    % Set up plot handles
    %-------------------------------------------------------------------------
    figure(6); clf(); hold on;
    guiH.og = surf(zeros(60));
    og = OccupancyGrid(3, 3, 0.05);
    title('Occupancy Grid');
    xlabel('x'); ylabel('y'); zlabel('z');
    % Image Feed
    figure(1); clf();
    guiH.Image = imagesc();
    colorbar;
    caxis([0 1500]);
    axis([0 160 0 120]);
    
    % 3D vertices plot raw
    figure(2); clf(); hold on; rotate3d on; 
    guiH.PointCloud = plot3(0,0,0,'.','MarkerSize',2.5);
    guiH.FloorNormal = quiver3(0, 0, 0, 0, 0, 0, 500);
    guiH.PointCloud_Title = title('');
    xlabel('x'); ylabel('y'); zlabel('z');
    axis([0 1500 -600 600 -400 600]);
    grid on;
    
    % 3D vertices plot transformed
    figure(3); clf(); hold on; rotate3d on; 
    guiH.PointCloudT = plot3(0,0,0,'.','MarkerSize',2.5);
    xlabel('x'); ylabel('y'); zlabel('z');
    axis([0 1500 -600 600 -400 600]);
    title('3D Vertices Plot Transformed');
    grid on;
    
    % Local Frame plot of Depth Scans
    figure(4); clf(); hold on;
    LocalMap.ScanData = plot(0,0,'.');   %Depth map at horizon scatterplot handle
    LocalMap.OOIs = plot(0,0,'o','MarkerFaceColor', 'r');  %Object of interest marker overlay Handle
    axis([-1 1 0 1.5]);
    title('DepthMap Scan at height = 0');
    xlabel('y'); ylabel('x');
    
    % Global Frame plot
    f = figure(5); clf(); hold on; axis equal
    GlobalMap.DepthScan = plot(0,0,'cyan.');
    GlobalMap.Localisation = plot(0,0,'r');
    GlobalMap.CurrentPos = plot(0,0,'blue.','markersize',12);
    GlobalMap.Heading = quiver(0,0,0,0,'magenta');
    GlobalMap.DA_Labels = text(0,0,' ');
    GlobalMap.DA_Labels_Map = text(0,0, ' ');
    GlobalMap.Landmarks = plot(0,0,'r.','markersize',15);   %Depth map at horizon scatterplot handle
    GlobalMap.LandmarksMap = plot(0,0,'black.','markersize',15);   %Depth map at horizon scatterplot handle
    GlobalMap.Destination = plot(0,0,'magenta.','markersize',15);
    set(f, 'WindowButtonDownFcn', @clicker);
    axis([-2 2 -0.2 3.5]);
    title('Global Map');
    xlabel('y'); ylabel('x');
    
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
%     
%     figure(9); clf(); hold on;
%     Handles.Roll_G = plot(0,0,'r');
%     Handles.Pitch_G = plot(0,0,'g');
%     Handles.Yaw_G = plot(0,0,'b');
%     Handles.Title_Attitude_G = title('');
%     ylim([-100, 100]);
%     xlabel('Time (Seconds)')
%     ylabel('Attitude (Degrees)')
%     zoom on; grid on;
%     
%     figure(10); clf(); hold on;
%     Handles.Roll_A = plot(0,0,'r');
%     Handles.Pitch_A = plot(0,0,'g');
%     Handles.Title_Attitude_A = title('');
%     ylim([-100, 100]);
%     xlabel('Time (Seconds)')
%     ylabel('Attitude (Degrees)')
%     zoom on; grid on;
    
    % Plot known Landmark Map on global Frame
     set(GlobalMap.LandmarksMap, 'xdata', LandmarkMap.x(:), 'ydata', LandmarkMap.y(:));
    for i = 1:LandmarkMap.N
        GlobalMap.DA_Labels_Map(i) = text(double(LandmarkMap.x(i) - 0.03),double(LandmarkMap.y(i) + 0.15),int2str(i),'FontSize',8,'Color','black');
    end

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
    
% %     Connect to IMU server
%     p = tcpip(ip_address, remote_port_imu);
%     p.ByteOrder = 'littleEndian';   %Set Endian to convert
%     fopen(p);   pause(1); 
%     
%     % Wait for incoming bytes from IMU TCP
%     while p.BytesAvailable == 0
%         pause(1)
%         disp('waiting for initial imu bytes...');
%     end
        
    % Wait for incoming bytes from Camera TCP
    if (Live == 1)   
        while t.BytesAvailable == 0 
            pause(1)
            disp('waiting for initial cam bytes...');
        end
    end
    
    disp('Connected to Servers');
    
    if (Live == 0)
        load('recordedCameraData_240517.mat')
    end
    
    %-------------------------------------------------------------------------
    % Begin main program loop
    %-------------------------------------------------------------------------
    counter = 1;
    
    while (true)
        %----------------------------------------------------------------------
        %Receive and save incoming IMU data
        %----------------------------------------------------------------------
        
        if false
            disp('Reading IMU...');
 
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
            
            % Update vehicle yaw based on accelometer pitch and roll + current vehicle yaw 
            X(3) = DeadReckoningYaw(IMU_data.Gyros(counter,:), IMU_data.Dt(counter), IMU_data.Attitude_A(counter,1), IMU_data.Attitude_A(counter,2), X(3)); % Update vehicle pose based on gyros
            
            Plot_IMU(Handles, IMU_data, counter);
            counter = counter + 1;
        end
        
        %----------------------------------------------------------------------
        %Receive and save incoming Camera data
        %----------------------------------------------------------------------
        
        if (Live == 0)             
            % Load saved camera data frame
            y = CamRecord.y(it,:)';
            x = CamRecord.x(it,:)';
            z = CamRecord.z(it,:)';
            
            it = it + 1;
        elseif (get(t, 'BytesAvailable') > 0)
            %disp("Reading Camera...");
            buff = fread(t, height*width*3, 'int16');
            
            % Read new camera data frame
            y = buff(1:19200); % Horizontal displacement
            z = buff(19201:38400);  % Vertical displacement
            x = buff(38401:57600);  % Depth
            
            CamRecord.x(it,:) = x(:);
            CamRecord.y(it,:) = y(:);
            CamRecord.z(it,:) = z(:);
            
            it = it + 1;
            
            CameraRead = 1; % Flag a successful read
        end
        
        %----------------------------------------------------------------------
        % Process incoming camera frame and triangulate pose
        %---------------------------------------------------------------------
        
        if (Live == 0 || ((Live == 1) && CameraRead == 1))
            xFilter = x;  % Save all depth values
            xFilter(xFilter > 2000) = -1;    % Flag depths that are too far
            xFilter(xFilter < 0) = -1;   % Flag negative depths

            % Filter out invalid data
            xx = x(xFilter ~= -1);
            yy = -y(xFilter ~= -1);
            zz = z(xFilter ~= -1);
            
            % Transform camera data into image
            DepthMap = GetDepthMap(x);
            
            % Process Camera data
            [roi_x, roi_y, roi_z] = ProcessROI(xx,yy,zz);  % Selects region of interest and transforms dataset
            [Pitch, Roll, Normal] = CalculateAttitude(roi_x, roi_y, roi_z); % Estimate Pitch and Roll
            [xT,yT,zT] = TransformCamera(Pitch,Roll,xx,yy,zz); %Transform data such that floor is flat
            DepthScan = ScanPlane(-xT/1000,yT/1000,zT/1000);    % Extract a depth scan along the horizon and convert mm to m
            
            set(guiH.Image,'CData',DepthMap); % Plot DepthMap as an image
            set(guiH.PointCloud,'xdata',x,'ydata',y,'zdata',z);  % Plot raw point cloud
            % Plot normal vector estimated from raw point cloud
            set(guiH.FloorNormal,'xdata',mean(roi_x(:)),'ydata',mean(roi_y(:)),'zdata',mean(roi_z(:)),'udata',Normal(1),'vdata',Normal(2),'wdata', Normal(3));
            s = sprintf('3D Point Cloud Raw: Pitch = %3f Roll = %3f', Pitch, Roll); % Plot title with pitch and roll
            set(guiH.PointCloud_Title, 'string', s);
            set(guiH.PointCloudT,'xdata',xT,'ydata',yT,'zdata',zT);
            
            % Perform object classification from camera data
            OOIs = FindOOIs(DepthScan,LocalMap);
            
            [GlobalOOIs, GlobalDepthScan] = TransformToGlobal(OOIs, DepthScan, X);   % Rotate and translate data by X
            
            delete(labels);
            labels = AssociateLandmarks(GlobalOOIs, OOIs, LandmarkMap, GlobalMap.DA_Labels);    % Update DA_Landmarks
            
            set(GlobalMap.DepthScan,'xdata',GlobalDepthScan.x,'ydata',GlobalDepthScan.y);
            set(GlobalMap.Landmarks,'xdata',GlobalOOIs.x,'ydata',GlobalOOIs.y);
            
            CameraRead = 0; % Reset read flag
            
            %----------------------------------------------------------------------
            % Process Vehicle Localisation
            %----------------------------------------------------------------------
            %Ranges = zeros(361,1);  % Replace with scan data
            %LaserScan = GetLaserScan(dataL);
            
            %VehiclePosition = FindVehiclePosition(LaserScan, ScanMap, GlobalMap.LaserScan);   % Estimate vehicle position from LIDAR data
            %[X(1), X(2)] = UpdatePosition(VehiclePosition);  % Update position (X(1), X(2) based on lidar scan
            %X(3) = LocaliseYaw(X,LandmarkMap);    % Update yaw (X(3)) based on data associated landmarks
            
            X = Localise(X,LandmarkMap);    % Part 3 original localisation
            Pose(:,k) = X;
            
            [MV, MH] = PathFollower(X);
            HexControl(0,0,MV,MH,t_hex);
            %HexControl(0,0,0,0,t_hex);
            Rot = 0;
            % Display Results of localisation
            fprintf('Pose: X = %3f, Y = %3f, Theta = %3f\n', Pose(1,k), Pose(2,k), rad2deg((Pose(3,k))));	% Print current pose 
            fprintf('Rot_Command = %d; MV_Command = %d; MH_Command = %d;', Rot, MV, MH);
            
            set(GlobalMap.Localisation,'xdata',Pose(1,k),'ydata',Pose(2,k));  %Global Frame Plot of Vehicle Pose
            set(GlobalMap.Heading,'xdata',X(1),'ydata',X(2),'Udata',0.3*cos(X(3)),'Vdata',0.3*sin(X(3)));
            set(GlobalMap.CurrentPos,'xdata',Pose(1,k),'ydata', Pose(2,k));
            set(GlobalMap.Destination,'xdata',destination.x,'ydata',destination.y);
            
            k = k + 1;
            
            %----------------------------------------------------------------------
            % Process and Plot Occupancy Grid
            %----------------------------------------------------------------------
            
            og.addObservations(-DepthScan(1,:), DepthScan(2,:));
            og.visualise(guiH.og);
            og.decrement();
        end
        
        if (Live == 1)
            pause(0.001);   % Short pause to allow rotation inputs for plotting
        else
            pause(0.2); % Longer pause to set recorded data reading in framerate
        end
    end

    %-------------------------------------------------------------------------
    % Close main program loop
    %-------------------------------------------------------------------------
    
    disp('Closing Connections to Servers');
    
    % Close and clear connection to IMU
    pause(1);
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
% Camera Functions
%-------------------------------------------------------------------------

function DepthMap = GetDepthMap(x)
    DepthMap = flipud(reshape(x,[160,120])');
end

function [roi_x, roi_y, roi_z] = ProcessROI(xx,yy,zz)
    % Parameters for ROI conditions (millimeters)
    xSize = 400;
    ySize = 400;
    xOffset = 100;
    floorNoise = 100;
    
    % Set conditions for points to be within ROI
    minX = min(xx);
    xCrit = (xx > (minX + xOffset)) & (xx < (minX + xOffset + xSize));
    yCrit = (yy > -ySize/2) & (yy < ySize/2);
    zCrit = zz < (min(zz) + floorNoise);
    roi_condition = xCrit & yCrit & zCrit;
    
    % Apply conditions and save ROI points
    roi_x = xx(roi_condition);
    roi_y = yy(roi_condition);
    roi_z = zz(roi_condition);
end

function [Pitch, Roll, Normal] = CalculateAttitude(roi_x, roi_y, roi_z)
    temp = [roi_x,  roi_y ones(size(roi_x))];
    
    p = temp\roi_z;    % Calculate floor plane
    n = [-p(1), -p(2),1];  % Calculate floor normal
    
    Pitch = -atand(n(1) / n(3));
    Roll = atand(n(2) / n(3));
    Normal = n / norm(n);
end

function [xT,yT,zT] = TransformCamera(Pitch,Roll,x,y,z)
    Rx = [1, 0, 0;
          0, cosd(Roll), -sind(Roll);
          0, sind(Roll), cosd(Roll)];  % Generate roll rotation matrix
        
    Ry = [cosd(Pitch), 0, sind(Pitch);
          0, 1, 0;
          -sind(Pitch), 0, cosd(Pitch)];  % Generate pitch rotation matrix
    
    rotatedCoords = Rx * [x'; y'; z'];  % Apply roll transformation
    rotatedCoords = Ry * [rotatedCoords(1,:); rotatedCoords(2,:); rotatedCoords(3,:)];  % Apply pitch transformation
    
    % Save transformed point cloud
    xT = rotatedCoords(1,:);
    yT = rotatedCoords(2,:);
    zT = rotatedCoords(3,:);
    
    % Attempt to offset floor to z = 0 position
    zOffset = min(zT(zT > -200)) + 95;
    zT = zT - zOffset;
end

function DepthScan = ScanPlane(xT,yT,zT)    
    roi = find((zT > 0.15)&(zT < 0.25));
    
    DepthScan(1,:) = xT(roi);
    DepthScan(2,:) = yT(roi);
end

%-------------------------------------------------------------------------
% Classification and Data Association
%-------------------------------------------------------------------------

function [LaserScan, intensities] = GetLaserScan(scan)
    anglesDeg = [0:360]'*0.5;      % angles in degrees
    anglesRad = anglesDeg/180*pi;   % angles in radians
    
    % scan data is provided as a array of class uint16, which encode range
    % and intensity (that is the way the sensor provides the data, in that
    % mode of operation)
    
    MaskLow13Bits = uint16(2^13 - 1);   % Set 13 bit mask, to extract range data
    MaskHigh3Bits = bitshift(uint16(2^3 - 1), 13); % Set top 3 bit mask for intensity
    
    rangesCM = bitand(scan, MaskLow13Bits); % range vector in CMs
    ranges = 0.01*double(rangesCM); % range vector in meters as float
    intensities = bitand(scan, MaskHigh3Bits); % intensity vector
    
    % Convert POLAR to Cartesian co-ords
    [LaserScan(1), LaserScan(2)] = pol2cart(anglesRad, ranges);
end

function VehiclePosition = FindVehiclePosition(X,LaserScan,intensities,lh,gh)
    X = LaserScan(1,:);
    Y = LaserScan(2,:);
    
    OOIs.N = 0;             % scalar for number of OOIs detected
    OOIs.Colors = [];       % Vector 1 x N for Color of each OOI
    OOIs.Centers = [];      % Matrix of size 2 x N, with X,Y coords for center of OOI
    OOIs.Diameters = [];    % Vector 1 x N for diam of each OOI
    
    clusters.N = 1;
    clusters.start = zeros(1,361);  %index of cluster starts
    clusters.end = zeros(1,361);    %index of cluster ends
    
    clusters.start(1) = 1;          %index of first cluster starts at 1
    
    % Find clusters
    for i = 2:361
       if sqrt((X(i)-X(i-1))^2 + (Y(i)-Y(i-1))^2) > 0.10
            clusters.end(clusters.N) = i-1;
            clusters.start(clusters.N + 1) = i;
            clusters.N = clusters.N + 1;
       end 
    end
    
    clusters.end(clusters.N) = 361; %index of last cluster ends at 361
    
    %Detect diameters - hacky approximation start - end
    for i = 1:clusters.N
       iStart = clusters.start(i);
       iEnd = clusters.end(i);
       dist = sqrt((X(iStart)-X(iEnd))^2 + (Y(iStart)-Y(iEnd))^2);
       
       % If distances between start and end points are within range,
       % register as OOI
       if (dist >= 0.05) && (dist <= 0.20)
           
           %Increment OOI count
           OOIs.N = OOIs.N + 1;
           
           %Save Diameter
           OOIs.Diameters(OOIs.N) = dist;
           
           %Scan to see if cluster contains pixel with high intensity
           clusterIntensities = intensities(iStart:iEnd);
           
           if find(clusterIntensities > 0)
              OOIs.Colors(OOIs.N) = 1;
           else
              OOIs.Colors(OOIs.N) = 0;
           end
           
           %Add OOI center data
           OOIs.Centers(:, OOIs.N) = [(X(iStart)+X(iEnd))/2, (Y(iStart)+Y(iEnd))/2];
       end
    end
    
    if OOIs.N > 0
       Dist = (X(1) - OOIs.Centers(1,OOIs.Colors == 1)).^2 + (X(2) - OOIs.Centers(2,OOIs.Colors == 1)).^2;  
       
       if size(Dist) > 0
           % If at least one OOI has at least intense point pick closest to current position
           VehiclePosition(1) = OOIs.Centers(1,min(Dist(:)));
           VehiclePosition(2) = OOIs.Centers(2,min(Dist(:)));
       else
           % If no OOIs have at least one intense point set to last known position
           VehiclePosition(1) = X(1);
           VehiclePosition(2) = X(2);
       end
    else 
       % If no OOIs found set to last known position
       VehiclePosition(1) = X(1);
       VehiclePosition(2) = X(2);
    end
    
    set(lh, 'xdata', X(:), 'ydata', Y(:));
    set(gh, 'xdata', VehiclePosition(1), 'ydata', VehiclePosition(2));
end

function OOIs = FindOOIs(DepthScan,h)
    x = DepthScan(1,:);
    y = DepthScan(2,:);
    
    %since the data contains several scan lines, it is no longer an ordered
    %dataset. Fix this by converting to polar form, ordering based on
    %theta, then changing back to cartesian
    unorderedRanges = sqrt(x.^2 + y.^2);
    unorderedTheta = atan(y./x);
    [theta, order] = sort(unorderedTheta);
    ranges = unorderedRanges(order);
    x = ranges .* cos(theta);
    y = ranges .* sin(theta);
    
    OOIs.N = 0;
    OOIs.centers.x = [];
    OOIs.centers.y = [];
    
    MinPoleDia = 0.04;
    MaxPoleDia = 0.15;
    MinPoints = 20;
    ClusterDist = 0.05;
    
    iStart = 1;
    iEnd = 1;
    oldiEnd = 0;
    StartArray = [];
    EndArray = [];
    CenterXArray =  [];
    CenterYArray =  [];
    for i = 2:(length(x)-1)
      if (sqrt((x(i)-x(i-1))^2 + (y(i)-y(i-1))^2)) < ClusterDist
         iEnd = i;
      else
          %create of start, end and center fo every cluster
            StartArray = [StartArray iStart];
            if oldiEnd == iEnd 
                iEnd = iEnd +1;
            end
            EndArray = [EndArray iEnd];
            oldiEnd = iEnd;
            CenterXArray = [CenterXArray mean(x(iStart:iEnd))];
            CenterYArray = [CenterYArray mean(y(iStart:iEnd))];
            iStart = i;
      end  
    end
    
    %Average positions of close OOIs
    ConcatClusters.N = 0;
    ConcatClusters.Start = [];
    ConcatClusters.End = [];
    offset = 0;
    toMean = 0;
    minGap = 0.2; %to make a new cluster
  
    for i=1:1:length(StartArray)-1
        if sqrt((CenterXArray(i)-CenterXArray(i+1))^2 + (CenterYArray(i)-CenterYArray(i+1))^2) > minGap
            ConcatClusters.N = ConcatClusters.N+1;
            ConcatClusters.Start = [ConcatClusters.Start min(StartArray(i-toMean:i))] ;
            ConcatClusters.End = [ConcatClusters.End max(StartArray(i-toMean:i))];
            toMean = 0;
        else
            offset=offset+1;
            toMean=toMean +1;
        end
        
        if i == length(StartArray)-1
            ConcatClusters.N = ConcatClusters.N+1;
            ConcatClusters.Start = [ ConcatClusters.Start min(StartArray(i+1-toMean:i+1))];
            ConcatClusters.End = [ ConcatClusters.End max(StartArray(i+1-toMean:i+1))]; 
            toMean = 0;
        end
    end
    
    for i = 1:1:ConcatClusters.N
         iStart = ConcatClusters.Start(i);
         iEnd = ConcatClusters.End(i);
         dist = sqrt((x(iStart)-x(iEnd))^2 + (y(iStart)-y(iEnd))^2);
          
          if ((dist >= MinPoleDia)&&(dist <= MaxPoleDia)&&((iEnd-iStart) >= MinPoints))
             OOIs.N = OOIs.N + 1;
             OOIs.centers.x(OOIs.N) = mean(x(iStart:iEnd));
             OOIs.centers.y(OOIs.N) = mean(y(iStart:iEnd));
          end
    end 
    
    set(h.ScanData, 'xdata', y, 'ydata', x);
    set(h.OOIs, 'xdata', OOIs.centers.y, 'ydata', OOIs.centers.x);
end

function [GlobalOOIs,GlobalDepthScan] = TransformToGlobal(OOIs, DepthScan, X)
    % Transform OOIs into global frame
    theta = X(3);
    rotationMatrix = [cos(theta), -sin(theta); sin(theta), cos(theta)]; % Form rotation matrixes
    
    % Transform scan data and OOIs detected into the global frame
    V_DepthScan = rotationMatrix*[-DepthScan(1,:); DepthScan(2,:)];    % DepthScan Rotation
    V_OOIs = rotationMatrix*[OOIs.centers.x(:)'; -OOIs.centers.y(:)'];   % OOIs Rotation

    % DepthScan Translation
    GlobalDepthScan.x = V_DepthScan(1,:) + X(1);
    GlobalDepthScan.y = V_DepthScan(2,:) + X(2);

    % OOIs Translation
    GlobalOOIs.x = V_OOIs(1,:) + X(1);    % Translate x positions of OOI
    GlobalOOIs.y = V_OOIs(2,:) +X(2);    % Translate y positions of OOI
    GlobalOOIs.N = OOIs.N;  % Number of landmarks
end

function GlobalOOIs = GlobalTransform(OOIs, X)
    % Transform OOIs into global frame
    theta = X(3);
    rotationMatrix = [cos(theta), -sin(theta); sin(theta), cos(theta)]; % Form rotation matrixes
    
    % Transform scan data and OOIs detected into the global frame
    V_OOIs = rotationMatrix*[OOIs.x(:)'; -OOIs.y(:)'];   % OOIs Rotation

    % OOIs Translation
    GlobalOOIs.x = V_OOIs(1,:) + X(1);    % Translate x positions of OOI
    GlobalOOIs.y = V_OOIs(2,:) + X(2);    % Translate y positions of OOI
    GlobalOOIs.N = OOIs.N;  % Number of landmarks
end

function gh = AssociateLandmarks(GlobalOOIs, LocalOOIs, LandmarkMap,gh)
    global DA_Landmarks;
    ID_Tolerance = 0.2;
    
    DA_Landmarks.x = [];
    DA_Landmarks.y = [];
    DA_Landmarks.ID = [];
    DA_Landmarks.N = 0;
    
    g = 1;

    if (GlobalOOIs.N > 0)
        for m = 1:GlobalOOIs.N %iterate through live OOIs
            for n = 1:LandmarkMap.N %Check each live OOIs against each OOI in first scan
                % Check is distance between points is close enough
                if (((GlobalOOIs.x(m) - LandmarkMap.x(n))^2 + ((GlobalOOIs.y(m) - LandmarkMap.y(n))^2)) < ID_Tolerance^2)
                    if ~any(n == DA_Landmarks.ID(:))    % Hack to get rid of duplicate associate values
                        DA_Landmarks.x(g) = LocalOOIs.centers.x(m);   % Save x value
                        DA_Landmarks.y(g) = LocalOOIs.centers.y(m);   % Save y value
                        LabelPos.x(g) = GlobalOOIs.x(m);
                        LabelPos.y(g) = GlobalOOIs.y(m);
                        DA_Landmarks.ID(g) = n;    % Save current ID
                        g = g + 1;
                    end
                end
            end
        end
        
        DA_Landmarks.N = g - 1; % Save number of data associated landmarks
    else
        DA_Landmarks.N = 0; % No data associated landmarks
    end
    
    if DA_Landmarks.N > 0
        gh = text(LabelPos.x(:) + 0.07, LabelPos.y(:),int2str(DA_Landmarks.ID(:)),'FontSize',10,'Color','r');
    end
end

%-------------------------------------------------------------------------
% Localisation and Process Model Functions
%-------------------------------------------------------------------------

function Yaw = LocaliseYaw(X_Last,LandmarkMap)
        global DA_Landmarks;
        
        if (DA_Landmarks.N > 1)   %Triangulate and localise
            X0 = fminsearch(@(X) Triangulate(X,LandmarkMap),[X_Last(1),X_Last(2),X_Last(3)]);
        else
            X0 = X_Last;
        end
        
        Yaw = X0(3);   % Only update yaw using triangulation not position
end

function X = Localise(X_Last,LandmarkMap)
        global DA_Landmarks;
        
        phi = X_Last(3);
        
        if (DA_Landmarks.N == 1)   %Triangulate and localise
            OptimFunc = @(X_Last) Triangulate(LandmarkMap,phi, X_Last);
            X(1:2) = fminsearch(OptimFunc,[X_Last(1),X_Last(2)]);
            X(3) = X_Last(3);
        elseif (DA_Landmarks.N > 1)
            OptimFunc = @(X_Last) MultiTriangulate(LandmarkMap,X_Last);
            X = fminsearch(OptimFunc,[X_Last(1),X_Last(2),X_Last(3)]);
        else
            X = X_Last;
        end
end

function Cost = Triangulate(RealLandmarks,phi, X_Last)
    global DA_Landmarks;
    X_Last(3) = phi;
    GlobalLandmarks = GlobalTransform(DA_Landmarks, X_Last);
    
    id = DA_Landmarks.ID(1);
    error_x = GlobalLandmarks.x(1) - RealLandmarks.x(id);
    error_y = GlobalLandmarks.y(1) - RealLandmarks.y(id);
    
    Cost = error_x^2 + error_y^2;
end

function Cost = MultiTriangulate(RealLandmarks,X_Last)
    global DA_Landmarks;
    
    error_x = zeros(DA_Landmarks.N,1);
    error_y = zeros(DA_Landmarks.N,1);
    
    GlobalLandmarks = GlobalTransform(DA_Landmarks, X_Last);
    
    for i = 1:DA_Landmarks.N
        id = DA_Landmarks.ID(i);
        
        error_x(i) = GlobalLandmarks.x(i) - RealLandmarks.x(id);
        error_y(i) = GlobalLandmarks.y(i) - RealLandmarks.y(id);
    end

    Cost = sqrt(sum(error_x(:).^2 + error_y(:).^2));
end

%-------------------------------------------------------------------------
% IMU Functions
%-------------------------------------------------------------------------

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
    set(mh.Wx, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Gyros(1:i,1)*(180/pi));
    set(mh.Wy, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Gyros(1:i,2)*(180/pi));
    set(mh.Wz, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Gyros(1:i,3)*(180/pi));
    s = sprintf('Gyroscope Plot: x(r), y(g), z(b)');
    set(mh.Title_Gyros, 'string', s);
    
    set(mh.Ax, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Accel(1:i,1));
    set(mh.Ay, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Accel(1:i,2));
    set(mh.Az, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Accel(1:i,3));
    s = sprintf('Accelerometer Plot: x(r), y(g), z(b)');
    set(mh.Title_Accel, 'string', s);
    
%     set(mh.Roll_G, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Attitude_G(1:i,1)*4*(180/pi));
%     set(mh.Pitch_G, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Attitude_G(1:i,2)*4*(180/pi));
%     set(mh.Yaw_G, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Attitude_G(1:i,3)*4*(180/pi));
%     s = sprintf('Attitude_G Plot: Roll(r), Pitch(g), Yaw(b)');
%     set(mh.Title_Attitude_G, 'string', s);
%     
%     set(mh.Roll_A, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Attitude_A(1:i,1)*(180/pi));
%     set(mh.Pitch_A, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Attitude_A(1:i,2)*(180/pi));
%     s = sprintf('Attitude_A Plot: Roll(r), Pitch(g)');
%     set(mh.Title_Attitude_A, 'string', s);
return;
end

function HexControl(RightVert, RightHor, LeftVert, LeftHor, t_hex)
    %Specify Movement of Hexapod between -128 to 127
    RV = 128-RightVert;
    RH = 128+RightHor;
    LV = 128-LeftVert;
    LH = 128-LeftHor;
    
    outgoing(1) = RV;
    outgoing(2) = RH;
    outgoing(3) = LV;
    outgoing(4) = LH;
    outgoing(5) = 0;
    outgoing(6) = 0;
                
    fwrite(t_hex, outgoing);    
end

function clicker(h,~)
    cursorPoint = get(gca, 'currentpoint');
    global destination
    
    destination.x = cursorPoint(1,1);
    destination.y = cursorPoint(1,2);
end

function [MV, MH] = PathFollower(X)
    global destination
    
    %Kp_Rotate = 30;%142;
    %Kp_tV = 50;%84;
    %Kp_tH = 50;%103;
    
    %DesiredYaw = atan2((destination.y - X(2)),(destination.x - X(1)));
    %CurrentYaw = X(3); 
    
    MV = 0;
    MH = 0;
    
    if (abs(destination.x - X(1)) >= 0.05)
        MH = 60*((destination.x - X(1))/(abs(destination.x - X(1))));
    end
    if (abs(destination.y - X(2)) >= 0.05)
        MV = 60*((destination.y - X(2))/(abs(destination.y - X(2))));
    end
    %MH = Kp_tH * (destination.x - X(1));
    %MV = Kp_tV * (destination.y - X(2));
    %Rot = Kp_Rotate * (CurrentYaw - DesiredYaw);
    
%     if MV > 127
%         MV = 127;
%     elseif MV < -128
%         MV = -128;
%     end
%     
%     if MH > 127
%         MH = 127;
%     elseif MH < -128
%         MH = -128;
%     end
%     
%     if Rot > 127
%         Rot = 127;
%     elseif Rot < -128
%         Rot = -128;
%     end
%     
%     Rot = floor(Rot);
    MV = floor(MV);
    MH = floor(MH);
end