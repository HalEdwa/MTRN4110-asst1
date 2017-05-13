function TCPRead()
    
    %-------------------------------------------------------------------------
    % Set up program variables and initialisers
    %-------------------------------------------------------------------------
    
    clear   % Perform an initial clear
    
    % TCP Variables
    ip_address = '127.0.0.1';
    remote_port_cam = 15000;
    remote_port_imu = 14500;
    
    % Camera Variables
    width = 160;
    height = 120;
    Live = 0;
    CameraRead = 0;
    
    % IMU Variables
    Bias.Ax = -0.0175; Bias.Ay = -0.0868; Bias.Az = -1.0134;
    Bias.Gx = -1.8285; Bias.Gy = 2.5944; Bias.Gz = 1.3713;
    IMU_data = struct('Attitude_G',[0, 0, 0],'Attitude_A',[0, 0],'Accel',[0, 0, 0],'Gyros',[0, 0, 0],'Dt',0,'TimeStamp',0);
    
    % Landmark map Variables
    global DA_Landmarks;
    LandmarkMap.x = [-1,0,1,-0.5,0.5,-1,0,1,-0.5,0.5,-1,0,1];
    LandmarkMap.y = [0.5,0.5,0.5,1,1,1.5,1.5,1.5,2,2,2.5,2.5,2.5];
    LandmarkMap.N = numel(LandmarkMap.x);
    
    % Localisation Variables
    X = [0;0;pi/2];
    k = 1;
    
    %-------------------------------------------------------------------------
    % Set up plot handles
    %-------------------------------------------------------------------------
    
    % Image Feed
    figure(1); clf();
    guiH.Image = imagesc();
    colorbar;
    caxis([0 1500]);
    axis([0 160 0 120]);
    
    % 3D vertices plot
    figure(2); clf(); hold on; rotate3d on; 
    guiH.PointCloud = plot3(0,0,0,'.','MarkerSize',2.5);
    guiH.FloorNormal = quiver3(0, 0, 0, 0, 0, 0, 500 );
    xlabel('x'); ylabel('y'); zlabel('z');
    axis([0 1500 -600 600 -400 600]);
    %axis ij
    title('3D Vertices Plot');
    grid on;
    
    % Local Frame plot of Depth Scans
    figure(3); clf(); hold on;
    LocalMap.ScanData = plot(0,0,'.');   %Depth map at horizon scatterplot handle
    LocalMap.OOIs = plot(0,0,'o','MarkerFaceColor', 'r');  %Object of interest marker overlay Handle
    axis([0 1 -1 1]);
    title('DepthMap Scan at height = 0');
    xlabel('x'); ylabel('y');
    
    % Global Frame plot
    figure(4); clf(); hold on; axis equal
    GlobalMap.DepthScan = plot(0,0,'b.');
    GlobalMap.Localisation = plot(0,0,'r');
    GlobalMap.CurrentPos = plot(0,0,'black.','markersize',7);
    GlobalMap.Heading = quiver(0,0,0,0,'magenta');
    GlobalMap.DA_Labels = [];
    GlobalMap.DA_Labels_Map = [];
    GlobalMap.Landmarks = plot(0,0,'g.','markersize',15);   %Depth map at horizon scatterplot handle
    GlobalMap.LandmarksMap = plot(0,0,'cyan.','markersize',15);   %Depth map at horizon scatterplot handle
    axis([-2 2 -0.2 3.5]);
    title('Global Map');
    xlabel('x'); ylabel('y');
    
    figure(5);
    guiH.og = surf(zeros(60));
    og = OccupancyGrid(3, 3, 0.05);
    title('Occupancy Grid');
    xlabel('x'); ylabel('y'); zlabel('z');
    
%     figure(6); clf(); hold on;
%     Handles.Wx = plot(0,0,'r');
%     Handles.Wy = plot(0,0,'g');
%     Handles.Wz = plot(0,0,'b');
%     Handles.Title_Gyros = title('');
%     ylim([-800,800]);
%     xlabel('Time (Seconds)')
%     ylabel('Rate of Yaw (Degrees/Sec)')
%     zoom on; grid on;
%     
%     figure(7); clf(); hold on;
%     Handles.Ax = plot(0,0,'r');
%     Handles.Ay = plot(0,0,'g');
%     Handles.Az = plot(0,0,'b');
%     Handles.Title_Accel = title('');
%     ylim([-15,15]);
%     xlabel('Time (Seconds)')
%     ylabel('Linear Acceleration (M/Sec^2)')
%     zoom on; grid on;
    
%     figure(8); clf(); hold on;
%     Handles.Roll_G = plot(0,0,'r');
%     Handles.Pitch_G = plot(0,0,'g');
%     Handles.Yaw_G = plot(0,0,'b');
%     Handles.Title_Attitude_G = title('');
%     ylim([-100, 100]);
%     xlabel('Time (Seconds)')
%     ylabel('Attitude (Degrees)')
%     zoom on; grid on;
%     
%     figure(9); clf(); hold on;
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
        GlobalMap.DA_Labels_Map(i) = text(double(LandmarkMap.x(i) - 0.03),double(LandmarkMap.y(i) + 0.15),int2str(i),'FontSize',8,'Color','cyan');
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
    
    % Connect to IMU server
%     p = tcpip(ip_address, remote_port_imu);
%     p.ByteOrder = 'littleEndian';   %Set Endian to convert
%     fopen(p);   pause(1); 
%     
    % Wait for incoming bytes from IMU TCP
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
        load('recordedCameraData_20170503.mat')
        iteration = 1;
    end
    
    %-------------------------------------------------------------------------
    % Begin main program loop
    %-------------------------------------------------------------------------
    
    while (true)
        %----------------------------------------------------------------------
        %Receive and save incoming IMU data
        %----------------------------------------------------------------------
        
        if (false)%get(p, 'BytesAvailable') > 0) 
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
            
            Plot_IMU(Handles, IMU_data, counter);
            
            counter = counter + 1;
        end
        
        %----------------------------------------------------------------------
        %Receive and save incoming Camera data
        %----------------------------------------------------------------------
        
        if (Live == 0)             
            % Load saved camera data frame
            y = (recordedData(2,:,iteration))';
            z = (recordedData(3,:,iteration))';
            x = (recordedData(1,:,iteration))';
            
            iteration = iteration + 1;
        elseif (get(t, 'BytesAvailable') > 0)
            disp("Reading Camera...");
            buff = fread(t, height*width*3, 'int16');
            
            % Read new camera data frame
            y = buff(1:19200); % Horizontal displacement
            z = buff(19201:38400);  % Vertical displacement
            x = buff(38401:57600);  % Depth
            
            CameraRead = 1; % Flag a successful read
        end
        
        %----------------------------------------------------------------------
        % Process incoming camera frame and triangulate pose
        %----------------------------------------------------------------------
        
        if (Live == 0 || ((Live == 1) && CameraRead == 1))
            x(x > 1500) = 0;    % Filter out depths that are too far
            x(x < 0) = 0;   % Filter out negative depths
            
            % Process incoming camera frame
            [xx,yy,zz] = ProcessCameraFrame(x,y,z); % Filter and transform camera data
            [roi_x, roi_y, roi_z] = ProcessROI(xx,yy,zz);  % Selects region of interest and transforms dataset
            [Pitch, Roll, Normal] = CalculateAttitude(roi_x, roi_y, roi_z); % Estimate Pitch and Roll
            [xT,yT,zT] = TransformCamera(Pitch,Roll,x,y,z); %Transform data such that floor is flat
            DepthScan = ScanPlane(xT/1000,yT/1000);    % Extract a depth scan along the horizon and convert mm to m
            
            set(guiH.Image,'CData',xx); % Plot DepthMap as an image
            set(guiH.PointCloud,'xdata',xT,'ydata',yT,'zdata',zT);  % Plot transformed point cloud
            % Plot normal, projected from transformed point cloud at approximate center of data
            set(guiH.FloorNormal,'xdata',mean(roi_x(:)),'ydata',mean(roi_y(:)),'zdata',mean(roi_z(:)),'udata',Normal(1),'vdata',Normal(2),'wdata', Normal(3));
            
            % Perform object classification from camera data 
            OOIs = ExtractOOIs(DepthScan,LocalMap);  % Capture pole like objects as OOIs
            [GlobalOOIs, GlobalDepthScan] = TransformToGlobal(OOIs, DepthScan, X);   % Rotate and translate data by X
            AssociateLandmarks(GlobalOOIs, LandmarkMap);    % Update DA_Landmarks
            
            set(GlobalMap.DepthScan,'xdata',GlobalDepthScan.x,'ydata',GlobalDepthScan.y);
            set(GlobalMap.Landmarks,'xdata',GlobalOOIs.x,'ydata',GlobalOOIs.y);
            
            if DA_Landmarks.N > 0
                delete(GlobalMap.DA_Labels); % Delete labels every plot
            end
            
            CameraRead = 0; % Reset read flag
            
            %----------------------------------------------------------------------
            % Process Vehicle Localisation
            %----------------------------------------------------------------------

            X = Localise(X,LandmarkMap);    % Update pose based on data associated landmarks
            Pose(:,k) = X;
            
            % Display Results of localisation
            %printf('Pose: X = %3f, Y = %3f, Theta = %3f\n', Pose(1,k), Pose(2,k), (Pose(3,k)));	% Print current pose 
            set(GlobalMap.Localisation,'xdata',Pose(1,1:k),'ydata',Pose(2,1:k));  %Global Frame Plot of Vehicle Pose
            set(GlobalMap.Heading,'xdata',X(1),'ydata',X(2),'Udata',0.3*cos(X(3)),'Vdata',0.3*sin(X(3)));
            
            k = k + 1;
            
            %----------------------------------------------------------------------
            % Process and Plot Occupancy Grid
            %----------------------------------------------------------------------

            og.addObservations(DepthScan(1,:), DepthScan(2,:));
            og.visualise(guiH.og);
            og.decrement();
        end
        
        if(Live == 0)
           pause(0.3);
        end
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
% Camera Functions
%-------------------------------------------------------------------------

function [xx,yy,zz] = ProcessCameraFrame(x,y,z);
    xx = flipud(reshape(x,[160,120])');
    yy = flipud(reshape(y,[160,120])');
    zz = flipud(reshape(z,[160,120])');
end

function [roi_x, roi_y, roi_z] = ProcessROI(xx,yy,zz)
    % 40 rows, 81 cols Middle section of first third closest rows
    roi_y= yy(1:30, 40:120)';
    roi_z= zz(1:30, 40:120)'; 
    roi_x= xx(1:30, 40:120)';
end

function [Pitch, Roll, Normal] = CalculateAttitude(roi_x, roi_y, roi_z)
%     temp = [roi_x,  roi_y ones(size(roi_x))];   
%     
%     p = temp\roi_z;    % Calculate floor plane
%     n = [-p(1), -p(2), 1];  % Calculate floor normal
%     
%     Pitch = atand(n(1)/n(3));
%     Roll = -atand(n(2)/n(3));
%     Normal = n;

    [EstimatePlane, ~] = fit([roi_x', roi_y'], roi_z', 'poly11');

    %plane: a*x + b*y + c*z = d
    a = EstimatePlane.p10;
    b = EstimatePlane.p01;
    c = 1;
    d = EstimatePlate.p00;

    n = [a, b, -c];
    
    Normal = n / norm(n);
    Pitch = asind(n(1));
    Roll = asind(-n(2)/cos(Pitch));
end

function [xT,yT,zT] = TransformCamera(Pitch,Roll,x,y,z)
    Rx = [1, 0, 0;
          0, cosd(Roll), -sind(Roll);
          0, sind(Roll), cosd(Roll)];  % Generate roll rotation matrix
        
    Ry = [cosd(Pitch), 0, sind(Pitch);
          0, 1, 0;
          -sind(Pitch), 0, cosd(Pitch)];  % Generate pitch rotation matrix
    
    rotatedCoords = Rx * [x'; y'; z'];  % Appply roll transformation
    rotatedCoords = Ry * [rotatedCoords(1,:); rotatedCoords(2,:); rotatedCoords(3,:)];  % Apply pitch transformation
    
    xT = rotatedCoords(1,:);
    yT = rotatedCoords(2,:);
    zT = rotatedCoords(3,:);
end

function DepthScan = ScanPlane(xT,yT)
    xT = flipud(reshape(xT,[160,120])');
    yT = flipud(reshape(yT,[160,120])');
    
    DepthScan(1,:) = xT(65,:);
    DepthScan(2,:) = yT(65,:);
end

%-------------------------------------------------------------------------
% Classification and Data Association
%-------------------------------------------------------------------------

function r = ExtractOOIs(DepthScan,h)
    x = DepthScan(1,:);
    y = DepthScan(2,:);

    r.N = 0;
    r.centers.x = [];
    r.centers.y = [];
    r.Sizes   = [];
    
    if numel(x) + numel(y) == 0
        return;
    end
    
    %since the data contains several scan lines, it is no longer an ordered
    %dataset. Fix this by converting to polar form, ordering based on
    %theta, then changing back to cartesian
    unorderedRanges = sqrt(x.^2 + y.^2);
    unorderedTheta = atan(y./x);
    [theta, order] = sort(unorderedTheta);
    ranges = unorderedRanges(order);
    x = ranges .* cos(theta);
    y = ranges .* sin(theta);
    
    % Classifier parameters
    poleDia = 0.1;     % Expected diameter of OOI
    poleDiaTol = 0.1;  % Percentage of deviation allowed on OOI dia
    filterSize = 0.15;   % Distance between points to be grouped as a cluster
    minClusterSize = 3;
    
    clusterEndPts = [1];%have to initialise
    hues = zeros(1, length(x));
    newHue = 0.3;
    purple = 213/255;
    
    %find all clusters of points:
    for i = 2:length(x) - 1
        
        if (x(i) - x(i-1))^2 + (y(i) - y(i - 1))^2 > filterSize^2
            newHue = mod(newHue + 0.73, 1);
            
            %filter yellow because its hard to see and purple because it's 
            %reserved:
            while (newHue(1) > 90/255 && newHue(1) < 30/255 ) || (newHue(1) > 200/255 && newHue(1) < 230/255)
                newHue = mod(newHue + 0.73, 1);
            end

            if (x(i) - x(i+1))^2 + (y(i) - y(i+1))^2 > filterSize^2
                clusterEndPts = [clusterEndPts, i - 1, i + 1];
                hues(:, i) = 0;%pts not in a cluster are black
                i = i + 1;
            else 
                clusterEndPts = [clusterEndPts, i - 1, i];
            end
        end
        hues(:, i) = newHue;
    end

    clusterEndPts = [clusterEndPts, length(x)];
    
    %the ends of these clusters tend to have points that have fallen off
    %the back. Trim the size of the clusters:
    for i = 1:2:length(clusterEndPts)
        meanX = mean( x(clusterEndPts(i):clusterEndPts(i+1)) );
        
        %trim beginning:
        while abs(x(clusterEndPts(i)) - meanX) > poleDia*0.6%tolerance is arbitrary
            hues(clusterEndPts(i)) = 0;
            clusterEndPts(i) = clusterEndPts(i) + 1;
            meanX = mean( x(clusterEndPts(i):clusterEndPts(i+1)) );
        end
        
        %trim end:
        while abs(x(clusterEndPts(i+1)) - meanX) > poleDia*0.6%tolerance is arbitrary
            hues(clusterEndPts(i+1)) = 0;
            clusterEndPts(i+1) = clusterEndPts(i+1) - 1;
            meanX = mean( x(clusterEndPts(i):clusterEndPts(i+1)) );
        end
    end
    
    for i = 1:2:length(clusterEndPts) - 1
        %the 3D camera tends to have trailing points on the outside of
        %clusters. So compare the y distances only to ignore these:
        objectSize = abs(y(clusterEndPts(i)) - y(clusterEndPts(i+1)));
        if  (objectSize < (poleDia*(1 + poleDiaTol )))&&( objectSize > (poleDia*(1 - poleDiaTol)))&&(length(hues(clusterEndPts(i):clusterEndPts(i+1))) >= minClusterSize)
            hues(clusterEndPts(i):clusterEndPts(i+1)) = purple; % Purple points identify clusters with an OOI
        end
        
        r.N = r.N + 1;
        r.centers.x = [r.centers.x mean(x(clusterEndPts(i):clusterEndPts(i+1)))];
        r.centers.y = [r.centers.y mean(y(clusterEndPts(i):clusterEndPts(i+1)))];
        r.Sizes = [r.Sizes pdist( [x(clusterEndPts(i)), y(clusterEndPts(i));
               x(clusterEndPts(i+1)), y(clusterEndPts(i+1))])];
    end

    x = x(hues ~= 0);
    y = y(hues ~= 0);
    hues = hues(hues ~= 0);
    vals = ones(1, length(hues));
    vals(hues ~= purple) = 0.6;
    
    if numel(hues) == 0
        return
    end
    
    colours = hsv2rgb([hues; ones(1, length(hues)); vals]');
    set(h.ScanData, 'xdata', x, 'ydata', y);%, 'cdata', colours);
    set(h.OOIs, 'xdata', r.centers.x, 'ydata', r.centers.y);
end

function [GlobalOOIs,GlobalDepthScan] = TransformToGlobal(OOIs, DepthScan, X)
    % Transform OOIs into global frame
    theta = X(3);
    rotationMatrix = [cos(theta), -sin(theta); sin(theta), cos(theta)]; % Form rotation matrixes
    
    % Transform scan data and OOIs detected into the global frame
    V_DepthScan = rotationMatrix*[DepthScan(1,:); DepthScan(2,:)];    % DepthScan Rotation
    V_OOIs = rotationMatrix*[OOIs.centers.x(:)'; OOIs.centers.y(:)'];   % OOIs Rotation

    % DepthScan Translation
    GlobalDepthScan.x = V_DepthScan(1,:) + X(1);
    GlobalDepthScan.y = V_DepthScan(2,:) + X(2);

    % OOIs Translation
    GlobalOOIs.x = V_OOIs(1,:) + X(1);    % Translate x positions of OOI
    GlobalOOIs.y = V_OOIs(2,:) + X(2);    % Translate y positions of OOI
    GlobalOOIs.N = OOIs.N;  % Number of landmarks
end

function AssociateLandmarks(GlobalOOIs, LandmarkMap)
    global DA_Landmarks;
    ID_Tolerance = 0.1;
    
    g = 1;
    
    if (GlobalOOIs.N > 0)
        for m = 1:GlobalOOIs.N %iterate through live OOIs
            for n = 1:LandmarkMap.N %Check each live OOIs against each OOI in first scan
                % Check is distance between points is close enough
                if (((GlobalOOIs.x(m) - LandmarkMap.x(n))^2 + ((GlobalOOIs.y(m) - LandmarkMap.y(n))^2)) < ID_Tolerance^2)
                    gh.DA_Labels(g) = text(double(GlobalOOIs.x(m) - 0.1),double(GlobalOOIs.y(m)),int2str(n),'FontSize',8,'Color','g');
                    DA_Landmarks.x(g) = GlobalOOIs.x(m);   % Save x value
                    DA_Landmarks.y(g) = GlobalOOIs.y(m);   % Save y value
                    DA_Landmarks.ID(g) = n;    % Save current ID
                    g = g + 1;
                end
            end
        end
        
        DA_Landmarks.N = g - 1; % Save number of data associated landmarks
    else
        DA_Landmarks.N = 0; % No data associated landmarks
    end
    
    pause(0.01);   % Small delay to make sure labels are visible
end

%-------------------------------------------------------------------------
% Localisation and Process Model Functions
%-------------------------------------------------------------------------

function X = Localise(X_Last,LandmarkMap)
        global DA_Landmarks;
        
        options = optimset('MaxFunEvals',1000);
        
        if (DA_Landmarks.N > 1)   %Triangulate and localise
            X = fminsearch(@(X) Triangulate(X,LandmarkMap),[X_Last(1),X_Last(2),X_Last(3)], options); % Triangulate and return x, y, phi
        else
            X = X_Last;
        end
end

function F = Triangulate(X, LandmarkMap)
    global DA_Landmarks;
    
    F = zeros(DA_Landmarks.N * 2, 1);
    u = 1;
    
    for i = 1:DA_Landmarks.N
        ID = DA_Landmarks.ID(i);
        
        edx = LandmarkMap.x(ID) - X(1);
        edy = LandmarkMap.y(ID) - X(2);
        mdx = DA_Landmarks.x(i) - X(1);
        mdy = DA_Landmarks.y(i) - X(2);
        
        ExpectedRange = sqrt(edx^2 + edy^2);
        MeasuredRange = sqrt(mdx^2 + mdy^2);
        ExpectedBearing = atan2(edx, edy) + pi/2 - X(3);
        MeasuredBearing = atan2(mdx, mdy) + pi/2 - X(3);
        
        F(u) = MeasuredRange - ExpectedRange; % Range
        F(u + 1) = MeasuredBearing - ExpectedBearing; % Bearing
        u = u + 2;
    end
    F = sum(F.^2);
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