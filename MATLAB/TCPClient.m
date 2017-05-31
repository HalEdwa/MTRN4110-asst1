function TCPRead()
    
    %-------------------------------------------------------------------------
    % Set up program variables and initialisers
    %-------------------------------------------------------------------------
    
    clear   % Perform an initial clear
    
    % Camera Variables
    width = 160;
    height = 120;
    Live = 1;
    CameraRead = 1;
    pathmode = 0;
    
    % Landmark map Variables
    global DA_Landmarks;
    
%    LandmarkMap.x = [-1,0,1,-0.5,0.5,-1,0,1,-0.5,0.5,-1,0,1];
%    LandmarkMap.y = [0.5,0.5,0.5,1,1,1.5,1.5,1.5,2,2,2.5,2.5,2.5];
%    LandmarkMap.N = numel(LandmarkMap.x);

     LandmarkMap.x = [-1,    -0.5,   0,      0.5,    1,      1,      -1];
     LandmarkMap.y = [1,   1,    1,    1,    1,    0.5,      0.5];
     LandmarkMap.N = numel(LandmarkMap.x);
    
    % Localisation Variables
    X = [0;0;pi/2];
    k = 1;
    
    it = 1;
    labels = text(0,0,' ');
    
    global state
    state = 1;
    
    global destination
    destination.x = 0;
    destination.y = 0;
    
    nextDest.x = 0;
    nextDest.y = 0;
    
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
    GlobalMap.Path = plot(0,0,'g');
    set(f, 'WindowButtonDownFcn', @clicker);
    axis([-2 2 -0.2 3.5]);
    title('Global Map');
    xlabel('y'); ylabel('x');

    % Plot known Landmark Map on global Frame
    set(GlobalMap.LandmarksMap, 'xdata', LandmarkMap.x(:), 'ydata', LandmarkMap.y(:));
    
    for i = 1:LandmarkMap.N
        GlobalMap.DA_Labels_Map(i) = text(double(LandmarkMap.x(i) - 0.03),double(LandmarkMap.y(i) + 0.15),int2str(i),'FontSize',8,'Color','black');
    end
    
    %-------------------------------------------------------------------------
    % Connect to Servers
    %-------------------------------------------------------------------------
    
    % TCP Variables
    ip_address = '127.0.0.1';
    remote_port_cam = 15000;
    remote_port_hex = 14000;
    remote_port_arm = 13000;
    
    % Connect to Camera server
    if (Live == 1)
        t = tcpip(ip_address,remote_port_cam);  %Initiate cam TCP connection
        t.ByteOrder = 'littleEndian';   %Set Endian to convert
        set(t,'InputBufferSize', width*height*3*2);
        fopen(t);
    end
    
    t_hex = tcpip('127.0.0.1', remote_port_hex);
    t_hex.ByteOrder = 'littleEndian';%Set Endian to convert
    fopen(t_hex);
    
    t_arm = tcpip('127.0.0.1', remote_port_arm);
    t_arm.ByteOrder = 'littleEndian';%Set Endian to convert
    fopen(t_arm);
        
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
    while (true)
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
            
            for i=1:OOIs.N
               if (sqrt((OOIs.centers.x(i))^2 + (OOIs.centers.y(i))^2)) < 0.25
                 ArmControl(t_arm);
               end
            end
           
            [GlobalOOIs, GlobalDepthScan] = TransformToGlobal(OOIs, DepthScan, X);   % Rotate and translate data by X
            
            delete(labels);
            labels = AssociateLandmarks(GlobalOOIs, OOIs, LandmarkMap, GlobalMap.DA_Labels);    % Update DA_Landmarks
            
            set(GlobalMap.DepthScan,'xdata',GlobalDepthScan.x,'ydata',GlobalDepthScan.y);
            set(GlobalMap.Landmarks,'xdata',GlobalOOIs.x,'ydata',GlobalOOIs.y);
            
            CameraRead = 0; % Reset read flag
            
            %----------------------------------------------------------------------
            % Process Vehicle Localisation
            %----------------------------------------------------------------------

            X = Localise(X,LandmarkMap);    % Part 3 original localisation
            Pose(:,k) = X;
            
            if pathmode == 1
                setDest = nextDest;
            else
                setDest = destination;
            end
            
            [MV, MH, Rot] = PathFollower(X,setDest);
            HexControl(0,Rot,MV,MH,t_hex);
            %HexControl(0,0,0,0,t_hex);

            % Display Results of localisation
            fprintf('Pose: X = %3f, Y = %3f, Theta = %3f\n', Pose(1,k), Pose(2,k), rad2deg((Pose(3,k))));	% Print current pose 
            fprintf('Rot = %3f; MV_Command = %d; MH_Command = %d;\n\n', Rot, MV, MH);
            
            set(GlobalMap.Localisation,'xdata',Pose(1,k),'ydata',Pose(2,k));  %Global Frame Plot of Vehicle Pose
            set(GlobalMap.Heading,'xdata',X(1),'ydata',X(2),'Udata',0.3*cos(X(3)),'Vdata',0.3*sin(X(3)));
            set(GlobalMap.CurrentPos,'xdata',Pose(1,k),'ydata', Pose(2,k));
            set(GlobalMap.Destination,'xdata',destination.x,'ydata',destination.y);
            
            k = k + 1;
            
            %----------------------------------------------------------------------
            % Process and Plot Occupancy Grid
            %----------------------------------------------------------------------
            
            %Populate occupancy grid
            og.addObservations(GlobalOOIs.x, GlobalOOIs.y, 0.12);
            og.visualise(guiH.og);
            og.decrement();
        end
        
        if (Live == 1)
            pause(0.01);   % Short pause to allow rotation inputs for plotting
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
    ID_Tolerance = 0.12;
    
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
    
    destination.x = cursorPoint(1,1)
    destination.y = cursorPoint(1,2)
end

function [MV, MH, Rot] = PathFollower(X,setDest)   
    MV = 0;
    MH = 0;
    Rot = 0;
    
    destinationTol = 0.07;
    speed = 70;
    
    if (abs(setDest.x - X(1)) >= destinationTol)
        MH = speed*((setDest.x - X(1))/(abs(setDest.x - X(1))));
    end
    
    if (abs(setDest.y - X(2)) >= destinationTol)
        MV = speed*((setDest.y - X(2))/(abs(setDest.y - X(2))));
    end
    
    if (abs(pi/2 - X(3)) >= deg2rad(5))
        Rot = -(50)*((pi/2 - X(3))/(abs(pi/2 - X(3))));
    end
    
    MV = floor(MV);
    MH = floor(MH);
    Rot = floor(Rot);
end

function [OG_Index_Dest, OG_Index_Pos] = GlobaltoOG(X)
    global destination
    
    OG_Index_Dest = floor(((destination.x + 1.5) + 60 * destination.y) / 0.05);
    OG_Index_Pos = floor(((X(1) + 1.5) + 60 * X(2)) / 0.05);
end

function GlobalPath = OGPathtoGlobal(Path)
    GlobalPath.y = Path(:,1).* 0.05;
    GlobalPath.x = Path(:,2).* 0.05 - 1.5;
end

function ArmControl(t_arm)
    global state
    
    if (state)
        %Sweep Left
        Motor1Speed = int16(0);
        Motor2Speed = int16(0);
        Motor1Pos = int16(195);
        Motor2Pos = int16(195);
    else
        %Sweep Right
        Motor1Speed = int16(0);
        Motor2Speed = int16(0);
        Motor1Pos = int16(0);
        Motor2Pos = int16(0);
    end
    
    state = ~state;
    
    Motor1Speed = (typecast(Motor1Speed, 'uint8'));
    Motor2Speed = (typecast(Motor2Speed, 'uint8'));
    Motor1Pos = (typecast(Motor1Pos, 'uint8'));
    Motor2Pos = (typecast(Motor2Pos, 'uint8'));
        
    outgoing(1) = 'c';
    outgoing(2) = Motor1Speed(1);
    outgoing(3) = Motor1Speed(2);
    outgoing(4) = Motor2Speed(1);
    outgoing(5) = Motor2Speed(2);
    outgoing(6) = Motor1Pos(1);
    outgoing(7) = Motor1Pos(2);
    outgoing(8) = Motor2Pos(1);
    outgoing(9) = Motor2Pos(2);
    
    fwrite(t_arm, outgoing);
    fprintf("Demon Slash of 1000 cuts of pain!");
end