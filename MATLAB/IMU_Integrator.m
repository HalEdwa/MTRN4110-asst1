function main()
    % Create TCP/IP object 't'. Specify server machine and port number. 
    t = tcpip('127.0.0.1', 15000); 

    % Set size of receiving buffer, if needed. 
    set(t, 'InputBufferSize', 46);
    
    % Open connection to the server. 
    fopen(t);   pause(1); 
    
    %Set up plot handles
    figure(1); clf(); hold on;
    Handles.Wx = plot(0,0,'r');
    Handles.Wy = plot(0,0,'g');
    Handles.Wz = plot(0,0,'b');
    Handles.Title_Gyros = title('');
    ylim([-500,500]);
    xlabel('Time (Seconds)')
    ylabel('Rate of Yaw (Degrees/Sec)')
    zoom on; grid on;
    
    figure(2); clf(); hold on;
    Handles.Ax = plot(0,0,'r');
    Handles.Ay = plot(0,0,'g');
    Handles.Az = plot(0,0,'b');
    Handles.Title_Accel = title('');
    ylim([-100,100]);
    xlabel('Time (Seconds)')
    ylabel('Acceleration of Yaw (Degrees/Sec^2)')
    zoom on; grid on;
    
    figure(3); clf(); hold on;
    Handles.Roll_G = plot(0,0,'r');
    Handles.Pitch_G = plot(0,0,'g');
    %Handles.Yaw_G = plot(0,0,'b');
    Handles.Title_Attitude_G = title('');
    xlabel('Time (Seconds)')
    ylabel('Attitude (Degrees)')
    zoom on; grid on;
    
    figure(4); clf(); hold on;
    Handles.Roll_A = plot(0,0,'r');
    Handles.Pitch_A = plot(0,0,'g');
    %Handles.Yaw_A = plot(0,0,'b');
    Handles.Title_Attitude_A = title('');
    xlabel('Time (Seconds)')
    ylabel('Attitude (Degrees)')
    zoom on; grid on;
    
    %Initialise variables
    timeout = 0;
    pi = 3.14159265359;
    IMU_data = struct('Attitude_G', [0, 0], 'Attitude_A', [0, 0], 'Accel', [0, 0, 0], 'Gyros', [0, 0, 0], 'TimeStamp', 0);
    Bias_Buffer = struct('Accel', [0, 0, 0], 'Gyros', [0, 0, 0]);
    
    counter = 1;
    
    c = 1;
    % Discard first c amount of frames (Noisy)
    while c < 20
        if (get(t, 'BytesAvailable') > 0) 
            temp = Parse_IMU_Serial(t);   % Read over TCP and save IMU values
            c = c + 1;
        end
    end
    
    c = 1;
    % Calculate Bias
    while c < 50
        if (get(t, 'BytesAvailable') > 0) 
            temp = Parse_IMU_Serial(t);   % Read over TCP and save IMU values
            Bias_Buffer.Accel(c,1) = temp.Ax;
            Bias_Buffer.Accel(c,2) = temp.Ay;
            Bias_Buffer.Accel(c,3) = temp.Az;
            Bias_Buffer.Gyros(c,1) = temp.Gx;
            Bias_Buffer.Gyros(c,2) = temp.Gy;
            Bias_Buffer.Gyros(c,3) = temp.Gz;
            c = c + 1;
        end
    end
    
    Bias.Ax = mean(Bias_Buffer.Accel(:,1));
    Bias.Ay = mean(Bias_Buffer.Accel(:,2));
    Bias.Az = mean(Bias_Buffer.Accel(:,3));
    Bias.Gx = mean(Bias_Buffer.Gyros(:,1));
    Bias.Gy = mean(Bias_Buffer.Gyros(:,2));
    Bias.Gz = mean(Bias_Buffer.Gyros(:,3));
   
    % Receive lines of data from server 
    while (timeout < 1500)
       if (get(t, 'BytesAvailable') > 0) 
           IMU = Parse_IMU_Serial(t);   % Read over TCP and save IMU values
           
           % Save data into a history buffer applying bias removal
           IMU_data.Accel(counter, 1) = IMU.Ax - Bias.Ax;    
           IMU_data.Accel(counter, 2) = IMU.Ay - Bias.Ay;
           IMU_data.Accel(counter, 3) = IMU.Az - Bias.Az;
           IMU_data.Gyros(counter, 1) = (IMU.Gx - Bias.Gx)*(pi/180);
           IMU_data.Gyros(counter, 2) = (IMU.Gy - Bias.Gy)*(pi/180);
           IMU_data.Gyros(counter, 3) = (IMU.Gz - Bias.Gz)*(pi/180);
           IMU_data.TimeStamp(counter) = IMU.TimeStamp;  
           
           % Discard first integration since timestamp needs to initialise
           % a previous value
           if counter == 1
               dT = 0;
           else
               dT = IMU_data.TimeStamp(counter) - IMU_data.TimeStamp(counter - 1);
           end
           
           %Integrate and calculate new attitude (Using Gyroscope + Using Accelometer)
           IMU_data.Attitude_G(counter + 1,:) = ProcessAttitude_Gyros(IMU_data.Gyros(counter,:), dT, IMU_data.Attitude_G(counter,:));   
           IMU_data.Attitude_A(counter + 1,:) = ProcessAttitude_Accel(IMU_data.Accel(counter,:), IMU_data.Attitude_A(counter,:));
           
           %Plot Results
           if (rem(counter,3) == 0)  %Plot only every 10 frames  
                Plot_IMU(Handles, IMU_data, counter);   %Plot results
           end
           
           counter = counter + 1;
           timeout = 0;
           pause(0.01)
       end
       timeout = timeout + 1;
    end
    
    % Disconnect and clean up the server connection.
    fclose(t); 
    delete(t); 
    clear t;
    clear;
return;
end

function NewAttitude = ProcessAttitude_Gyros(gyros, dt, CurrentAttitude)
    wx = gyros(1);  r = CurrentAttitude(1); %local roll rate, current global roll
    wy = gyros(2);  p = CurrentAttitude(2); %local pitch rate, current global pitch
    wz = gyros(3);  %y = CurrentAttitude(3); %local yaw rate, current global yaw
    
    if (~isnan(wx) && ~isnan(wy) && ~isnan(wz)) % If data is valid  
        %Integrate gyroscope values to get new attitude
        roll = r + dt*(wx + (wy*sin(r) + wz*cos(r))*tan(p)); 
        %roll = r + dt*(wx + wy*sin(r)*tan(p));
        pitch = p + dt*(wy*cos(r) - wz*sin(r));
        %pitch = p + dt*wy*cos(r);
        %yaw = y + dt*((wy*sin(r) + wz*cos(r))/cos(p));

        %NewAttitude = [roll, pitch, yaw]; %new global Roll, Pitch, Yaw (at time t+dt)
        NewAttitude = [roll, pitch]; %new global Roll, Pitch, Yaw (at time t+dt)
    else
        NewAttitude = CurrentAttitude;
        fprintf("Gyro Fucked");
    end
end

% function NewAttitude = ProcessAttitude_Accel(accel)
%     g = -9.81;
%     fun = @(x) [sin(x(2))*g - accel(1); -cos(x(2))*sin(x(1))*g - accel(2); -cos(x(2))*cos(x(1))*g - accel(3)]; 
%     x0 = [0,0];
%     NewAttitude = fsolve(fun, x0); %new global Roll, Pitch, Yaw (at time t+dt)
% end

function NewAttitude = ProcessAttitude_Accel(accel, CurrentAttitude)
    g = -9.81;
    ax = accel(1);  
    ay = accel(2);
    az = accel(3);
 
    if (~isnan(ax) && ~isnan(ay) && ~isnan(az)) % Is data valid
        roll = atan(-accel(1)/accel(3));
        pitch = atan(accel(2)/(sqrt((accel(1))^2 + (accel(3))^2)));

        NewAttitude = [roll, pitch]; %new global Roll, Pitch (at time t+dt)
    else
        NewAttitude = CurrentAttitude;
        fprintf("Accel fucked");
    end
end

function Plot_IMU(mh, imu, i)
    set(mh.Wx, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Gyros(1:i,1)*(180/pi));
    set(mh.Wy, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Gyros(1:i,2)*(180/pi));
    set(mh.Wz, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Gyros(1:i,3)*(180/pi));
    s = sprintf('Gyroscope Plot: iteration#[%.3f] at time [%.3f] secs', i, imu.TimeStamp(i));
    set(mh.Title_Gyros, 'string', s);
    
    set(mh.Ax, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Accel(1:i,1));
    set(mh.Ay, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Accel(1:i,2));
    set(mh.Az, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Accel(1:i,3));
    s = sprintf('Accelerometer Plot: iteration#[%.3f] at time [%.3f] secs', i, imu.TimeStamp(i));
    set(mh.Title_Accel, 'string', s);
    
    set(mh.Roll_G, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Attitude_G(1:i,1)*(180/pi));
    set(mh.Pitch_G, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Attitude_G(1:i,2)*(180/pi));
    %set(mh.Yaw_G, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Attitude_G(1:i,3)*(180/pi));
    s = sprintf('Attitude_G Plot: iteration#[%.3f] at time [%.3f] secs', i, imu.TimeStamp(i));
    set(mh.Title_Attitude_G, 'string', s);
    
    set(mh.Roll_A, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Attitude_A(1:i,1)*(180/pi));
    set(mh.Pitch_A, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Attitude_A(1:i,2)*(180/pi));
    %set(mh.Yaw_A, 'xdata', imu.TimeStamp(1:i), 'ydata', imu.Attitude_A(1:i,3)*(180/pi));
    s = sprintf('Attitude_A Plot: iteration#[%.3f] at time [%.3f] secs', i, imu.TimeStamp(i));
    set(mh.Title_Attitude_A, 'string', s);
return;
end

function imuRaw = Parse_IMU_Serial(t) 
    imuRaw = struct('Ax', 0, 'Ay', 0, 'Az',0, 'Gx',0,'Gy',0,'Gz',0, 'TimeStamp', 0);
    
    DataReceived = fscanf(t);
    tmp = find(DataReceived(2:7) ~= '0',1);
    imuRaw.Ax = str2double(DataReceived(tmp + 1:7));    %Capture local 
    imuRaw.Ax = imuRaw.Ax * 3.9;
    
    tmp = find(DataReceived(8:13) ~= '0',1);
    imuRaw.Ay = str2double(DataReceived(tmp + 7:13));
    imuRaw.y = imuRaw.Ay * 3.9;
    
    tmp = find(DataReceived(14:19) ~= '0',1);
    imuRaw.Az = str2double(DataReceived(tmp + 13:19));
    imuRaw.Az = imuRaw.Az * 3.9;
    
    tmp = find(DataReceived(20:25) ~= '0',1);
    imuRaw.Gx = str2double(DataReceived(tmp + 19:25));
    
    tmp = find(DataReceived(26:31) ~= '0',1);
    imuRaw.Gy = str2double(DataReceived(tmp + 25:31));
    
    tmp = find(DataReceived(32:37) ~= '0',1);
    imuRaw.Gz = str2double(DataReceived(tmp + 31:37));
    
    tmp = find(DataReceived(38:46) ~= '0',1);
    imuRaw.TimeStamp = str2double(DataReceived(tmp + 37:46));
   
    return;
end