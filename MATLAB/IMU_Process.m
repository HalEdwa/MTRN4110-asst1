function IMU_Process()
    % Create TCP/IP object 't'. Specify server machine and port number.
    t = tcpip('127.0.0.1', 14500);
    t.ByteOrder = 'littleEndian';%Set Endian to convert
     
    % Open connection to the server. 
    fopen(t);   pause(1); 
    

    
    
    %Set up plot handles
    figure(1); clf(); hold on;
    Handles.Wx = plot(0,0,'r');
    Handles.Wy = plot(0,0,'g');
    Handles.Wz = plot(0,0,'b');
    Handles.Title_Gyros = title('');
    ylim([-800,800]);
    xlabel('Time (Seconds)')
    ylabel('Rate of Yaw (Degrees/Sec)')
    zoom on; grid on;
    
    figure(2); clf(); hold on;
    Handles.Ax = plot(0,0,'r');
    Handles.Ay = plot(0,0,'g');
    Handles.Az = plot(0,0,'b');
    Handles.Title_Accel = title('');
    ylim([-15,15]);
    xlabel('Time (Seconds)')
    ylabel('Linear Acceleration (M/Sec^2)')
    zoom on; grid on;
    
    figure(3); clf(); hold on;
    Handles.Roll_G = plot(0,0,'r');
    Handles.Pitch_G = plot(0,0,'g');
    Handles.Yaw_G = plot(0,0,'b');
    Handles.Title_Attitude_G = title('');
    ylim([-100, 100]);
    xlabel('Time (Seconds)')
    ylabel('Attitude (Degrees)')
    zoom on; grid on;
    
    figure(4); clf(); hold on;
    Handles.Roll_A = plot(0,0,'r');
    Handles.Pitch_A = plot(0,0,'g');
    Handles.Title_Attitude_A = title('');
    ylim([-100, 100]);
    xlabel('Time (Seconds)')
    ylabel('Attitude (Degrees)')
    zoom on; grid on;
    
    %Initialise variables
    timeout = 0;
    IMU_data = struct('Attitude_G',[0, 0, 0],'Attitude_A',[0, 0],'Accel',[0, 0, 0],'Gyros',[0, 0, 0],'Dt',0,'TimeStamp',0);
    
    c = 1;
    % Discard first noisy values
    while c < 100
        if (get(t, 'BytesAvailable') > 0) 
            Parse_IMU_Serial(t);   % Read over TCP and save IMU values
            c = c + 1;
        end
    end
    
    %Bias = EstimateBias(t);
    Bias.Ax = -0.0175; Bias.Ay = -0.0868; Bias.Az = -1.0134;
    Bias.Gx = -1.8285; Bias.Gy = 2.5944; Bias.Gz = 1.3713;
    
    counter = 1;
    % Receive lines of data from server 
    while (timeout < 1500)
       if (get(t, 'BytesAvailable') > 0) 
           IMU = Parse_IMU_Serial(t);   % Read over TCP and save IMU values
           
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
           
           %if (rem(counter,5) == 0)  %Plot only every 5 frames  
               Plot_IMU(Handles, IMU_data, counter);
           %end
           
           counter = counter + 1;
           timeout = 0;
       end
       
       timeout = timeout + 1;
       pause(0.01)
       
    end
    
    % Disconnect and clean up the server connection.
    fclose(t); 
    delete(t); 
    clear t;
return;
end

function NewAttitude = ProcessAttitude_Gyros(gyros, dt, CurrentAttitude)
    wx = gyros(1);  r = CurrentAttitude(1); 
    wy = gyros(2);  p = CurrentAttitude(2); 
    wz = gyros(3);  y = CurrentAttitude(3);
    
%     if (~isnan(wx) && ~isnan(wy) && ~isnan(wz)) % If data is valid
        roll = r + dt*(wx + (wy*sin(r) + wz*cos(r))*tan(p)); 
        pitch = p + dt*(wy*cos(r) - wz*sin(r));
        yaw = y + dt*((wy*sin(r) + wz*cos(r))/cos(p));

        NewAttitude = [roll, pitch, yaw] %new global Roll, Pitch, Yaw (at time t+dt)
%     else
%         NewAttitude = CurrentAttitude;
%     end
end

function NewAttitude = ProcessAttitude_Accel(accel, CurrentAttitude)
    ax = accel(1);  
    ay = accel(2);
    az = accel(3);
 
    %if (~isnan(ax) && ~isnan(ay) && ~isnan(az)) % Is data valid
        pitch = atan(-ax/az);
        roll = atan(ay/sqrt(ax^2 + az^2));

        NewAttitude = [roll, pitch]; %new global Roll, Pitch (at time t+dt)
%     else
%         NewAttitude = CurrentAttitude;
%     end
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

function Bias = EstimateBias(t)
    Bias_Buffer = struct('Accel', [0, 0, 0], 'Gyros', [0, 0, 0]);
    
    c = 1;
    % Calculate Bias
    while c < 200
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
    Bias.Az = -9.81 + mean(Bias_Buffer.Accel(:,3));
    Bias.Gx = mean(Bias_Buffer.Gyros(:,1));
    Bias.Gy = mean(Bias_Buffer.Gyros(:,2));
    Bias.Gz = mean(Bias_Buffer.Gyros(:,3));
return;
end