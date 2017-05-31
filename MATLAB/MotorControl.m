function IMU_Process()
    % Create TCP/IP object 't'. Specify server machine and port number.
    t = tcpip('127.0.0.1', 13000);
    t.ByteOrder = 'littleEndian';%Set Endian to convert
    
    fopen(t)
    
    
    
    Motor1Speed = int16(0);
    Motor2Speed = int16(0);
    
    speedPrompt = 'Would you like to set motor angular rate? y/n ';
    Motor1SpeedPrompt = 'Angular rate for motor 1? 0-1023 (0 is max) ';
    Motor2SpeedPrompt = 'Angular rate for motor 2? 0-1023 (0 is max) ';
    
    Motor1PosPrompt = 'What angle for motor 1? 0-300 ';
    Motor2PosPrompt = 'What angle for motor 2? 0-300 ';
    
    while(1)
        speed = input(speedPrompt,'s');
        if speed == 'y'
            Motor1Speed = int16(input(Motor1SpeedPrompt));
            Motor2Speed = int16(input(Motor2SpeedPrompt));
        elseif speed == 'n'
            
        end
        
        Motor1Pos = int16(input(Motor1PosPrompt));
        Motor2Pos = int16(input(Motor2PosPrompt));
        
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
        
        fwrite(t, outgoing);
        
    end
end