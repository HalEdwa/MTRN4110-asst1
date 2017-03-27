
% Create TCP/IP object 't'. Specify server machine and port number. 
t = tcpip('127.0.0.1', 15000); 

% Set size of receiving buffer, if needed. 
set(t, 'InputBufferSize', 12); 

% Open connection to the server. 
fopen(t); 

 pause(1)
% Receive lines of data from server 
t.BytesAvailable 
DataReceived = fscanf(t)
pause(1)
h = animatedline('Color','r');
g = animatedline('Color','g');
k = animatedline('Color','b');
axis([0,100,-1,257])

tic

while (get(t, 'BytesAvailable') > 0) 
t.BytesAvailable 
DataReceived = fscanf(t)
A1 = DataReceived(2:4);
A2 = DataReceived(6:8);
A3 = DataReceived(10:12);
a1= str2double(A1)
a2 = str2double(A2)
a3 = str2double(A3)
dt = toc

addpoints(h,dt,a1);
addpoints(g,dt,a2);
addpoints(k, dt, a3);
drawnow


%end calc bits

pause(1)
end

% Disconnect and clean up the server connection. 
fclose(t); 
delete(t); 
clear t 