load('recordedCameraData_20170407.mat');


o = OccupancyGrid(3, 3, 0.05);

for i = 1:length(recordedData(1, 1, :))
    x = recordedData(1, :, i);
    y = recordedData(2, : i);
    z 
end
