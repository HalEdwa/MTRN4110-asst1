classdef OccupancyGrid<handle
    %OCCUPANCYGRID Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Grid
        Resolution
        Landmarks
        
        maxVal = 4000;
        incrAmount = 20;
        decrAmount = 10;
        decrMultiplier = 0.8;
    end
    
    methods
        function obj = OccupancyGrid(sizeX, sizeY, resolution)
                
            obj.Grid = zeros(sizeX / resolution, sizeY / resolution);
            obj.Resolution = resolution;
        end
        
        function obj = addObservations(obj, x, y)
%                 camerax = 1.5;
                cameray = 1.5;
%                 x = x + camerax;
                y = y + cameray;

            [x, y] = cleanPts(obj, x, y);
            
            for i = 1:length(x)
                obj.Grid(x(i), y(i)) = obj.Grid(x(i), y(i)) + obj.incrAmount;
            end
            obj.Grid(obj.Grid > obj.maxVal) = obj.maxVal;
        end
        
        function obj = addLandmarks(obj, x, y)
            [x, y] = cleanPts(obj, x, y);
            obj.Landmarks = [x;y];
        end
        
        function obj = decrement(obj, ~)
            obj.Grid = obj.Grid - obj.decrAmount;
            obj.Grid = round(obj.Grid * obj.decrMultiplier);
            
            obj.Grid(obj.Grid< 0) = 0;
        end
        
        function visualise(obj, h)
            if numel(obj.Landmarks > 0)
                hues = zeros(size(obj.Grid));%hue channel
                hues = hues + 0.5;
                %hues = obj.Grid / (max(max(obj.Grid)));
                landMarkIdx = sub2ind(size(hues), obj.Landmarks(1, :), obj.Landmarks(2, :));
                hues(landMarkIdx) = 0;%red
                cdata = ones(size(obj.Grid, 1), size(obj.Grid, 1), 3);
                cdata(:, :, 1) = hues;
                cdata = hsv2rgb(cdata);
                
                heightData = obj.Grid;
                heightData(landMarkIdx) = max(max(heightData));
                set(h, 'cdata', cdata, 'zdata', heightData);
            else
                set(h, 'cdata', obj.Grid, 'zdata', obj.Grid);
            end
        end
        
    end
    
end

function [x, y] = cleanPts(obj, x, y)
    x = round(x / obj.Resolution);
    y = round(y / obj.Resolution);
    goodpts = x < size(obj.Grid, 1) & x > 0 & y < size(obj.Grid, 2) & y > 0;
    if find(goodpts == 0)
        %for some reason there are lots of pts where y = 30. Why is this???
%          disp('OCCUPANCYGRID:WARNING: some points are outside the occupancy grid');
         y = y(goodpts);
         x = x(goodpts);
    end
end

















