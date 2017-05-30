classdef OccupancyGrid<handle
    %OCCUPANCYGRID Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Grid
        Resolution
        Landmarks
        
        maxVal = 4000;
        incrAmount = 20;
        decrAmount = 2;
        decrMultiplier = 0.8;
    end
    
    methods
        function obj = OccupancyGrid(sizeX, sizeY, resolution)
                
            obj.Grid = zeros(sizeX / resolution, sizeY / resolution);
            obj.Resolution = resolution;
        end
        
        function obj = addObservations(obj, x, y, diameter)

            [x, y] = cleanPts(obj, x, y);
            shapeInserter = vision.ShapeInserter('Shape','Circles', 'BorderColor', 'white','Fill', true, 'FillColor', 'white');
            observations = [x' + round(size(obj.Grid, 1)/2), y', repmat(round(diameter' / obj.Resolution), numel(x), 1)];
            obj.Grid = obj.Grid + obj.incrAmount*shapeInserter(zeros(size(obj.Grid)), observations);

%             for i = 1:length(x)
%                 obj.Grid(1, 50) = 1000;%obj.Grid(10, 50) + obj.incrAmount;%
%                 obj.Grid(y(i), x(i) + round(size(obj.Grid, 1)/2)) = obj.Grid(y(i), x(i) + round(size(obj.Grid, 1)/2)) + obj.incrAmount;
%             end
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
        
        function OOIs = FindOOIs(obj)
            Threshold = 400;

            [OOIs.centers.x, OOIs.centers.y] = find(obj.Grid > Threshold);

            OOIs.centers.x = OOIs.centers.x * obj.Resolution; 
            OOIs.centers.y = OOIs.centers.y * obj.Resolution; 
            OOIs.N = numel(OOIs.centers.x);
        end
    end
    
end

function [x, y] = cleanPts(obj, x, y)
    x = round(x / obj.Resolution);
    y = round(y / obj.Resolution);
    goodpts = x < size(obj.Grid, 1)/2 & x > -size(obj.Grid, 1)/2 & y < size(obj.Grid, 2) & y > 0;
    if find(goodpts == 0)
        %for some reason there are lots of pts where y = 30. Why is this???
%          disp('OCCUPANCYGRID:WARNING: some points are outside the occupancy grid');
         y = y(goodpts);
         x = x(goodpts);
    end
end

















