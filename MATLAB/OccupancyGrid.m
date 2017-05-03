classdef OccupancyGrid<handle
    %OCCUPANCYGRID Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Grid
        Resolution
        Landmarks
    end
    
    methods
        function obj = OccupancyGrid(numelX, numelY, resolution)
                
            obj.Grid = zeros(numelX / resolution, numelY / resolution);
            obj.Resolution = resolution;
        end
        
        function obj = addObservations(obj, x, y)
            x = round(x / obj.Resolution);
            y = round(y / obj.Resolution);
            if max(x) > size(obj.Grid, 1) || min(x) < 0 || max(y) > size(obj.Grid, 2) || min(y) < 0
                 error('some points are outside the occupancy grid');
            end
            
            for i = 1:length(x)
                obj.Grid(x(i), y(i)) = obj.Grid(x(i), y(i)) + 1;
            end
        end
        
        function obj = addLandmarks(obj, x, y)
            x = round(x / obj.Resolution);
            y = round(y / obj.Resolution);
            if max(x) > size(obj.Grid, 1) || min(x) < 0 || max(y) > size(obj.Grid, 2) || min(y) < 0
                error('some points are outside the occupancy grid');
            end
            obj.Landmarks = [x;y];
            
            
        end
        
        function obj = decrement(obj, ~)
            obj.Grid = obj.Grid - 1;
            obj.Grid(obj.Grid< 0) = 0;
        end
        
        function visualise(obj)
            imagesc(obj.Grid);
        end
    end
    
end

