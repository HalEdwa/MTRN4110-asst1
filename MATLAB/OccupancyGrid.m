classdef OccupancyGrid<handle
    %OCCUPANCYGRID Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Grid
        Resolution
    end
    
    methods
        function obj = OccupancyGrid(numelX, numelY, resolution)
                
            obj.Grid = zeros(numelX, numelY);
            obj.Resolution = resolution;
        end
        
%         function obj = addObservations(x, y)
%             x = round(x / Resolution);
%             y = round(y / Resolution);
%             for i = 1:length(x)
%                 Resolution(x(i), y(i)) = Resolution(x(i), y(i)) + 1;
%             end
%         end
        
        function obj = decrement(obj, ~)
            obj.Grid = obj.Grid - 1;
            obj.Grid(obj.Grid< 0) = 0;
        end
    end
    
end

