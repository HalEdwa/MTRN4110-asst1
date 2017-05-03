function r = ExtractOOIs_cam(x, y, h)
    r.N = 0;
    r.centers.x = [];
    r.centers.y = [];
    r.Sizes   = [];
    
    if numel(x) + numel(y) == 0
        return;
    end
    
    %since the data contains several scan lines, it is no longer an ordered
    %dataset. Fix this by converting to polar form, ordering based on
    %theta, then changing back to cartesian
    unorderedRanges = sqrt(x.^2 + y.^2);
    unorderedTheta = atan(y./x);
    [theta, order] = sort(unorderedTheta);
    ranges = unorderedRanges(order);
    x = ranges .* cos(theta);
    y = ranges .* sin(theta);
    
    filterSize = 0.03;
    poleDia = 0.04;
    poleDiaTol = 0.3;
    clusterEndPts = [1];%have to initialise
    hues = zeros(1, length(x));
    newHue = 0.3;
    purple = 213/255;
    %find all clusters of points:
    for i = 2:length(x) - 1
        
        if (x(i) - x(i-1))^2 + (y(i) - y(i - 1))^2 > filterSize^2
            newHue = mod(newHue + 0.73, 1);
            
            %filter yellow because its hard to see and purple because it's 
            %reserved:
            while (newHue(1) > 90/255 && newHue(1) < 30/255 ) || (newHue(1) > 200/255 && newHue(1) < 230/255)
                newHue = mod(newHue + 0.73, 1);
            end

            if (x(i) - x(i+1))^2 + (y(i) - y(i+1))^2 > filterSize^2
                clusterEndPts = [clusterEndPts, i - 1, i + 1];
                hues(:, i) = 0;%pts not in a cluster are black
                i = i + 1;
            else 
                clusterEndPts = [clusterEndPts, i - 1, i];
            end
        end
        hues(:, i) = newHue;
    end

    clusterEndPts = [clusterEndPts, length(x)];
    
    %the ends of these clusters tend to have points that have fallen off
    %the back. Trim the size of the clusters:
    for i = 1:2:length(clusterEndPts)
        meanX = mean( x(clusterEndPts(i):clusterEndPts(i+1)) );
        %trim beginning:
        while abs(x(clusterEndPts(i)) - meanX) > poleDia*0.6%tolerance is arbitrary
            hues(clusterEndPts(i)) = 0;
            clusterEndPts(i) = clusterEndPts(i) + 1;
            meanX = mean( x(clusterEndPts(i):clusterEndPts(i+1)) );
        end
        %trim end:
        while abs(x(clusterEndPts(i+1)) - meanX) > poleDia*0.6%tolerance is arbitrary
            hues(clusterEndPts(i+1)) = 0;
            clusterEndPts(i+1) = clusterEndPts(i+1) - 1;
            meanX = mean( x(clusterEndPts(i):clusterEndPts(i+1)) );
        end
    end
    
    for i = 1:2:length(clusterEndPts) - 1
    
        %the 3D camera tends to have trailing points on the outside of
        %clusters. So compare the y distances only to ignore these:
        objectSize = abs(y(clusterEndPts(i)) - y(clusterEndPts(i+1)));
        if  (objectSize < (poleDia*(1 + poleDiaTol )))&&( objectSize > (poleDia*(1 - poleDiaTol)))
            hues(clusterEndPts(i):clusterEndPts(i+1)) = purple;
        end
        
        r.N = r.N + 1;
        r.centers.x = [r.centers.x mean(x(clusterEndPts(i):clusterEndPts(i+1)))];
        r.centers.y = [r.centers.y mean(y(clusterEndPts(i):clusterEndPts(i+1)))];
        r.Sizes = [r.Sizes pdist( [x(clusterEndPts(i)), y(clusterEndPts(i));
                           x(clusterEndPts(i+1)), y(clusterEndPts(i+1))])];
     
    end
    %% plotting:
    if ~isvalid(h)
        return
    end

    x = x(hues ~= 0);
    y = y(hues ~= 0);
    hues = hues(hues ~= 0);
    vals = ones(1, length(hues));
    vals(hues ~= purple) = 0.6;
    colours = hsv2rgb([hues; ones(1, length(hues)); vals]');
    
%     for i = 10:length(x)
%         set(h, 'xdata', x(1:i), 'ydata', y(1:i), 'cdata', colours(1:i, :));
%         pause(0.05)
%     end

    set(h, 'xdata', x, 'ydata', y, 'cdata', colours);
    
   
    %%
return;
end





















