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
    
    filterSize = 0.04;
    poleDia = 0.09;
    poleDiaTol = 0.3;
    clusterEndPts = [1];%have to initialise
    hues = zeros(1, length(x));
    newHue = 0;
    rng(9001);%repeatable random numbers
    for i = 1:100
        rand();
    end
    %find all clusters of points:
    for i = 2:length(x) - 1
        
        if (x(i) - x(i-1))^2 + (y(i) - y(i + 1))^2 > filterSize^2
            newHue = abs(rand());
            while newHue(1) > 90/255 && newHue(1) < 30/255
                newHue = abs(rand());
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
    
    for i = 1:2:length(clusterEndPts) - 1
    
          %for the moment, since we don't know what size the OOI is
          %leave the filtering based on size out
%         objectSize = pdist([X(clusterEndPts(i)),  Y(clusterEndPts(i)); X(clusterEndPts(i+1)),  Y(clusterEndPts(i+1))]);
%         if  (objectSize > (poleDia*(1 + poleDiaTol )))||( objectSize < (poleDia*(1 - poleDiaTol)))
%             continue
%         end
        
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
%     the below two lines visualise each cluster in a different colour
%     plot(X(clusterEndPts(i):clusterEndPts(i+1)), Y(clusterEndPts(i):clusterEndPts(i+1)), 'Color', hsv2rgb(lineColor), 'Marker', '+')%, 'LineStyle', 'none'
%     lineColor(1) = mod(lineColor(1) + pi, 1);
    x = x(hues ~= 0);
    y = y(hues ~= 0);
    hues = hues(hues ~= 0);
%     colours = hsv2rgb([rand(length(x), 1), ones(length(x), 1), ones(length(x), 1)])';
%     figure(10);
%     plot(1:length(hues), hues);
    colours = hsv2rgb([hues; ones(1, length(hues)); ones(1, length(hues))]');
    %     for i = 1:length(x)
%         set(h, 'xdata', x(1:i), 'ydata', y(1:i), 'cdata', colours(1:i)');
%         pause(0.1)
%     end

    set(h, 'xdata', x, 'ydata', y, 'cdata', colours);
    
   
    %%
return;
end





















