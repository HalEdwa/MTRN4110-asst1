function r = ExtractOOIs(Y)
    maxPoleDia = 0.2;
    r.N = 0;
    r.Centers.x = [];
    r.Centers.y = [];
    r.Sizes   = [];
    clusterEndPts = [1];%have to initialise

    X = [1:length(ScanArray)];
    
    %find all clusters of points:
    for i = 2:length(ranges) - 1
        if (X(i) - X(i-1))^2 + (Y(i) - Y(i-1))^2 > maxPoleDia^2
            if (X(i) - X(i+1))^2 + (Y(i) - Y(i+1))^2 > maxPoleDia^2
                clusterEndPts = [clusterEndPts, i - 1, i + 1];
                i = i + 1;
            else 
                clusterEndPts = [clusterEndPts, i - 1, i];
            end
        end
    end

    lineColor = [0 1 1];
    for i = 1:2:length(clusterEndPts) - 1
        if ~any(find(brightPts > clusterEndPts(i) & brightPts < clusterEndPts(i+1)))
            continue
        end
        if pdist([X(clusterEndPts(i)),  Y(clusterEndPts(i)); X(clusterEndPts(i+1)),  Y(clusterEndPts(i+1))]) > maxPoleDia
            continue
        end
        r.N = r.N + 1;
        r.Centers.x = [r.Centers.x mean(X(clusterEndPts(i):clusterEndPts(i+1)))];
        r.Centers.y = [r.Centers.y mean(Y(clusterEndPts(i):clusterEndPts(i+1)))];
        r.Sizes = [r.Sizes pdist( [X(clusterEndPts(i)), Y(clusterEndPts(i));
                           X(clusterEndPts(i+1)), Y(clusterEndPts(i+1))])];
%         plot(X(clusterEndPts(i):clusterEndPts(i+1)), Y(clusterEndPts(i):clusterEndPts(i+1)), 'Color', hsv2rgb(lineColor), 'Marker', '+')%, 'LineStyle', 'none'
%         lineColor(1) = mod(lineColor(1) + pi, 1);
%         axis([-10,10,0,20]);
    %     pause(0.1)
    end
%     plot(X, Y, 'k.')
%     plot(X(brightPts), Y(brightPts), 'r.')
return;
end