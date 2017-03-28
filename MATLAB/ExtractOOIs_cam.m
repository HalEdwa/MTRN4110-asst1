function r = ExtractOOIs_cam(X, Y)
    filterSize = 0.02;
    poleDia = 0.09;
    poleDiaTol = 0.3;
    
    r.N = 0;
    r.Centers.x = [];
    r.Centers.y = [];
    r.Sizes   = [];
    clusterEndPts = [1];%have to initialise
    
    %find all clusters of points:
    for i = 2:length(X) - 1
        meanY = mean(Y(clusterEndPts(end):i));%range data is very noisy so take average range within current cluster
        
        if (X(i) - X(i-1))^2 + (Y(i) - meanY)^2 > filterSize^2
            if (X(i) - X(i+1))^2 + (Y(i) - Y(i+1))^2 > filterSize^2
                clusterEndPts = [clusterEndPts, i - 1, i + 1];
                i = i + 1;
            else 
                clusterEndPts = [clusterEndPts, i - 1, i];
            end
        end
    end

    clusterEndPts = [clusterEndPts, length(X)];
    lineColor = [0 1 1];
    for i = 1:2:length(clusterEndPts) - 1
        %the below two lines visualise each cluster in a different colour
%         plot(X(clusterEndPts(i):clusterEndPts(i+1)), Y(clusterEndPts(i):clusterEndPts(i+1)), 'Color', hsv2rgb(lineColor), 'Marker', '+')%, 'LineStyle', 'none'
%         lineColor(1) = mod(lineColor(1) + pi, 1);
    
          %for the moment, since we don't know what size the OOI is
          %leave the filtering based on size out
%         objectSize = pdist([X(clusterEndPts(i)),  Y(clusterEndPts(i)); X(clusterEndPts(i+1)),  Y(clusterEndPts(i+1))]);
%         if  (objectSize > (poleDia*(1 + poleDiaTol )))||( objectSize < (poleDia*(1 - poleDiaTol)))
%             continue
%         end
        
        r.N = r.N + 1;
        r.Centers.x = [r.Centers.x mean(X(clusterEndPts(i):clusterEndPts(i+1)))];
        r.Centers.y = [r.Centers.y mean(Y(clusterEndPts(i):clusterEndPts(i+1)))];
        r.Sizes = [r.Sizes pdist( [X(clusterEndPts(i)), Y(clusterEndPts(i));
                           X(clusterEndPts(i+1)), Y(clusterEndPts(i+1))])];
    end
return;
end
