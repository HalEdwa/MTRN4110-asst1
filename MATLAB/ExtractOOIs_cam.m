<<<<<<< HEAD
function r = ExtractOOIs_cam(X, Y)
    filterSize = 0.03;
    poleDia = 0.09;
    poleDiaTol = 0.3;
=======
function OOI = ExtractOOIs_cam(x, z)
    DataLength = size(x);
    startPoint = 1;
    endPoint = 1;
>>>>>>> refs/remotes/origin/master
    
    OOI = repmat(struct('valid', false, 'width', 0, 'height',0, 'distance',0,'centralX',0,'centralY',0),1,10);
    objCounter = 0;
    
<<<<<<< HEAD
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
        objectSize = pdist([X(clusterEndPts(i)),  Y(clusterEndPts(i)); X(clusterEndPts(i+1)),  Y(clusterEndPts(i+1))]);
        if  (objectSize > (poleDia*(1 + poleDiaTol )))||( objectSize < (poleDia*(1 - poleDiaTol)))
            continue
        end
        
        r.N = r.N + 1;
        r.Centers.x = [r.Centers.x mean(X(clusterEndPts(i):clusterEndPts(i+1)))];
        r.Centers.y = [r.Centers.y mean(Y(clusterEndPts(i):clusterEndPts(i+1)))];
        r.Sizes = [r.Sizes pdist( [X(clusterEndPts(i)), Y(clusterEndPts(i));
                           X(clusterEndPts(i+1)), Y(clusterEndPts(i+1))])];
=======
    for scanIndex = 1:(DataLength(2)-1)
       if (abs(z(startPoint)-z(scanIndex+1)) <= 0.04) && (abs(x(scanIndex)- x(scanIndex+1)) <= 0.04)
           endPoint = scanIndex;
       else
           objWidth = abs(x(endPoint) - x(startPoint));
           if (objWidth >= 0.03) && (objWidth <= 0.08) && (objCounter <10) && ((endPoint-startPoint) >= 5)
               objCounter = objCounter + 1;
               OOI(objCounter).valid = true;
               OOI(objCounter).width = objWidth;
               OOI(objCounter).distance = (z(startPoint) + z(endPoint))/2;
               OOI(objCounter).centralX = (x(startPoint) + x(endPoint))/2;
           end 
           startPoint = scanIndex;
       end
>>>>>>> refs/remotes/origin/master
    end
return;
end

