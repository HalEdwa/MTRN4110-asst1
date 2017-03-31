function OOI = ExtractOOIs_cam(x, z)
    DataLength = size(x); 
    startPoint = 1;
    endPoint = 1;
    
    OOI = repmat(struct('valid', false, 'width', 0, 'height',0, 'distance',0,'centralX',0,'centralY',0),1,10);
    objCounter = 0;
    
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
    end
return;
end

