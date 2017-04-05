function PlotOOIs(OOI, guiH)
%    if OOIs.N<1, return ; end;
%    set(guiH, 'xdata', OOIs.Centers.x(:), 'ydata', OOIs.Centers.y(:));
    
    Index=1;
    
    markX = zeros(1, 10);
    markDistance= zeros(1, 10);
    
    while (Index <=10 )
        if OOI(Index).valid == true
            markX(Index) = OOI(Index).centralX;
            markDistance(Index) = OOI(Index).distance;
            Index = Index + 1;
        else
            break;
        end
    end
   
    markX = markX(markX~=0);
    markDistance = markDistance(markX~=0);
    
    set(guiH, 'xdata', markX(:), 'ydata', markDistance(:));
    
    pause(0.01);
return;
end