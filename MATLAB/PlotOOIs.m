function PlotOOIs(OOIs, guiHandle)
    if OOIs.N<1, return ; end;
    
    set(guiHandle.OOI, 'xdata', OOIs.Centers.x(:), 'ydata', OOIs.Centers.y(:));
    
%     theta = 0:0.1:2*pi;
%     
%     for i = 1:OOIs.N
%         circX = OOIs.Centers.x(i) + OOIs.Sizes(i)*cos(theta);
%         circY = OOIs.Centers.y(i) + OOIs.Sizes(i)*sin(theta);
%         set(guiHandle.handle1, 'xdata', circX, 'ydata', circY);
%     end
    pause(2)
return;