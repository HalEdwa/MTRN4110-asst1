function PlotOOIs(OOIs, guiHandle)
    if OOIs.N<1, return ; end;
    
    theta = 0:0.1:2*pi;
    
    for i = 1:OOIs.N
        circX = OOIs.Centers.x(i) + OOIs.Sizes(i)*cos(theta);
        circY = OOIs.Centers.y(i) + OOIs.Sizes(i)*sin(theta);
        set(guiHandle.handle1, 'xdata', circX, 'ydata', circY);
    end
    
return;