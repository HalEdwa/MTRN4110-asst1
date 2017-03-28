function PlotOOIs(OOIs, guiH)
    if OOIs.N<1, return ; end;
    
    set(guiH, 'xdata', OOIs.Centers.x(:), 'ydata', OOIs.Centers.y(:));
return;
end