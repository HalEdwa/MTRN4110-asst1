function A = AdjMatrix( N )
%AdjMatrix makes an adjaceny matrix for a grid of N by N
addcol = zeros(N*N,1); % an empty row 
addrow = zeros(1, N*N);
I = sparse(eye(N*N));
Igap = I;

for gap = 3:3:N*N
    Igap(gap,:) = 0; % puts in the gap for diagonal r2 and col2
end 

for row = 1:N*N
    IcolShift = [addcol, addcol, addcol, I(:,1:N*N-3)]; % shift the identity matrix 3 columns right
    IrowShift = [addrow; addrow; addrow; I(1:N*N-3,:)]; % shift the identity matrix 3 rows down
    IgapcolShift = [addcol, Igap(:,1:N*N-1)]; % shift the identity matrix 3 columns right
    IgaprowShift = [addrow; Igap(1:N*N-1,:)]; % shift the identity matrix 3 rows down
end 

A = IcolShift+ IrowShift+IgapcolShift+IgaprowShift;

end

