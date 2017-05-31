function [ PathCoordinates ] = FindFastestRoute( OccupancyMatrix , currentPos, Goal)
%find fastest route takes in the occupancy grid generated from the scanner, 
% applies Dijkstras algorithm to the grid then spits out a path in coordinates
% on that grid 

%OCCU = randi(20,60,60);

% for testing a random set up of integers between 1-20 
%  for a grid of 60x60




[n,m] = size(OccupancyMatrix); % m should = n

if (n==m)
    A = getAdjacencyMatrix(n);
    C = CostMatrix(A,OccupancyMatrix);
    [cost,path] = dijkstra(A,C,currentPos,Goal);
    disp(cost)
    PathCoordinates = Route(path');
end 

end

function W = getAdjacencyMatrix(n)
% from stackoverflow user mnmltype, Oct 10 '13 at 11:49
%[m, n] = size(I);
m=n;
I_size = n*n;

% 1-off diagonal elements
V = repmat([ones(m-1,1); 0],n, 1);
V = V(1:end-1); % remove last zero

% n-off diagonal elements
U = ones(m*(n-1), 1);

% get the upper triangular part of the matrix
W = sparse(1:(I_size-1),    2:I_size, V, I_size, I_size)...
  + sparse(1:(I_size-m),(m+1):I_size, U, I_size, I_size);

% finally make W symmetric
W = W + W';
end 

function [ C ] = CostMatrix( AdjM, OccM )
%CostMatrix makes a matrix of the cost of getting from one grid point to another 
% for use in Dijkstras algorithm, takes in the adjacency matrix and
% occupancy matrix.
% needs a square occupancy grid

V = reshape(OccM,[],1); % makes the occupancy matrix into a vector 
temp = AdjM.*diag(V); % every row of the adjacency matrix is multiplied by V

C = temp'; % transposed again to orientate it correctly 


end

function [ Coordinates ] = Route( path )
%route takes in the path and returns the index coordinates 
% this may result in some funky translated matrix
%   Detailed explanation goes here

p = length(path);
Coordinates = zeros(p,2);

for i = 1:p
    Coordinates(i,2) = mod(path(i), 60); % takes the remainder after /60 
end
for i = 1:p
    Coordinates(i,1) = floor(path(i)/60)+1; % takes the remainder after /60 
end


end 