function plane = generateFakePlane(p, n, noise)

%%
%if the point on the plane P is given by P = [x0, y0, z0]
%and the normal vector is given by N = [a, b, c] then the 
%equation of the plane is:
%a(x - x0) + b(y - y0) + c(z - z0) = 0
%so for a given series of x, y points you can just rearrange
%for z to find the series of points that make up the plane
%%

%since a plane is technically of infinite size, only generate
%a ss*ss square around the specified point
ss = 10;
[x, y] = ndgrid( (p(1) - ss/2):(p(1) + ss/2), (p(2) - ss/2):(p(2)) + ss/2 );

%generate dot product:
d = -p*n';
if(n(3) ~= 0)
    z = ( -n(1)*x - n(2)*y - d ) / n(3);
else
    z = ( -n(3)*x - n(1)*y - d) / n(2);%assumes n(2) ~= 0
end

plane = cat(3, x, y, z) + noise*randn([size(x), 3]);
%%
% NB: suggested to plot with:
% hold on;
% for i = 1:length(plane(1,:,1))
%     plot3(plane(:, i, 1), plane(:, i, 2), plane(:, i, 3), 'b*')
% end
%%
end