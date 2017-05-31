clf
 % Calculate the shortest distance and path from point 3 to 5
 n = 5; A = AdjMatrix(n); xy = randi(10,n*n,2)
 [cost,path] = dijkstra(A,xy,1,3)
 figure(1);
 gplot(A,xy,'b.:'); hold on;
 plot(xy(path,1),xy(path,2),'ro-','LineWidth',2)
 for k = 1:n*n, text(xy(k,1),xy(k,2),[' ' num2str(k)],'Color','k'); end
 
