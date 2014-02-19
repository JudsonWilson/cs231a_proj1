function [distances] = shortest_path_distances(D, last)
%SHORTEST_PATH Calculate shortest path from all points to last point using
%  distances D. Implemented using Dynamic Programming.
%
% Source: Based on Judson Wilson's Homework for EE378b, Stanford
%     University 2013,

%In dynamic programming, we start at the end, and then we progress
% toward the beginning. For each step, we compute for every node
% the shortest path to the end by finding the minimum sum of the distance
% to each other node, and the shortest distance from that not to the end
% (i.e. the result of the previous step).
S = inf*ones(size(D,1),size(D,1)+2);

%First step - start at the end
S(last,end) = 0;

%March backwards
for i=(size(S,2)-1):-1:1
    %for each node
    for v=1:size(S,1)
        %Find the minimum distance of sum of distances to other nodes
        % and those nodes S of previous step.
        [y,ind] = min(D(:,v) + S(:,(i+1)));
        S(v,i) = y;
    end
    %early quit for speed improvement
    if isequal(S(:,i), S(:,i+1))
        distances = S(:,i);
        return;
    end
end

%distance = S(first,1);
    
distances = S(:,1);


end

