function [ X ] = mds_map( D )
%MDS_MAP Given a distance matrix D, return a 2D approximation.
% Inputs:
%     D - The distance matrix between points. Missing entries should
%         be set to inf (or a number greater than 1e20). Matrix will be
%         converted to symmetric using the lowest distance of 
%         x1->x2 or x2->x1, etc.
% Outputs:
%     X - 2d list of points, where rows are points.
%
% Source: Adapted from Judson Wilson's Homework for EE378b, Stanford
%     University 2013.
%     Also uses information from:
%         Sensor Network Localization from Local Connectivity :
%             Performance Analysis for the MDS-MAP Algorithm
%                 Oh, Andrea Montanari, Amin Karbasi

if ndims(D) ~= 2 || size(D,1) ~= size(D,2) || size(D,1) < 2
    error('Improperly sized distance matrix D.');
end

N = size(D,1);

% Force symmetry by choosing smallest distance if different values given
% for a pair, e.g. smallest of x1->x2 or x2->x1
% Do this by stacking the matrices on top of eachother, creating a 3rd 
% dimension, and finding the minimum in this 3rd dimension.
Dsym = min(cat(3,D,D'),[],3);

% Fill in shortest paths for the missing entries
Dfull = Dsym;
for i=1:N
    % Find shortests paths for all vertices to this vertex
    short_distances = mds_map_shortest_path_distances(Dsym,i);
    % Figure out which distances to replace by finding ones that are
    % infinite (more precisely, greater than 1e20, which is used as a
    % marker).
    indices = Dsym(i,:) > 1e20;
    Dfull(i,indices) = short_distances(indices);
end

%Do MDS-MAP

Dfull_sqr = Dfull.^2;

u=ones(N,1);

P = eye(N) - u*(u')/N;

Q = (-1/2)*P*Dfull_sqr*P;

%  opts.tol = 1e-7;
%  opts.maxit = 1e5;
%  opts.disp = 1;
%  [V,S] = eigs(Q,2,'lr', opts);

power_iterations = 1000;
[V sigma] = powmeth_rightvecs(Q,2,power_iterations);
S = diag(sigma);

%Force eigenvectors into a certain orientation for display purposes
for i=1:2
    if V(1,i) < 0
        V(:,i) = V(:,i)*-1;
    end
end

X = V*sqrt(S); 

end

