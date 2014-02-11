function [ output_args ] = plot_poly( poly )
%PLOT_POLY Summary of this function goes here
%   Detailed explanation goes here

%Get x coordinates and y coordinates as columns and force the last
% element to connect to the first element
x = poly.x(:);
x = [x; x(1)];
y = poly.y(:);
y = [y; y(1)];

plot(x,y);

end

