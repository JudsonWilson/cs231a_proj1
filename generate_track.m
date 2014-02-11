function [ track ] = generate_track( bound_box, object_std_dev, object_start_vel)
%GENERATE_TRACK Generate a track within a bound box
%
% TODO: Generate curved tracks.
% TODO: Generate curved tracks.
% TODO: Generate curved tracks.
% TODO: Generate curved tracks.
% TODO: Generate curved tracks.
%
%
%


%Function for a random variable on range [-1,1], using a clipped normal
%distribution with stddev=0.5:
randnclip = @() max(min(randn(1,1)*0.5,1),-1);


%State variable, x, y, x_prime, y_prime
state = [0;0;0;0]; 

%Pick a starting point on the bounding box, and a direction / speed that
%makes sense (pointing inward into the bounding box).
bound_box.x = sort(bound_box.x, 'ascend'); %Lowest first
bound_box.y = sort(bound_box.y, 'ascend'); %Lowest first
%Start on top or bottom
if rand(1) > 0.5
    state(1) = rand(1)*(bound_box.x(2)-bound_box.x(1)) + bound_box.x(1);
    %bottom
    if rand(1) > 0.5
        angle = pi/2*randnclip() + pi/2;
        state(2) = bound_box.y(1);
    %top
    else
        angle = pi/2*randnclip() + pi*3/2;
        state(2) = bound_box.y(2);
    end
%Start on left or right
else
    state(2) = rand(1)*(bound_box.y(2)-bound_box.y(1)) + bound_box.y(1);
    %left
    if rand(1) > 0.5
        angle = pi/2*randnclip();
        state(1) = bound_box.x(1);
    %right
    else
        angle = pi/2*randnclip() + pi;
        state(1) = bound_box.x(2);
    end
end
state(3:4) = object_start_vel*[cos(angle); sin(angle)];
    
    
%
% Time step on travel path
%

%Starting point in track
track = [state(1) state(2)];

hold on;
keepgoing = true;
while keepgoing 
    state = [1 0 1 0;
             0 1 0 1;
             0 0 1 0;
             0 0 0 1] * state;
	if(state(1) < bound_box.x(1) || state(1) > bound_box.x(2) || state(2) < bound_box.y(1) || state(2) > bound_box.y(2))
        keepgoing = false;
    else
        %Add new point to track
        track = [track; state(1) state(2)];
    end
end


end

