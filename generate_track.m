function [ track ] = generate_track( bound_box,  object_start_speed, shape_noise_factor, measurement_noise_factor)
%GENERATE_TRACK Generate a track within a bound box. Paths are smooth
%  curves of (programmably) random curviness, with gaussian measurement
%  noise added.
%
%  shape_noise_factor determines the acceleration, and change of
%      acceleration of the point, to make smoothish curvey paths with
%      changing speed. Set this to 0 for constant-speed straight lines
%      (plus any noise you add from the measurement_noise_factor).
%  measurement_noise_factor adds clipped gaussian noise to the final result
%      Set this to 0 if you want really smooth paths.

%Function for a random variable on range [-1,1], using a clipped normal
%distribution with stddev=0.5:
randnclip = @() max(min(randn(1,1)*0.5,1),-1);

%State variable, with x,y and derivatives up to the 3rd
state = [0;  % x
         0;  % y
         0;  % x'  
         0;  % y'
         0;  % x''
         0;  % y''
         0;  % x'''
         0]; % y'''

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

%Starting point in track
track = [state(1) state(2)];

%Randomize start speed
speed = object_start_speed * (2^(randnclip()*2+1)); %scale by random amount (2^-2 to 2^2)
state(3:4) = speed*[cos(angle); sin(angle)];

%Random 2nd and 3rd derivatives (acceleration, jerk!)
angle = angle + randnclip()*2/3*pi*shape_noise_factor;
state(5:6) = randnclip() * 0.005 * speed/object_start_speed * shape_noise_factor * [cos(angle); sin(angle)];  %acceleration
angle = angle + randnclip()*2/3*pi*shape_noise_factor;
state(7:8) = randnclip() * 0.005 * speed/object_start_speed * shape_noise_factor * [cos(angle); sin(angle)];  %acceleration-dot

% Time step the linear dynamical system.
keepgoing = true;
while keepgoing
    %Adds 1 unit of all derivatives to the next lowest order derivative.
    %  (e.g. acceleration -> speed,   speed -> position).
    state = [1 0 1 0 0 0 0 0;
             0 1 0 1 0 0 0 0;
             0 0 1 0 1 0 0 0;
             0 0 0 1 0 1 0 0;
             0 0 0 0 1 0 1 0;
             0 0 0 0 0 1 0 1;
             0 0 0 0 0 0 1 0;
             0 0 0 0 0 0 0 1] * state;
    
    %detect end of path
	if(state(1) < bound_box.x(1) || state(1) > bound_box.x(2) || state(2) < bound_box.y(1) || state(2) > bound_box.y(2))
        keepgoing = false;
    else
        %Add new point to track
        track = [track; state(1) state(2)];
    end
end

%Add clipped gaussian noise. Tries to scale with velocity so that slow moving
%things bounce around less. Not 100% that is the right thing to do.
track = track + randn(size(track))*measurement_noise_factor/4*object_start_speed;

end

