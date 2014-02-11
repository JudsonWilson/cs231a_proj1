bound_box.x=[20,100];
bound_box.y=[-10,50];
object_std_dev = 0;
object_start_vel = 1; %units per tick
track = generate_track( bound_box, object_std_dev, object_start_vel)

%
% Plot the result
%

figure(1);
%clf
hold on;
%Plot bounding box
plot( [bound_box.x(1); bound_box.x(2); bound_box.x(2); bound_box.x(1); bound_box.x(1)], ...
      [bound_box.y(1); bound_box.y(1); bound_box.y(2); bound_box.y(2); bound_box.y(1)], ...
      'r');
%Plot Path
plot(track(:,1),track(:,2),'xg');
%Plot starting point
plot(track(1,1),track(1,2),'o'); 
axis equal;





