function make_plots_camera_relation_estimates( camera_relation_votes_and_centers, c1, c2 )
%MAKE_PLOTS_CAMERA_RELATION_ESTIMATES Makes a figure with subplots 
% Inputs:
%     theta1, r, theta2 - estimated center of camera-relation votes.

theta1 = camera_relation_votes_and_centers.centers.theta(c1,c2);
r      = camera_relation_votes_and_centers.centers.r(c1,c2);
theta2 = camera_relation_votes_and_centers.centers.theta(c2,c1);

%Strings for variable labeling.
s1 = num2str(c1);
s2 = num2str(c2);
rstr  = ['r_{' s1 ',' s2 '}'];
t1str = ['\theta_{' s1 '|' s1 ',' s2 '}'];
t2str = ['\theta_{' s2 '|' s1 ',' s2 '}'];

plot_format_str = 'xr';

subplot(2,2,1)
hold on;
plot(theta1, r, plot_format_str,'markers',12,'LineWidth', 2);
xlabel(t1str); ylabel(rstr);
ax = axis; ax([1,2,3]) = [-pi, pi, 0    ]; axis(ax);

subplot(2,2,2)
hold on;
plot(r, theta2, plot_format_str,'markers',12,'LineWidth', 2);
xlabel(rstr); ylabel(t2str);
ax = axis; ax([1 3 4]) = [0,    -pi, pi]; axis(ax);

subplot(2,2,3)
hold on;
plot(theta2, theta1, plot_format_str,'markers',12,'LineWidth', 2);
xlabel(t2str); ylabel(t1str);
ax = [-pi, pi, -pi, pi]; axis(ax);

subplot(2,2,4)
hold on;
plot3(theta1, r, theta2, plot_format_str,'markers',12,'LineWidth', 2);
xlabel(t1str); ylabel(rstr); zlabel(t2str);
ax = axis; ax([1 2 3 5 6]) = [-pi, pi, 0,   -pi, pi]; axis(ax);


end

