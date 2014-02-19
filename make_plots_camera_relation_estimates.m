function make_plots_camera_relation_estimates( theta1, r, theta2 )
%MAKE_PLOTS_CAMERA_RELATION_ESTIMATES Makes a figure with subplots 
% Inputs:
%     theta1, r, theta2 - estimated center of camera-relation votes.

plot_format_str = 'xr';

subplot(2,2,1)
hold on;
plot(theta1, r, plot_format_str);
xlabel('\theta_1'); ylabel('r');

subplot(2,2,2)
hold on;
plot(r, theta2, plot_format_str);
xlabel('r'); ylabel('\theta_2');

subplot(2,2,3)
hold on;
plot(theta2, theta1, plot_format_str);
xlabel('\theta_2'); ylabel('\theta_1');

subplot(2,2,4)
hold on;
plot3(theta1, r, theta2, 'xr');
xlabel('\theta_1'); ylabel('r'); zlabel('\theta_2');


end

