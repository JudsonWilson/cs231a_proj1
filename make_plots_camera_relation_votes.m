function make_plots_camera_relation_votes( relation_votes )
%MAKE_PLOTS_CAMERA_RELATION_VOTES Makes a figure with subplot scatters.
% Inputs:
%     relation_votes - an N x 3 list of (theta,r,theta) pairs

subplot(2,2,1)
plot(relation_votes(:,1), relation_votes(:,2),'o');
hold on;
plot(mean(relation_votes(:,1)), mean(relation_votes(:,2)),'xr');
xlabel('\theta_1'); ylabel('r');

subplot(2,2,2)
plot(relation_votes(:,2), relation_votes(:,3),'o');
hold on;
plot(mean(relation_votes(:,2)), mean_angle(relation_votes(:,3)),'xr');
xlabel('r'); ylabel('\theta_2');

subplot(2,2,3)
plot(relation_votes(:,3), relation_votes(:,1),'o');
hold on;
plot(mean_angle(relation_votes(:,3)), mean(relation_votes(:,1)),'xr');
xlabel('\theta_2'); ylabel('\theta_1');

subplot(2,2,4)
plot3(relation_votes(:,1), ...
      relation_votes(:,2), ...
      relation_votes(:,3),'o');
hold on;
plot3(mean      (relation_votes(:,1)), ...
      mean      (relation_votes(:,2)), ...
      mean_angle(relation_votes(:,3)),'xr');
xlabel('\theta_1'); ylabel('r'); zlabel('\theta_2');


end

