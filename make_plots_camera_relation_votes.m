function make_plots_camera_relation_votes( camera_relation_votes_and_centers, c1, c2 )
%MAKE_PLOTS_CAMERA_RELATION_VOTES Makes a figure with subplot scatters.
% Inputs:
%     relation_votes - an N x 3 list of (theta,r,theta) pairs

relation_votes = camera_relation_votes_and_centers.votes{c1,c2};

%Strings for variable labeling.
s1 = num2str(c1);
s2 = num2str(c2);
rstr  = ['r_{' s1 ',' s2 '}'];
t1str = ['\theta_{' s1 '|' s1 ',' s2 '}'];
t2str = ['\theta_{' s2 '|' s1 ',' s2 '}'];

formatstring = '.';

subplot(2,2,1)
hold on;
plot(relation_votes(:,1), relation_votes(:,2),formatstring);
%plot(mean(relation_votes(:,1)), mean(relation_votes(:,2)),'xr');
xlabel(t1str); ylabel(rstr);
ax = axis; ax([1,2,3]) = [-pi, pi, 0    ]; axis(ax);

subplot(2,2,2)
hold on;
plot(relation_votes(:,2), relation_votes(:,3),formatstring);
%plot(mean(relation_votes(:,2)), mean_angle(relation_votes(:,3)),'xr');
xlabel(rstr); ylabel(t2str);
ax = axis; ax([1 3 4]) = [0,    -pi, pi]; axis(ax);

subplot(2,2,3)
hold on;
plot(relation_votes(:,3), relation_votes(:,1),formatstring);
%plot(mean_angle(relation_votes(:,3)), mean(relation_votes(:,1)),'xr');
xlabel(t2str); ylabel(t1str);
ax = [-pi, pi, -pi, pi]; axis(ax);

subplot(2,2,4)
hold on;
plot3(relation_votes(:,1), ...
      relation_votes(:,2), ...
      relation_votes(:,3),formatstring);
%plot3(mean      (relation_votes(:,1)), ...
%      mean      (relation_votes(:,2)), ...
%      mean_angle(relation_votes(:,3)),'xr');
xlabel(t1str); ylabel(rstr); zlabel(t2str);
ax = axis; ax([1 2 3 5 6]) = [-pi, pi, 0,   -pi, pi]; axis(ax);


end

