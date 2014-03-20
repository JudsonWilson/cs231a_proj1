function make_plots_camera_relation_votes( camera_relation_votes_and_centers, c1, c2 )
%MAKE_PLOTS_CAMERA_RELATION_VOTES Makes a figure with subplot scatters.
% Inputs:
%     relation_votes - an N x 3 list of (theta,r,theta) pairs

%If we have the "votes in window" field, use it, set up the data,
%otherwise don't
if  isfield(camera_relation_votes_and_centers,'votes_in_window')
    vw = camera_relation_votes_and_centers.votes_in_window{c1,c2};
    votes = camera_relation_votes_and_centers.votes{c1,c2};
    relation_votes_used = votes(vw,:);
    votes(vw,:) = [];
    relation_votes_notused = votes;
else
    relation_votes_used = camera_relation_votes_and_centers.votes{c1,c2};
    relation_votes_notused = zeros(0,3);
end
%Strings for variable labeling.
s1 = num2str(c1);
s2 = num2str(c2);
rstr  = ['r_{' s1 ',' s2 '}'];
t1str = ['\theta_{' s1 '|' s1 ',' s2 '}'];
t2str = ['\theta_{' s2 '|' s1 ',' s2 '}'];

formatstring_used    = '.k'; %for the points we used
formatstring_notused = 'ob'; %for the points we didn't use

subplot(2,2,1)
hold on;
plot(relation_votes_notused(:,1), relation_votes_notused(:,2),formatstring_notused,'markers',4);
plot(relation_votes_used   (:,1), relation_votes_used   (:,2),formatstring_used);
xlabel(t1str); ylabel(rstr);
ax = axis; ax([1,2,3]) = [-pi, pi, 0    ]; axis(ax);

subplot(2,2,2)
hold on;
plot(relation_votes_notused(:,2), relation_votes_notused(:,3),formatstring_notused,'markers',4);
plot(relation_votes_used   (:,2), relation_votes_used   (:,3),formatstring_used);
xlabel(rstr); ylabel(t2str);
ax = axis; ax([1 3 4]) = [0,    -pi, pi]; axis(ax);

subplot(2,2,3)
hold on;
plot(relation_votes_notused(:,3), relation_votes_notused(:,1),formatstring_notused,'markers',4);
plot(relation_votes_used   (:,3), relation_votes_used   (:,1),formatstring_used);
xlabel(t2str); ylabel(t1str);
ax = [-pi, pi, -pi, pi]; axis(ax);

subplot(2,2,4)
hold on;
plot3(relation_votes_used   (:,1), ...
      relation_votes_used   (:,2), ...
      relation_votes_used   (:,3),formatstring_used);
plot3(relation_votes_notused(:,1), ...
      relation_votes_notused(:,2), ...
      relation_votes_notused(:,3),formatstring_notused,'markers',4);
xlabel(t1str); ylabel(rstr); zlabel(t2str);
ax = axis; ax([1 2 3 5 6]) = [-pi, pi, 0,   -pi, pi]; axis(ax);


end

