function [ tracklets ] = sort_tracklets_by_camera( tracklets )
%SORT_TRACKLETS_BY_CAMERA Sort tracklets by camera.

camera_indices = zeros(length(tracklets),1);

for i=1:length(tracklets)
    camera_indice(i) = tracklets{i}.cam_num;
end

[~,i] = sort(camera_indices,'ascend');
tracklets = tracklets(i);


end

