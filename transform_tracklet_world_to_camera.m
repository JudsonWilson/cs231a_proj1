function tracklet_out = transform_tracklet_world_to_camera(cameras, tracklet)

Tw2c = cameras(tracklet.cam_num).gen.Tw2c;

% copy all the other fields of trackelt to tracklets_out
tracklet_out = tracklet;

% homogonize the points in 'path'. Note the path is in (x,y,t) format,
% so must strip off time, add a column of 1's.
homogonized = [tracklet.path(:,1:2) ones(size(tracklet.path,1),1)];

% transform into camera space. vectors are originally in row form, transpose
% matrix to put in columns
transformed = Tw2c*homogonized';

% unhomogonize the points, overwite the x,y, keep the time column.
unhomogonized = transformed(1:2,:);

% output needs to be transposed
tracklet_out.path(:,1:2) = unhomogonized';
