function tracklet_out = transform_tracklet_world_to_camera(cameras, tracklet)

Tw2c = cameras(tracklet.cam_num).gen.Tw2c;

% copy all the other fields of trackelt to tracklets_out
tracklet_out = tracklet;

% homogonize the points in 'path'
homogonized = [tracklet.path ones(size(tracklet.path,1),1)];

% transform into camera space. vectors are originally in row form, transpose
% matrix to put in columns
transformed = Tw2c*homogonized';

% unhomogonize the points
unhomogonized = transformed(1:2,:);

% output needs to be transposed
tracklet_out.path = unhomogonized';
