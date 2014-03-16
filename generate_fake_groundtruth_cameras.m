function [cameras] = generate_fake_groundtruth_cameras(config_num)
%GENERATE_FAKE_GROUNDTRUTH_CAMERAS returns a specific set of pregenerated
%cameras, with a lot of useful fields, as an array of camera structs.
%
% Available configuration numbers:
%  1 - The simple baseline 4-camera arrangement.
%  2 - A stretched out 10-camera arrangment, designed to reduce
%      connectivity a bit.
%
%
% Returns array of camera structs with the following fields:
%   .fov_poly_rel.x  - vector of x coordinates of field of view polygon,
%                          in relative (camera) coordinates.
%   .fov_poly_rel.y  - vector of x coordinates of field of view polygon,
%                          in relative (camera) coordinates.
%   .calib.x         - x position in world
%   .calib.y         - y position in world
%   .calib.theta     - angle of rotation in 2D world in radians
%   .gen.fov_poly_world - same as fov_poly_rel, but in world coordinates,
%                      i.e. translated and rotated.
%   .gen.Tc2w        - Homogeneous coordinate transform from camera coords
%                      to world coords.
%   .gen.Tw2c        - Homogeneous coordinate transform from world coords
%                      to camera coords.

allowed_config_nums = [1 2];
if all(config_num ~= allowed_config_nums)
    error(['generate_fake_groundtruth_cameras only knows configs: ' ...
           num2str(allowed_config_nums)]);
end

%General camera field of view polygon.
% Pointing in the x-axis direction, for an (x,y,theta)=(0,0,0)
fov_poly.x = [0 20 20 0];
fov_poly.y = [0 -6  6 0];

if config_num == 1
    camparams = [  0,   0,          0; %Keep this at zero for easy alignment of results.
                  30,  20, -pi*40/100;
                 -20, -10,  pi*10/100;
                   5,  15,  pi*70/100];
else
    camparams = [  0,   0,          0; %Keep this at zero for easy alignment of results.
                  30,  20, -pi*40/100;
                 -20, -10,  pi*10/100;
                   5,  15,  pi*70/100;
                  40,   5,          0;
                  10,   0,  pi*60/100;
                  35,  15,  -pi*70/100;
                 -30,   0,  pi*90/100;
                 -50, -10,  pi*60/100;
                 -20,   6,  pi*20/100];
end

for c=1:size(camparams,1)
    camera.fov_poly_rel = fov_poly;
    camera.calib.x = camparams(c,1);
    camera.calib.y = camparams(c,2);
    camera.calib.theta = camparams(c,3);
    cameras(c) = camera_put_in_world(camera);
end

end
