function [ camera ] = camera_put_in_world( camera )
%CAMERA_PUT_IN_WORLD takes the calibration information, and generates
% the external calibration - i.e. things in world coordinates.

%Get the uncalibrated fov-poly, we will rotate it to the correct angle
% and correct location, in world coordinates
cam_poly = [camera.fov_poly_rel.x(:)'; camera.fov_poly_rel.y(:)'];

%rotate
angle = camera.calib.theta;
rot_matrix = [cos(angle) -sin(angle); sin(angle) cos(angle)];
cam_poly = rot_matrix * cam_poly;
%translate
cam_poly = bsxfun(@plus,cam_poly, [camera.calib.x; camera.calib.y]);

%store world coordinats of poly
camera.gen.fov_poly_world.x = cam_poly(1,:)';
camera.gen.fov_poly_world.y = cam_poly(2,:)';

end

