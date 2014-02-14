function [ camera ] = camera_put_in_world( camera )
%CAMERA_PUT_IN_WORLD takes the calibration information, and generates
% the external calibration - i.e. things in world coordinates.

%Get the uncalibrated fov-poly in homogenious coordinates,
% we will rotate it to the correct angle and location, in world coordinates
cam_poly = [camera.fov_poly_rel.x(:)';
            camera.fov_poly_rel.y(:)';
            ones(1, length(camera.fov_poly_rel.x)) ];

%transform matrix 
angle = camera.calib.theta;
Tc2w = [cos(angle) -sin(angle) camera.calib.x;
        sin(angle)  cos(angle) camera.calib.y;
           0           0             1       ];

cam_poly = Tc2w*cam_poly;

%store world coordinats of poly
camera.gen.fov_poly_world.x = cam_poly(1,:)';
camera.gen.fov_poly_world.y = cam_poly(2,:)';
%store camera Transform matrix,camera->world and world->camera
camera.gen.Tc2w = Tc2w;
camera.gen.Tw2c = inv(Tc2w);

end

