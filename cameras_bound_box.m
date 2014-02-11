function [ bound_box ] = cameras_bound_box( cameras )
%CAMERAS_BOUND_BOX From an array of cameras, calculate bound box.

%Get all camera FOV-polygons x's, and y's
cam_xs = []; cam_ys = [];
for i=1:length(cameras)
    cam_xs = [cam_xs; cameras(i).gen.fov_poly_world.x(:)];
    cam_ys = [cam_ys; cameras(i).gen.fov_poly_world.y(:)];
end

%Make bound box
bound_box.x = [min(cam_xs) max(cam_xs)];
bound_box.y = [min(cam_ys) max(cam_ys)];

end

