function [ can_it ] = camera_can_see( camera, x, y )
%CAMERA_CAN_SEE returns true if x,y is in camera's field of view.
    polyx = camera.gen.fov_poly_world.x;
    polyy = camera.gen.fov_poly_world.y;
    can_it = inpolygon(x,y,polyx,polyy);
end

