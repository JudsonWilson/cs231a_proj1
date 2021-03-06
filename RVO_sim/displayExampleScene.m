%% Display the Scene
clf
figure(1)
axis([10 90 10 40])
% axis([-75 175 -75 125])
% patch([-75 175 175 -75 -75],[-75 -75 125 125 -75],'b')
% Obstacles
% (-100,-100);(  19,-100);(  19,  17);(-100,  17);
% (-100,  33);(  19,  33);(  19, 150);(-100, 150);
% (  27,-100);(  46,-100);(  46,  17);(  27,  17);
% (  27,  33);(  46,  33);(  46, 150);(  27, 150);
% (  54,-100);(  73,-100);(  73,  17);(  54,  17);
% (  54,  33);(  73,  33);(  73, 150);(  54, 150);
% (  81,-100);( 200,-100);( 200,  17);(  81,  17);
% (  81,  33);( 200,  33);( 200, 150);(  81, 150);
obstacles = {[-100,  19,  19,-100,-100;-100,-100,  17,  17,-100], ...
             [-100,  19,  19,-100,-100;  33,  33, 150, 150,  33], ...
             [  27,  46,  46,  27,  27;-100,-100,  17,  17,-100], ...
             [  27,  46,  46,  27,  27;  33,  33, 150, 150,  33], ...
             [  54,  73,  73,  54,  54;-100,-100,  17,  17,-100], ...
             [  54,  73,  73,  54,  54;  33,  33, 150, 150,  33], ...
             [  81, 200, 200,  81,  81;-100,-100,  17,  17,-100], ...
             [  81, 200, 200,  81,  81;  33,  33, 150, 150,  33]};
    
hold on
for i = 1:length(obstacles)
    obstacle = obstacles{i};
    k = patch(obstacle(1,:),obstacle(2,:),'k');
    alpha(k,0.5)
%     waitforbuttonpress
end
hold off

% Entrances and Exits
% (  0,17);(  0,33); ( 19,17);( 19,33); (-50, 17);(-50, 33);
% (100,17);(100,33); ( 81,17);( 81,33); (150, 17);(150, 33);
% ( 19, 0);( 27, 0); ( 19,18);( 27,18); ( 19,-50);( 27,-50);
% ( 19,50);( 27,50); ( 19,32);( 27,32); ( 19,100);( 27,100);
% ( 46, 0);( 54, 0); ( 46,18);( 54,18); ( 46,-50);( 54,-50);
% ( 46,50);( 54,50); ( 46,32);( 54,32); ( 46,100);( 54,100);
% ( 73, 0);( 81, 0); ( 73,18);( 81,18); ( 73,-50);( 81,-50);
% ( 73,50);( 81,50); ( 73,32);( 81,32); ( 73,100);( 81,100);
entrances = {[  0,  0; 17, 33], ...
             [100,100; 17, 33], ...
             [ 19, 27;  0,  0], ...
             [ 19, 27; 50, 50], ...
             [ 46, 54;  0,  0], ...
             [ 46, 54; 50, 50], ...
             [ 73, 81;  0,  0], ...
             [ 73, 81; 50, 50]};
         
% hold on
% for i = 1:length(entrances)
%     entrance = entrances{i};
%     plot(entrance(1,:),entrance(2,:),'-g','LineWidth',2)
% %     waitforbuttonpress
% end
% hold off
%          
% first_exits = {[ 19, 19; 17, 33], ...
%                [ 81, 81; 17, 33], ...
%                [ 19, 27; 18, 18], ...
%                [ 19, 27; 32, 32], ...
%                [ 46, 54; 18, 18], ...
%                [ 46, 54; 32, 32], ...
%                [ 73, 81; 18, 18], ...
%                [ 73, 81; 32, 32]};
%            
% hold on
% for i = 1:length(first_exits)
%     first_exit = first_exits{i};
%     plot(first_exit(1,:),first_exit(2,:),'-y','LineWidth',2)
% %     waitforbuttonpress
% end
% hold off
%            
% second_exits = {[-50,-50; 17, 33], ...
%                 [150,150; 17, 33], ...
%                 [ 19, 27;-50,-50], ...
%                 [ 19, 27;100,100], ...
%                 [ 46, 54;-50,-50], ...
%                 [ 46, 54;100,100], ...
%                 [ 73, 81;-50,-50], ...
%                 [ 73, 81;100,100]};
%             
% hold on
% for i = 1:length(second_exits)
%     second_exit = second_exits{i};
%     plot(second_exit(1,:),second_exit(2,:),'-r','LineWidth',2)
% %     waitforbuttonpress
% end
% hold off

                

% Cameras
% 17,17,45;
% 27,33,225;
% 27,17,90;
x0 =    [ 27, 17, 38, 21, 27, 32, 38, 67, 50, 50, 57, 57, 73, 73, 81, 81];
y0 =    [ 33, 17, 17, 17, 33, 17, 33, 17, 17, 33, 17, 33, 17, 33, 17, 33];
theta = [225, 45,140, 20,-20, 20,-20,160, 20,-20, 20,-20, 20,-20, 20,-20].*pi/180;
% x0 =    [ 21, 38];
% y0 =    [ 17, 17];
% theta = [ 20,140].*pi/180;
rot = @(theta) [cos(theta),-sin(theta);sin(theta),cos(theta)];
% Camera Polygon
v0 = [9;1];
v1 = [17.5;5.5];
v2 = [17.5;-5.5];
v3 = [9;-1];
poly = [v0,v1,v2,v3,v0];
% Camera Triangle
v0_t = [0;0];
v1_t = [17.5;5.5];
v2_t = [17.5;-5.5];
v1_t2 = [4;5.5/17.5*4];
v2_t2 = [4;-5.5/17.5*4];
tri = [v0_t,v1_t,v2_t,v0_t];
tri2 = [v0_t,v1_t2,v2_t2,v0_t];
% Camera Number Location
cam_num_loc = [-2;0];

% Plot
hold on
colors = 'crggbymbr';
active_cams = [1 2 3 5 6 7 8 9];
for i = active_cams%1:length(x0)
    % Rotate Poly by Theta
    tri_i = rot(theta(i))*tri + repmat([x0(i);y0(i)],1,size(tri,2));
%     plot(tri_i(1,:),tri_i(2,:),sprintf('-%c',colors(i)),'LineWidth',1)
    k = patch(tri_i(1,:),tri_i(2,:),colors(i));
    alpha(k,0.5)
    
    tri_i2 = rot(theta(i))*tri2 + repmat([x0(i);y0(i)],1,size(tri2,2));
    patch(tri_i2(1,:),tri_i2(2,:),'k');
    
    scatter(x0(i),y0(i),20,'k','filled');
    
    cam_num_loc_i = rot(theta(i))*cam_num_loc + ...
                    repmat([x0(i);y0(i)],1,size(cam_num_loc,2));
	text(cam_num_loc_i(1), ...
         cam_num_loc_i(2), ...
         sprintf('%d',i), ...
         'FontSize',16, ...
         'HorizontalAlignment','center', ...
         'VerticalAlignment','middle', ...
         'Color',colors(i));
%    poly_i = rot(theta(i))*poly + repmat([x0(i);y0(i)],1,size(poly,2));
%    plot(poly_i(1,:),poly_i(2,:),'-m')
%     waitforbuttonpress
    
end
hold off