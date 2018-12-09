function [map] = PreProcessMapImage(im,thresh)
% process grayscale map image
% threshold grayscale image such that 0: occupied 255: free space
% anyspace outside closed loop track is considered occupied
% im: MxN grayscale map
% map: processed black and white map

im = im>thresh;
[bw,L] = bwboundaries(im);
q = regionprops(L);

[~,id] = max([q.Area]);
map = (L==id)*255;
 
[wall_x, wall_y] = find(im == 0);

BB = q(id).BoundingBox + [-5 -5 10 10];
bool = (wall_x >= BB(1)) & (wall_x <= (BB(1) + BB(3)))...
    & (wall_y >= BB(2)) & (wall_y <= (BB(2) + BB(4)));


wall_x = wall_x(bool);
wall_y = wall_y(bool);


map(wall_x(~bool),wall_y(~bool)) = 10; %not considered

end

