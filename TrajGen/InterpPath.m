function [pathInterp] = InterpPath(path,d)
% given path interpolate path such that each waypoint distance is d
% meters
% path: Nx2 array of waypoints (x,y)
% d: interpolation distance
% pathInterp: Mx2 array of interpolated path (x,y)
pathInterp = [];
p0 = path(1,:);
for i = 2:length(path)
    p1 = path(i,:);
    dist = norm(p1-p0);
    pointCount = dist/d;
    unitvec = (p1-p0)/pointCount;
    
    x = p0(1):unitvec(1): (p1(1) - unitvec(1));
    y = p0(2):unitvec(2): (p1(2) - unitvec(2));
    tempPath = [x' y'];
    pathInterp = [pathInterp; tempPath];
    p0 = p1;
end

pathInterp = [pathInterp; path(end,:)];

end



