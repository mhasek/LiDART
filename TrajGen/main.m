load('path.mat');
im = imread("levine.pgm");

im_center = size(im)/2;
IM2W = @(p) (p*20 - im_center).*[-0.05 0.05];

res = [0.05 0.05];
d = 0.4; % set 40cm buffer zone to wall
maxIter = 0; % dont optimize for time do exhaustive search
InterpDist = 2; %set path interpolation to 0.5 meters

% map = PreProcessMapImage(im,240);
map = im;
PrunedPath = PruneWayPoints(map,res,path,d,maxIter);
LinInterpPath = InterpPath(PrunedPath,InterpDist);


points = LinInterpPath;
t = 0.25*[0 cumsum(vecnorm(diff(points)'))];
x = points(:,1);
y = points(:,2);

tq = 0:0.01:t(end);
slope0 = 0;
slopeF = 0;
xq = spline(t,[slope0; x; slopeF],tq);
yq = spline(t,[slope0; y; slopeF],tq);
yaw = getTheta([xq' yq']);
SplinedPath = [xq' yq' yaw];

[N,M] = size(im);
imagesc([0 M*res(1)],[0 N*res(2)],im)
hold on
plot(path(:,2),path(:,1))
plot(PrunedPath(:,2),PrunedPath(:,1),'o')
plot(LinInterpPath(:,2),LinInterpPath(:,1),'*');
plot(yq,xq);
quiver(yq',xq',0.05*sin(yaw),0.05*cos(yaw),0);
axis('equal');
hold off

fp = IM2W([xq' xq']); 
fp = [fp(:,2) fp(:,1) yaw];



