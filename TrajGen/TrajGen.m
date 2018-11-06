im = imread('levine.pgm');

points = [8.869,0.759; 8.896,7.768; -12.810, 7.752; -12.860,0.746];

im_points = [1203,1009; 1204,868;768,868; 767,1011];

l1 = points(1,:) - points(2,:);
l1_im = im_points(1,:) - im_points(2,:);

l2 =  points(2,:) - points(3,:);
l2_im =  im_points(2,:) - im_points(3,:);

raty = (l1_im./l1);
resy = raty(:,2);

ratx = (l2_im./l2);
resx = ratx(:,1);

res = [resx,resy];

im_center = [1024,1024];


W2IM = @(p) p.*res + im_center;
IM2W = @(p) (p - im_center)./res;

offset_x = [1, 0];
offset_y = [0, 1];

np = [points(1,:) - offset_y;  points(1,:) + offset_x; ...
     points(2,:) + offset_x; points(2,:) + offset_y; ...
     points(3,:) + offset_y; points(3,:) - offset_x; ...
     points(4,:) - offset_x; points(4,:) - offset_y];
 
np_im = W2IM(np);

isarc = [1,0,1,0,1,0,1,0];
angs = [-pi/2,0; 0,0; 0,pi/2; pi/2,pi/2; pi/2,pi; pi,pi; pi,3*pi/2; 3*pi/2,2*pi];
idx = repmat(1:4,[2 1]); idx = idx(:);
fp = [];

rmid = 1;
cnt = 1;
N = 100;
dx = 0.05;
dy = 0.05;
for i=1:size(np,1)
   
   c_theta = angs(i,:);
   
   if isarc(i) == 1
       rx = offset_x(1);
       ry = offset_y(2);

       rth =  ( c_theta(2) - c_theta(1) )/(N);
       d = (c_theta(2) - c_theta(1)) / 2;
       th1 = c_theta(1) : rth : (c_theta(1) + c_theta(2))/2;
       th2 = (c_theta(1) + c_theta(2))/2 : rth : c_theta(2);

       alpha = (th1-c_theta(1))/d;
       beta = 1- ( (c_theta(2) - th2)/d );
       th = [th1 th2(2:end)]';
       
       r = [rx + (rmid-rx)*alpha, rmid + (ry - rmid)*beta(2:end)]';

       pts = [cos(th).*r sin(th).*r] + points(idx(i),:) ;
       l = size(pts,1);
       pts = [pts wrapToPi((th+pi))];
   else
       i1 = i;
       i2 = (i+1);
       if i2 > size(np,1)
           i2 = 1;
       end
       s = sign(np(i2,:) - np(i1,:)); 
       pts_x = np(i1,1) : s(1)*dx : np(i2,1);
       pts_y = np(i1,2) : s(2)*dy : np(i2,2);
       
       nx = length(pts_x);
       ny = length(pts_y);
       
       if nx > ny
           pts_y = (np(i1,2) + eps): (np(i2,2)-np(i1,2))/(nx-1) :(np(i2,2) + 2*eps);
           pts_y = pts_y(1:nx);
           n = nx;
       else
           pts_x = (np(i1,1) + eps): (np(i2,1)-np(i1,1))/(ny-1) :(np(i2,1) + 2*eps);
           pts_x = pts_x(1:ny);
           n = ny;
       end
       
       
       pts = [pts_x' pts_y' wrapToPi((ones(n,1)*c_theta(1) + pi))];
   
   end
   fp = [fp; pts];
   

end
fp_im = W2IM(fp(:,1:2));

imagesc(im)
hold on
plot(np_im(:,1),np_im(:,2),'r*')
plot(fp_im(:,1),fp_im(:,2),'kx')
hold off

csvwrite("gen_waypts.csv",fp);


