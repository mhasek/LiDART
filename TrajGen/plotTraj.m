load('path.mat');
im = imread("levine.pgm");


% im = im>240;
% [bw,L] = bwboundaries(im);
% q = regionprops(L);
% [~,id] = max([q.Area]);
% im = (L==id)*255;
% 
% BB = q(id).BoundingBox + [-5 -5 10 10];



[N,M] = size(im);
% 
[wall_y, wall_x] = find(im == 0);
wall_y = 0.05*wall_y;
wall_x = 0.05*wall_x;


% bool = (wall_x > BB(1)) & (wall_x < (BB(1) + BB(3)))...
%     & (wall_y > BB(2)) & (wall_y < (BB(2) + BB(4)));
% 
% wall_y = wall_y(bool)*0.05;
% wall_x = wall_x(bool)*0.05;

id = 2;
newPath = path(1,:);
lastpts = path(1,:);
thresh = 0.4;

while id < length(path)
    count = 0;
    for j = id:length(path)
        x = [lastpts(2) path(j,2)]*20;
        y = [lastpts(1) path(j,1)]*20;
        vals = improfile(im,x,y);

        dist = dist2line([wall_y,wall_x],lastpts,path(j,:),thresh);
        
        cutswall = ~(sum(vals == 255) == length(vals));
        drivable = ~cutswall && (dist > thresh);
        
        if drivable
            id = j;
        elseif cutswall 
            break
        else
            count = count + 1;
        end
        
%         if (count > 1)
%             break
%         end
    end
    lastpts = path(id,:);
    id = id + 1;
    newPath = [newPath; lastpts];
    
%     x = [lastPts(2) path(i,2)]*20;
%     y = [lastPts(1) path(i,1)]*20;
%     vals = improfile(im,x,y);
%     
%     dist = dist2line([wall_y,wall_x]*0.05,lastPts,path(i,:));
%     
%     drivable = (sum(vals == 255) == length(vals)) && (dist > 0.4);
%     
%     if ~drivable
%         newPath = [newPath; path(i-1,:)];
%         dist
% %         dist2line([wall_x,wall_y]*0.05,lastPts,path(i,:))
% %         dist = sqrt(sum(([wall_x,wall_y]*0.05 - path(i,:)).^2,2));
%         lastPts = path(i-1,:);        
%     end
%     
    
    
end

% newPath = [newPath; path(end,:)];
newPathInterp = [];
p0 = newPath(1,:);
for i = 2:length(newPath)
    p1 = newPath(i,:);
    dist = norm(p1-p0);
    pointCount = dist/2;
    unitvec = (p1-p0)/pointCount;
    
    x = p0(1):unitvec(1): (p1(1) - unitvec(1));
    y = p0(2):unitvec(2): (p1(2) - unitvec(2));
    tempPath = [x' y'];
    newPathInterp = [newPathInterp; tempPath];
    p0 = p1;
end

newPathInterp = [newPathInterp; newPath(end,:)];
    
imagesc([0 M*0.05],[0 N*0.05],im)
hold on
plot(path(:,2),path(:,1))
plot(newPath(:,2),newPath(:,1),'b*')
plot(newPathInterp(:,2),newPathInterp(:,1),'*')

im_center = size(im)/2;
IM2W = @(p) (p*20 - im_center).*[-0.05 0.05];

fp = IM2W(newPathInterp);
fp = [fp(:,2) fp(:,1)];

points = newPathInterp;
t = 0.25*[0 cumsum(vecnorm(diff(points)'))];
x = points(:,1);
y = points(:,2);
tq = 0:0.01:t(end);
slope0 = 0;
slopeF = 0;
xq = spline(t,[slope0; x; slopeF],tq);
yq = spline(t,[slope0; y; slopeF],tq);
plot(yq,xq,':.');

TrajGenerator(newPathInterp);

fp = IM2W([xq' xq']); 
fp = [fp(:,2) fp(:,1)];

% 
% xlim([650,1400]*0.05)
% ylim([700 1200]*0.05)


% figure;
% plot(t,x,t,x,'o');
% title('x vs. t');
% 
% figure;
% plot(t,y,t,y,'o');
% title('y vs. t');
% 
% figure;
% plot(x,y,x,y,'o');
% title('y vs. x');
% 
% figure;
% plot(t,x,'o',tq,xq,':.');
% title('x vs. t');
% 
% figure;
% plot(t,y,'o',tq,yq,':.');
% title('y vs. t');



function d = dist2line(pt, v1, v2, thresh)
      v1 = [v1 0];
      v2 = [v2 0];
      pt = [pt zeros(length(pt),1)];
      a = v1 - v2;
      b = pt - v2;
      alpha = dot(repmat(a,[length(pt) 1])',b')./dot(a,a);
      proj = a.*repmat(alpha',[1,3]);
      bool = (vecnorm(proj') < norm(a)) & (alpha > 0);
      proj2 = proj(bool,:);      

      
      dot(repmat(a,[length(pt) 1])',b');
      d = vecnorm(cross(repmat(a,[length(pt) 1])',b')) / norm(a);
      [d,idx] = min(d(bool));
      pt_filt = pt(bool,:);
      
      if (d > thresh)
          figure(1)
          plot(v2(1) + proj2(:,1),v2(2) + proj2(:,2),'MarkerSize',2)
          hold on
          plot([v1(1),v2(1)],[v1(2),v2(2)],'r*')
          plot(pt(:,1),pt(:,2),'*','MarkerSize',2)
          plot(pt_filt(idx,1),pt_filt(idx,2),'bx')
          hold off
          pause(0.001)
      end
      if (isempty(d))
          d = 0;     
      end
          
end 
