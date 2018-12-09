function [sparsePath] = PruneWayPoints(map,res,path,d,maxIter)
% all points are in image axis i.e.(row column)
% map: boolean occupany map image, 255: free 0: obstacle
% res: 1x2 array repersenting resolution of map in m, i.e. 0.05: 5 cm per pixel
% path Mx2 array waypoints produced by path planner: 
% d: maximum distance to obstacle in meters
% sparsePath Nx2 pruned waypoints
if maxIter == 0
    maxIter = length(path);
end

visualize = true;

[wall_x, wall_y] = find(map == 0);
wall_x = res(:,1)*wall_x;
wall_y = res(:,2)*wall_y;

id = 2;
sparsePath = path(1,:);
lastpts = path(1,:);

while id < length(path)
    count = 0;
    for j = id:length(path)
        x = [lastpts(2) path(j,2)]*20;
        y = [lastpts(1) path(j,1)]*20;
        vals = improfile(map,x,y);
        dist = dist2line([wall_x,wall_y],lastpts,path(j,:),visualize);
        
        cutswall = ~(sum(vals == 255) == length(vals));
        drivable = ~cutswall && (dist > d);
        
        if drivable
            id = j;
        elseif cutswall 
            break
        else
            count = count + 1;
        end
        
        if (count > maxIter)
            break
        end
    end
    lastpts = path(id,:);
    id = id + 1;
    sparsePath = [sparsePath; lastpts];

    
end
end

function d = dist2line(pt, v1, v2, visualize)
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
      
      if (visualize)
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

function points = rayTrace(im,x,y)
    xq = max(x)-min;
    yq = min(y):max(y);
end