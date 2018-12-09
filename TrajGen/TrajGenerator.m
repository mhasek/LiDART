function [path] = TrajGenerator(path)

M = size(path,1);

theta = zeros(M,1);
l = zeros(M,2);

l1 = path(2,:) - path(1,:);
theta(1) = atan2(l1(2),l1(1));
l(1,:) = l1;

for i=2:M
    if i == M
        l2 = path(1,:) - path(end,:);
    else
        l2 = path(i+1,:) - path(i,:);
    end
    lines = [l1;l2];
    norms = vecnorm(lines');
    [~,idx] = min(norms);
    l(i,:) = lines(idx,:);
    
    theta(i) = atan2(l1(2) + l2(2), l1(1) + l2(1));
    
    l1 = l2;
    
end
 
% l = l./vecnorm(l')';
% figure(1)
% plot(path(:,1),path(:,2),'--','MarkerSize',5);
% hold on
% quiver(path(:,1),path(:,2),cos(theta),sin(theta),0);
% axis('equal');
% hold off
% 
% coefAmat = @(t1,t2) [1 t1 t1^2 t1^3 t1^4 t1^5;...
%                     1 t2 t2^2 t2^3 t2^4 t2^5;...
%                     0 1 2*t1 3*t1^2 4*t1^3 5*t1^4;...
%                     0 1 2*t2 3*t2^2 4*t2^3 5*t2^4;...
%                     0 0 2 6*t1 12*t1^2 20*t1^3;...
%                     0 0 2 6*t2 12*t2^2 20*t2^3];
% 
% posf = @(t,c) c(1) + c(2)*t + c(3)*t^2 + c(4)*t^3 +c(5)*t^4 + c(6)*t^5;
% velf = @(t,c) c(2) + c(3)*2*t + c(4)*3*t^2 + c(5)*4*t^3 + c(6)*5*t^4;
% accf = @(t,c) c(3)*2 + c(4)*6*t + c(5)*12*t^2 + c(6)*20*t^3;
% 
% paraR = zeros(6,M-1);
% paraT = zeros(6,M-1);
% 
% for i=1:M-1      
%     if i>1 && i<M-1
%         Bcond = [path0(i,:);path0(i+1,:); Bcond(4,:); ;zeros(2,2)];
%     elseif i >= M-1
%         Bcond = [path0(i,:);path0(M,:) ;Bcond(4,:) ;zeros(3,2)];
%     elseif i == 1
%         Bcond = [path0(i,:);path0(i+1,:) ;zeros(4,2)];
%     end
% 
%     A = coefAmat(t(i),t(i+1));
%     paraR(:,i) = A\Bcond(:,1);
%     paraT(:,i) = A\Bcond(:,2);
% end

end

