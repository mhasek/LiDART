function theta = getTheta(path)
% given a set of waypoints calculated a yaw angle profile
% path: Mx2 array of waypoints
% theta: Mx1 array of yaw angle profile that corresponds to the waypoints
    
    M = length(path);
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

end
