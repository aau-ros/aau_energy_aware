%% v4
%  limited sensor range
%  consider only frontiers that have been detected by sensor
%  mark all frontiers as visited that are within sensor range
%  turn measure more accurate
%  new config option for dfs: number of leaves
%  new config option for dfs: perleaf, percentage to turn around for one leaf
%% v3
%  select only frontiers where the energy is sufficient
%  fix criterion: revisit distance, leads to flower movement / depth first search
%% v2
%  robot has limited energy
%  new criterion: distance goal base, leads to spiral movement / breadth first search
%% v1
%  frontiers randomly distributed on a grid of size 25x25
%  robot starts from top left corner and needs no recharging
%  all frontiers are known in advance and all of them must be visited
%  energy is being measured in terms of traveled distance
%%
function [area] = sim_v4(w1, w2, w3, w4)

    %% configuration
    
    % dimensions (area of size dim x dim)
    dim = 600;

    % distance the robot can travel with full battery
    dmax = 488;
    
    % sensor range
    smax = 30;
    
    % weights
%     w1 = 12; %0.4779;  % distance robot goal
%     w2 = 11; %0.0327;  % revisit distance, dfs
%     w3 = 22; %11.4737; % turn measure
%     w4 = 0; %0.0661;  % distance goal base, bfs
    
    % reserve distance
    dr = 0;

    % bias for turning one direction
    thetabias = 0; % > 0: prefer ccw (rad)
    
    % the pattern for dfs
    leaves = 1; % 1, 2, 4
    
    % percentage of frontiers to visit befor aborting simulation
    front_do = 1;%0.067;
    
    % percentage of energy at which robot turns around and heads home
    % only for one leaf pattern
    perleaf = .5;
    
    %% initialization
    
    % uniformly random distributed frontiers
    %frontiers = randi([0,1],dim);
    frontiers = ones(dim);
    explored = zeros(dim);
    front_tot = sum(sum(frontiers));
    num_front = front_tot;

    % robot position
    pstart = [ceil(dim/2) ceil(dim/2)]; % start in center
    p = pstart;
    stheta = 0;
    
    % record how far the robot traveled overall
    dt = 0;
    
    % distance how far the robot can still travel
    dl = dmax;
    
    % record how much the robot turned overall
    tottheta = 0;
    
    % trajectory number (number of recharges)
    traj = 1;

    % mark visible frontiers as visited
    frontiers(p(1)-smax:p(1)+smax, p(2)-smax:p(2)+smax) = 0;
    explored(p(1)-smax:p(1)+smax, p(2)-smax:p(2)+smax) = 0;

    % copy new frontiers
    explored(p(1)-smax-1, p(2)-smax-1:p(2)+smax+1) = frontiers(p(1)-smax-1, p(2)-smax-1:p(2)+smax+1);
    explored(p(1)+smax+1, p(2)-smax-1:p(2)+smax+1) = frontiers(p(1)+smax+1, p(2)-smax-1:p(2)+smax+1);
    explored(p(1)-smax-1:p(1)+smax+1, p(2)-smax-1) = frontiers(p(1)-smax-1:p(1)+smax+1, p(2)-smax-1);
    explored(p(1)-smax-1:p(1)+smax+1, p(2)+smax+1) = frontiers(p(1)-smax-1:p(1)+smax+1, p(2)+smax+1);
    
    % initial exploration status
    area(1) = sum(sum(frontiers));

    % init goal
    g = pstart;
    
    % progress
    prog = 0;
    countdown = 2;
    
    %% computation

    while true
        % terminate if nothing changes anymore
        % (remaining frontiers out of reach)
        if num_front == sum(sum(frontiers))
            countdown = countdown - 1;
        else
            countdown = 2;
        end
        if countdown < 1
            break
        end
        
        % termination when required amount of frontiers have been visited
        num_front = sum(sum(frontiers));
        if num_front < front_tot*(1-front_do)
            break
        end
        
        % output progress
        front_done = front_tot-num_front;
        if prog < floor(front_done/front_do/front_tot*100)
            prog = floor(front_done/front_do/front_tot*100)
        end

        % distance to base
        db = sqrt((p(1)-pstart(1))^2+(p(2)-pstart(2))^2);
        
        % check energy
        if dl <= db + dr % critical, drive home immediately
            g = pstart;
            
        else % still enough energy, select next frontier
            
            % successor function
            f = inf;

            % select next goal
            for i=1:dim
                for j=1:dim
                    % not a frontier
                    if explored(i,j) < 1
                        continue;
                    end

                    % distance robot goal (w1)
                    dg = sqrt((i-p(1))^2+(j-p(2))^2);
                    
                    % distance goal base (w4)
                    dgb = sqrt((i-pstart(1))^2+(j-pstart(2))^2);
                    
                    % not enough energy for frontier
                    if dl <= dg + dgb + dr
                        continue
                    end

                    % revisit distance (w2)
                    % should be transformed to units of energy
                    if (leaves == 4 && dl/dmax > 11/14) || (leaves == 2 && dl/dmax > 7/10) || (leaves == 1 && dl/dmax > perleaf) % leaf 1
                       dir = 1;
                    elseif (leaves == 4 && dl/dmax > 10/14) || (leaves == 2 && dl/dmax > 5/10)
                       dir = 0;
                    elseif (leaves == 4 && dl/dmax > 9/14) || (leaves == 2 && dl/dmax > 3/10) % leaf 2
                        dir = 1;
                    elseif leaves == 4 && dl/dmax > 7/14
                       dir = 0;
                    elseif leaves == 4 && dl/dmax > 5/14 % leaf 3
                       dir = 1;
                    elseif leaves == 4 && dl/dmax > 4/14
                       dir = 0;
                    elseif leaves == 4 && dl/dmax > 3/14 % leaf 4
                       dir = 1;
                    else
                         dir = 0;
                    end
                    
                    if dir == 1 % drive away from base
                        drv = dim - dgb; % max - dgb, needs fix!
                    else % drive towards base
                        drv = dgb;
                    end

                    % turn measure (w3)
                    gtheta = atan2(p(1)-i,p(2)-j);
                    theta = (pi-abs(abs(stheta - gtheta + thetabias)-pi))/pi;

                    % calculate utility
                    f_new = w1*dg + w2*drv + w3*theta + w4*dgb;

                    % better frontier found
                    if f_new < f
                        f = f_new;
                        g = [i j];
                        theta2 = (pi-abs(abs(stheta - gtheta)-pi))/pi;
                    end
                end
            end
        end
        
        %output
        
        % no reachable frontier found, go charging
        if g(1) == p(1) && g(2) == p(2)
            g = pstart;
        end
        
        % move robot to new goal
        plast = p;
        p = g;

        % remove frontiers
        i = max(1,p(1)-smax):min(dim,p(1)+smax);
        j = max(1,p(2)-smax):min(dim,p(2)+smax);
        frontiers(i,j) = 0;
        explored(i,j) = 0;
        
        % copy new frontiers
        i = max(1,p(1)-smax-1);
        j = max(1,p(2)-smax-1):min(dim,p(2)+smax+1);
        explored(i,j) = frontiers(i,j);
        i = min(dim,p(1)+smax+1);
        explored(i,j) = frontiers(i,j);
        i = max(1,p(1)-smax-1):min(dim,p(1)+smax+1);
        j = max(1,p(2)-smax-1);
        explored(i,j) = frontiers(i,j);
        j = min(dim,p(2)+smax+1);
        explored(i,j) = frontiers(i,j);

        % distance traveled
        dt = dt + sqrt((plast(1)-p(1))^2 + (plast(2)-p(2))^2);
        dl = dl - sqrt((plast(1)-p(1))^2 + (plast(2)-p(2))^2);
        
        area(round(dt)) = sum(sum(frontiers));
        
        % amount turned
        tottheta = tottheta + theta2;
        
        % out of energy
        if dl <= 0
            break
        end
        
        % recharging
        if isequal(p,pstart)
            dl = dmax;
            traj = traj + 1;
        end
        
        % new orientation of robot
        stheta = atan2(plast(1)-p(1),plast(2)-p(2));
    end