%% v7
%  combine sim_vx and sim_vx_visual, add gui parameter
%  calculate exploration progress (percent) more accurately
%  fix that robot does not run out of power anymore
%  output exploration progress (i.e. area/dt) to a file
%  change search radius for frontiers more smoothly
%  only compute parameters when weights are non-zero
%% v6
%  new parameter: prefer frontiers close to obstacles
%  allow non-square maps
%  support rgb images (they are converted to grayscale)
%  store distances from points in map to base
%  create functions to reduce code redundancy
%  planning through unknown can be switched on and off
%% v5
%  introducing obstacles
%  paths are no longer straight lines, using A* to find paths
%  exploration along paths (i.e. evaluating sensor range at each point of the path)
%  grouping of frontier cells
%  sensing and traveling only along Von Neumann neighborhood (4-cell)
%  new performance measure: redundant traveling dtr (and dte)
%  live progress in figure
%  make debug output nicer
%  removed parameter dim, automatically calculate it
%  removed paramter leaves, now only considering dfs with one leaf
%  new performance measure: exploration progress
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
function [traj,area_done,dt,dte,dtr] = sim_v7(gui, w1, w2, w3, w4, w5)

    %% configuration

    % distance the robot can travel with full battery
    dmax = 2000;
    
    % sensor range
	% klagenfurt (400x341): 10 = 30m
	% manhattan (195x195): 20 = 30m
    smax = 30;
    
    % percentage of space to visit before aborting simulation
    area_do = 1;
    
    % percentage of space that the robot could theoretically visit
    % this is the ratio of the reachable pixels of the map to the
    % total number of pixels
    % needs to be updated for each map
	% klagenfurt (400x341): 0.13282991
	% manhattan (195x195): 0.2464957265
	% manhattan (597x597): 0.2672996473
    % manhattan (997x997): 0.07828299341
    % manhattan (1194x1194): 0.06706690908
    % hospital (1500x572): 0.06712937063
    % vienna (1000x510): 0.03910196078
    area_reachable = 0.03910196078;
    
    % reserve distance
    dr = 0;
    
    % plan paths through unexplored space
    % 0: plan paths only through explored space
    % 1: plan paths through unexplored space
    unknown = 0;
    
    % search radius for frontiers
    % initially only frontiers within range fmax are considered as goal
    % this value is incremented if no frontiers are left in this area
    fmax = smax;
    
    % the search strategy
    % can be: euclidean, astar (default euclidean)
    strategy = 'astar';
    
    % weights
%     w1 = 10; % distance robot goal
%     w2 = 9; % distance goal base, dfs
%     w3 = 15; % turn measure
%     w4 = 0; % distance goal base, bfs
%     w5 = 10; % distance of goal to next obstacle

    % bias for turning one direction
    thetabias = 0; % > 0: prefer ccw (rad)
    
    % percentage of energy at which robot turns around and heads home
    perleaf = .5;
    
    % the type of map that is used
    % 0: random map
    % 1: random map (the same each run)
    % 2: map from file
    %    filename must be map.jpg
    %    should not be too big
    map_type = 2;
    
    % percentage of grid being ocupied
    % only valid for map_type 0 and 1
    wallpercent = 0;
    
    % starting position of the robot [px-from-top px-from-left]
    % set to [-1 -1] to start in center
	% klagenfurt (400x341): [140 165]
    % hospital (1500x572): [550 700]
    % vienna (1000x510): [275 510]
    pstart = [275 510];
	
	% file output exploration status
	% 0: no output to file
	% 1: output explored area as function of traveled distance
	% file states the filename used
	output = 1;
    file = strcat('sim_v7_progress_',num2str(w1),'_',num2str(w2),'_',num2str(w3),'_',num2str(w4),'_',num2str(w5),'.txt');
    
    % debug output level
    % 0: no output to console
    % 1: display percentage
    % 2: display also text messages
    % 3: display also numeric values
    debug = 1;
    
    %% initialization
    
    % seed random map generation
    if map_type == 1
        rng(1)
    end
    % create random map
    if map_type == 0 || map_type == 1
        % dimensions (area of size dim1 x dim2)
        dim1 = dmax + 2*smax + 2; % +2 just to be sure not to cut off anything
        dim2 = dim1;
        % map
        map = ones(dim1,dim2) + 10*rand(dim1,dim2);
        map(ind2sub([dim1 dim2],ceil(dim1*dim2.*rand(floor(dim1*dim2*wallpercent),1)))) = Inf;
        map(map==inf) = 1;
        map(map~=1) = 0;
    % create map from file map.jpg
    elseif map_type == 2
        % map
        map = 1-round(rgb2gray(imread('map.jpg'))/255);
        % dimensions (area of size dim1 x dim2)
        dim1 = size(map,1);
        dim2 = size(map,2);
    end
    
    % array for explored map
    explored = zeros(dim1,dim2) + 0.5;
	area_total = dim1*dim2;
	area_left = area_total;
    area_max = pi*(dmax/2)^2*area_reachable; % approximation

    % robot position
    if pstart(1) == -1 && pstart(2) == -1
        pstart = [ceil(dim1/2) ceil(dim2/2)]; % start in center
    end
    p = pstart;
    stheta = 0;
    
    % keep track of the current frontiers
    frontiers = zeros(dim1,dim2);
    
    % store distances
    distances = zeros(dim1,dim2);
    
    % no obstacles at start position
    map(pstart(1),pstart(2)) = 0;
    
    % record how far the robot traveled
    dt  = 0; % overall
    dte = 0; % while exploring
    dtr = 0; % while not exploring
    dtm = floor(pi*dmax / (2*smax) * dmax); % maximum, approximation
    
    % distance how far the robot can still travel
    dl = dmax;
    
    % record how much the robot turned overall
    %tottheta = 0;
    
    % trajectory number (number of recharges)
    traj = 1;

    % copy initially explored map data
    explored = update_explored(map, explored, frontiers, p, smax);
    
    % find frontiers
    [frontiers, distances] = update_frontiers(explored, frontiers, distances, pstart, strategy, dr, smax);
    
    % initial exploration status
    area = zeros(dtm,1);
    area(1) = area_total - sum(sum(explored == 0.5));
        
    % write exploration status to file
	if output
		fileID = fopen(file,'a');
		fprintf(fileID, 'dt\tarea\n');
		fprintf(fileID, '0\t%d\n', area(1));
		fclose(fileID);
	end

    % init goal
    g = pstart;
    
    % search radius for frontiers
    fmax_var = fmax;
    
    % progress
    progress = 0;
    countdown = 2;
    
    %% computation
    
    % plot map
    if gui
        fig = figure;
        subplot(1,2,1)
        imagesc(1-map)
        colormap(gray)
        axis equal tight
        hold on
        plot(pstart(2), pstart(1), '.g', 'MarkerSize',20)
    end
    
    while true
        % terminate if nothing changes anymore
        % (remaining frontiers out of reach)
        if area_left == sum(sum(explored == 0.5))
            if fmax_var < dmax/2
                fmax_var = fmax_var + fmax;
                if debug > 1
                    fprintf('search radius: %d\n', fmax_var)
                end
            elseif isequal(p, pstart)
                countdown = countdown - 1;
            end
        elseif fmax_var > fmax || countdown < 2
            countdown = 2;
            fmax_var = fmax_var - fmax;
            if debug > 1
                fprintf('search radius: %d\n', fmax_var)
            end
        end
        if countdown < 1
            if debug > 1
                fprintf('nothing left to explore within radius %d\n', fmax_var)
            end
            break
        end
        
        % termination when required amount of frontiers have been visited
        area_left = sum(sum(explored == 0.5));
        area_done = area_total-area_left;
        if area_done > area_max*area_do
            break
        end

        % output progress
        if debug > 0
            if progress < floor(area_done/area_do/area_max*100)
                progress = floor(area_done/area_do/area_max*100);
                fprintf('%d%%\n', progress)
            end
        end

        % distance to base
        if distances(p) > 0 % check if path length is stored already
            db = distances(p(1),p(2));
            path_base = 0;
        elseif strcmp(strategy,'astar')
            if unknown > 0
                path_base = astar(floor(explored), p, pstart); % make unexplored look free for astar
            else
                path_base = astar(ceil(explored), p, pstart);  % make unexplored look occupied
            end
            db = size(path_base,1) - 1;
            distances(p) = db; % store path length
        else % default is euclidean
            db = sqrt((p(1)-pstart(1))^2+(p(2)-pstart(2))^2);
            distances(p) = db; % store path length
        end
        % no path to base possible
        if db < 0
            if debug > 1
                fprintf('there is no way back home\n')
            end
            break;
        end
        
        % furthest distance of a frontier to the base
        if w2 ~= 0
            dfb_max = max(max(frontiers));
        end
                
        % check energy
        if dl <= db + dr % critical, drive home immediately
            g = pstart;
            if strcmp(strategy,'astar')
                if path_base == 0
                    if unknown > 0
                        path_base = astar(floor(explored), p, pstart); % make unexplored look free for astar
                    else
                        path_base = astar(ceil(explored), p, pstart);  % make unexplored look occupied
                    end
                end
                path_goal = path_base;
            end
            dg = db;
            if debug > 1
                fprintf('critical energy level: dl=%d, db=%d\n', dl, db)
            end
            
        else % still enough energy, select next frontier
            
            % successor function
            f = inf;

            % select next goal
            for i = max(1,p(1)-fmax_var) : min(dim1,p(1)+fmax_var)
                for j = max(1,p(2)-fmax_var) : min(dim2,p(2)+fmax_var)
                    if frontiers(i,j) <= 0
                        continue
                    end

                    % distance to frontier (w1)
                    if strcmp(strategy,'astar')
                        if unknown > 0
                            path_frontier = astar(floor(explored), p, [i j]); % make unexplored look free for astar
                        else
                            path_frontier = astar(ceil(explored), p, [i j]);  % make unexplored look occupied
                        end
                        df = size(path_frontier,1) - 1;
                    else % default is euclidean
                        df = sqrt((i-p(1))^2+(j-p(2))^2);
                    end
        
                    % frontier not reachable
                    if df < 0
                        continue
                    end
                    
                    % not enough energy to reach frontier
                    if dl <= df + dr
                        continue
                    end
                    
                    % distance frontier to base (w4)
                    dfb = frontiers(i,j);
        
                    % frontier not reachable
                    if dfb < 0
                        continue
                    end
                    
                    % not enough energy to reach frontier
                    if dl <= df + dfb + dr
                        continue
                    end

                    % revisit distance (w2)
                    % should be transformed to units of energy
                    if (dl-dr)/dmax > perleaf
                        dir = 1;
                    else
                        dir = 0;
                    end
                    
                    if w2 == 0
                        drv = 0;
                    elseif dir == 1 % drive away from base
                        drv = dfb_max - dfb; % max - dgb, needs fix!
                    else % drive towards base
                        drv = dfb;
                    end

                    % turn measure (w3) - not accurate, needs fix?
                    if w3 == 0
                        theta = 0;
                    else
                        gtheta = atan2(p(1)-i,p(2)-j);
                        theta = (pi-abs(abs(stheta - gtheta + thetabias)-pi))/pi;
                    end
                    
                    % distance of goal to next obstacle (w5)
                    if w5 == 0
                        dfo = 0;
                    else
                        dfo = distance_obstacle(explored, [i j], smax);
                    end

                    % calculate utility
                    f_new = w1*df + w2*drv + w3*theta + w4*dfb + w5*dfo;

                    % better frontier found
                    if f_new < f
                        f = f_new;
                        g = [i j];
                        if strcmp(strategy,'astar')
                            path_goal = path_frontier;
                        end
                        dg = df;
                    end
                end
            end
        end
        
        if debug > 2
            fprintf('at: (%d,%d)\n', p(1), p(2))
        end
        if debug > 2
           fprintf('dt=%d, dl=%d, db=%d\n', dt, dl, db)
        end
        if debug > 2
           fprintf('goal: (%d,%d)\n', g(1), g(2))
        end
        
        % no frontier found that the robot can reach, go charging, decrease search radius
        if isequal(g,p) && (fmax_var >= dl || fmax_var >= dmax/2)
            g = pstart;
            if strcmp(strategy,'astar')
                if path_base == 0
                    if unknown > 0
                        path_base = astar(floor(explored), p, pstart); % make unexplored look free for astar
                    else
                        path_base = astar(ceil(explored), p, pstart);  % make unexplored look occupied
                    end
                end
                path_goal = path_base;
            end
            dg = db;
            if fmax_var > fmax
                fmax_var = fmax;
            end
            if debug > 1
                fprintf('no reachable frontier, go charging\n')
                fprintf('search radius: %d\n', fmax_var)
            end
        % no frontier found, increase search radius and keep looking
        elseif isequal(g,p)
            if debug > 2
                fprintf('\n')
            end
            continue
        end
        
        % store last position
        plast = p;
        
        if gui
            % set color
            if traj == 1
                color = 'k';
            elseif traj == 2
                color = 'b';
            elseif traj == 3
                color = 'g';
            elseif traj == 4
                color = 'r';
            elseif traj == 5
                color = 'c';
            elseif traj == 6
                color = 'm';
            elseif traj == 7
                color = 'y';
            else
                color = 'k';
                traj = 1;
            end
        end
        
        % open file for writing exploration status
		if output
			fileID = fopen(file,'a');
		end

        % plot trajectory and copy new map data
        if gui
            figure(fig)
            subplot(1,2,1);
        end
        if strcmp(strategy,'astar')
            k = 1;
            while k < size(path_goal,1)
                k = k+1;
                
                if dl < 0
                    break
                end
                
                % make sure path is valid
                if unknown > 0 && map(path_goal(k,1),path_goal(k,2)) > 0
                    path_goal = astar(floor(explored), p, g); % replan
                    dg = size(path_goal,1) - 1;
                    k = 1;
                    continue;
                end
                
                % move robot
                p(1) = path_goal(k,1);
                p(2) = path_goal(k,2);
                
                % distance traveled
                dt = dt + 1;
                dl = dl - 1;
                
                % keep copy of explored map to check if new area was explored
                temp = explored;
                
                % copy newly explored map data
                [explored, frontiers] = update_explored(map, explored, frontiers, p, smax);
                
                % no new area was explored
                if isequal(explored, temp)
                    if gui
                        linestyle = ':'; % make redundant paths dotted
                    end
                    dtr = dtr + 1; % measure redundancy
                    area(dt+1) = area(dt); % measure exploration status
                
                % new area was explored
                else
                    if gui
                        linestyle = '-'; % paths while exploring are solid
                    end
                    dte = dte + 1; % measure redundancy
                    area(dt+1) = area_total - sum(sum(explored == 0.5)); % measure exploration status
                end
                
                % plotting
                if gui
                    plot([path_goal(k-1,2) path_goal(k,2)],[path_goal(k-1,1) path_goal(k,1)],strcat(linestyle,color),'LineWidth',2)
                end
                
                % write exploration status to file
				if output
					fprintf(fileID, '%d\t%d\n', dt, area(dt+1));
				end
            end
        else
            % move robot to new goal
            p = g;

            % distance traveled
            dt = dt + dg;
            dl = dl - dg;

            % copy map data
            i = max(1,plast(1)-smax):min(dim1,plast(1)+smax);
            j = max(1,plast(2)-smax):min(dim2,plast(2)+smax);
            explored(i,j) = map(i,j); % copy map data
            frontiers(i,j) = 0; % remove frontiers
            i = max(1,p(1)-smax):min(dim1,p(1)+smax);
            j = max(1,p(2)-smax):min(dim2,p(2)+smax);
            explored(i,j) = map(i,j); % copy map data
            frontiers(i,j) = 0; % remove frontiers
            
            % measure exploration status
            area(floor(dt+1)) = area_total - sum(sum(explored == 0.5));
            
            % plotting
            if gui
                if ~isequal(plast,pstart) && ~isequal(p,pstart)
                    plot([plast(2) p(2)],[plast(1) p(1)],color,'LineWidth',2)
                end
            end
                
            % write exploration status to file
			if output
				fprintf(fileID, '%f\t%d\n', dt, area(floor(dt+1)));
			end
        end
        
        % close file for writing exploration status
		if output
			fclose(fileID);
		end
        
        % find new frontiers
        [frontiers, distances] = update_frontiers(explored, frontiers, distances, pstart, strategy, dr, smax);
        
        % plot current exploration status
        if gui
            figure(fig)
            subplot(1,2,2);
            imagesc(1-explored)
            colormap(gray)
            axis equal tight
            hold on
            plot(pstart(2), pstart(1), '.g', 'MarkerSize',20)
            plot(p(2), p(1), strcat('.',color), 'MarkerSize',20)

            % plot frontiers
            for i=1:dim1
                for j=1:dim2
                    if frontiers(i,j) > 0
                        plot(j, i, '.r', 'MarkerSize',10)
                    end
                end
            end
        end
        
        % out of energy
        if dl < 0
            if debug > 1
                fprintf('out of energy\n')
            end
            if gui
                figure(fig)
                subplot(1,2,1);
                plot(p(2), p(1), '.r', 'MarkerSize',20)
                subplot(1,2,2);
                plot(p(2), p(1), '.r', 'MarkerSize',20)
            end
            break
        end
        
        % recharging
        if isequal(p,pstart)
            if debug > 1
                fprintf('recharging\n')
            end
            dl = dmax;
            traj = traj + 1;
        end
        
        % new orientation of robot
        if w3 ~= 0
            stheta = atan2(plast(1)-p(1),plast(2)-p(2));
        end
        
        if debug > 2
            fprintf('\n')
        end
    end
end

%% update_frontiers
%  finds new frontiers and
%  updates the array containing the frontiers
function [frontiers, distances] = update_frontiers(explored, frontiers, distances, pstart, strategy, dr, smax)
    %% initialization
    dim1 = size(explored,1);
    dim2 = size(explored,2);
    
    %% find frontiers
    for i = 1:dim1
        for j = 1:dim2
            % not an empty cell
            if explored(i,j) ~= 0
                continue
            end

            % all neighboring cells are already explored, not a frontier
            if explored(max(1,i-1),j) ~= 0.5 && explored(i,max(1,j-1)) ~= 0.5 && explored(i,min(dim2,j+1)) ~= 0.5 && explored(min(dim1,i+1),j) ~= 0.5
                continue
            end
            
            frontiers(i,j) = -1;
        end
    end
    
    %% connect frontiers
    for i = 1:dim1
        for j = 1:dim2
            
            % not a frontier cell
            if frontiers(i,j) ~= -1
                continue
            end
            
            % vertical frontier
            if i < dim1 && frontiers(i+1,j) == -1
                
                % look up
                start = i;
                
                % look down
                k = i;
                while k < dim1 && k-i < smax && frontiers(k+1,j) == -1
                   k = k+1; 
                end
                stop = k;
                
                % combine to a single frontier
                frontiers(start:stop,j) = 0;
                k = round(start+(stop-start)/2);
                if frontiers(k,j-1:j+1) == 0
                    if distances(k,j) > 0 % check if path length is stored already
                        frontiers(k,j) = distances(k,j);
                    elseif strcmp(strategy,'astar')
                        if dr > 0
                            path_frontier_base = astar(floor(explored), [k j], pstart); % make unexplored look free for astar
                        else
                            path_frontier_base = astar(ceil(explored), [k j], pstart);  % make unexplored look occupied
                        end
                        frontiers(k,j) = size(path_frontier_base,1) - 1;
                    else % default is euclidean
                        frontiers(k,j) = sqrt((k-pstart(1))^2+(j-pstart(2))^2);
                    end
        
                    % store path length
                    distances(k,j) = frontiers(k,j); % max(distances)
                end
                
            % horizontal frontier
            elseif j<dim2 && frontiers(i,j+1) == -1
                
                % look left
                start = j;
                
                % look right
                k = j;
                while k < dim2 && k-j < smax && frontiers(i,k+1) == -1
                   k = k+1; 
                end
                stop = k;
                
                % combine to a single frontier
                frontiers(i,start:stop) = 0;
                k = round(start+(stop-start)/2);
                if frontiers(i-1:i+1,k) == 0
                    if distances(i,k) > 0 % check if path length is stored already
                        frontiers(i,k) = distances(i,k);
                    elseif strcmp(strategy,'astar')
                        if dr > 0
                            path_frontier_base = astar(floor(explored), [i k], pstart); % make unexplored look free for astar
                        else
                            path_frontier_base = astar(ceil(explored), [i k], pstart);  % make unexplored look occupied
                        end
                        frontiers(i,k) = size(path_frontier_base,1) - 1;
                    else % default is euclidean
                        frontiers(i,k) = sqrt((i-pstart(1))^2+(k-pstart(2))^2);
                    end
                    
                    % store path length
                    distances(i,k) = frontiers(i,k);
                end
                
            % isolated frontier cell
            else
                if distances(i,j) > 0 % check if path length is stored already
                    frontiers(i,j) = distances(i,j);
                elseif strcmp(strategy,'astar')
                    if dr > 0
                        path_frontier_base = astar(floor(explored), [i j], pstart); % make unexplored look free for astar
                    else
                        path_frontier_base = astar(ceil(explored), [i j], pstart);  % make unexplored look occupied
                    end
                    frontiers(i,j) = size(path_frontier_base,1) - 1;
                else % default is euclidean
                    frontiers(i,j) = sqrt((i-pstart(1))^2+(j-pstart(2))^2);
                end
        
                % store path length
                distances(i,j) = frontiers(i,j);
            end
        end
    end
end

%% update_explored
%  copies newly explored map data from map into explored
%  also clears frontiers in the newly explored area
function [explored, frontiers] = update_explored(map, explored, frontiers, p, smax)
    % initialization
    dim1 = size(map,1);
    dim2 = size(map,2);
    
    % copying
    j = max(1,p(2)-1):min(dim2,p(2)+1);
    for i = p(1):min(dim1,p(1)+smax)
        if map(i,p(2)) > 0 % copy only until obstacle
            explored(i,p(2)) = map(i,p(2));
            break
        end
        explored(i,j) = map(i,j); % copy map data
        frontiers(i,j) = 0; % remove frontiers
    end
    for i = p(1):-1:max(1,p(1)-smax)
        if map(i,p(2)) > 0 % copy only until obstacle
            explored(i,p(2)) = map(i,p(2));
            break
        end
        explored(i,j) = map(i,j); % copy map data
        frontiers(i,j) = 0; % remove frontiers
    end

    i = max(1,p(1)-1):min(dim1,p(1)+1);
    for j = p(2):min(dim2,p(2)+smax)
        if map(p(1),j) > 0 % copy only until obstacle
            explored(p(1),j) = map(p(1),j);
            break
        end
        explored(i,j) = map(i,j); % copy map data
        frontiers(i,j) = 0; % remove frontiers
    end
    for j = p(2):-1:max(1,p(2)-smax)
        if map(p(1),j) > 0 % copy only until obstacle
            explored(p(1),j) = map(p(1),j);
            break
        end
        explored(i,j) = map(i,j); % copy map data
        frontiers(i,j) = 0; % remove frontiers
    end
end

%% distance_obstacle
%  calculates the euclidean distance from a point p to the next obstacle in the currently explored map
%  the difference to smax is returned
function diff = distance_obstacle(explored, p, smax)
    % initialization
    dim1 = size(explored,1);
    dim2 = size(explored,2);
    
    % start looking in squares around current point
    d = 1;
    while d <= smax
        for i = max(1,p(1)-d) : min(dim1,p(1)+d)
            for j = max(1,p(2)-d) : min(dim2,p(2)+d)
                if explored(i,j) == 1
                    diff = smax - d;
                	return
                end
            end
        end
        d = d+1;
    end
    
    % no obstacle found, return max distance (=smax+1)
    diff = d;
end