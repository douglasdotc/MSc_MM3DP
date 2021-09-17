classdef CLS_ENV_SE2 < handle
    properties
        task_OBJ        % task
        task_T          % Transformation matrices 4x4xn
        s               % Task progress
        task_coord      % Task coordinates nx3
        robot_hitbox    % robot hit box
        IRM             % Inverse Reachability Map
        obstacles       % Obstacles
        IsDEBUG         % Debug flag
%         x_lim           % x-boundary of environment
%         y_lim           % y-boundary of environment
    end
    
    methods
        function this = CLS_ENV_SE2(task, task_T, s, robot_hitbox, IRM, obstacles, IsDEBUG)
        %% Initialize://///////////////////////////////////////////////////////////////////////////
            this.task_OBJ       = task;
            this.task_T         = task_T;
            this.s              = s;
            this.task_coord     = squeeze(this.task_T(1:2, 4, :))';
            this.robot_hitbox   = robot_hitbox;
            this.IRM            = IRM;
            this.obstacles      = obstacles;
            this.IsDEBUG        = IsDEBUG;
%             this.x_lim          = x_lim;
%             this.y_lim          = y_lim;
        end
        
        function nodes = sample_pts(this, s, n)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Sample n points from the IRM corresponds to progress s
        % Inputs:
        % s:    progress, scalar
        % n:    Number of points to sample, scalar
        % Outputs:
        % nodes: An array of nodes with size nx1
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

            % Find the transformation matrix correspond to the progress s:
            T_s     = this.task_T(:, :, min(size(this.task_T, 3), max(1, ceil(size(this.task_T, 3)*s))));
            nodes   = [];
            for idx = 1:n % sample n pts
                [pt, ~] = this.IRM.sample(T_s(1:3, 4), s);
                pt      = node_SE2(pt);

                % KD Tree
                if isempty(nodes)
                    nodes = [nodes; pt];
                else
                    [isReached, isInserted] = CLS_KDTree.insert(pt, nodes(1));
                    if isReached && isInserted
                        nodes = [nodes; pt];
                    end
                end
            end
        end
        
        function IsValid = ValidityCheck(this, node, varargin)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Check if the point pt is valid
        % 1. Is in IRM
        % 2. No collision with task
        % 3. No collision with environment
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            NoCollisionWithObstacles = this.CheckCollisionWithObstacles(node);
            NoCollisionWithTasks     = this.CheckCollisionWithTask(node);
            IsInIRM                  = this.isInIRM(node);
            IsValid                  = NoCollisionWithObstacles && NoCollisionWithTasks && IsInIRM;
        end
        
        function IsValid = CheckCollisionWithObstacles(this, node, varargin)
            if isempty(varargin)
                % Robot hit box pose:
                ro_T = [node.pose(3), -node.pose(4), node.pose(1);
                        node.pose(4),  node.pose(3), node.pose(2);
                                   0,             0,            1];

                Robot_Poly = (ro_T*this.robot_hitbox)';
            else
                Robot_Poly = varargin{1};
            end
            
            IsValid = true;
            % Check if robot collide with obstacles
            for o_idx = 1:length(this.obstacles)
                IsCollide = any(inpolygon(this.obstacles{o_idx}(:,1), this.obstacles{o_idx}(:,2), Robot_Poly(:,1), Robot_Poly(:,2)));
                Iscollide = IsCollide || any(inpolygon(Robot_Poly(:,1), Robot_Poly(:,2), this.obstacles{o_idx}(:,1), this.obstacles{o_idx}(:,2)));
                
                if Iscollide
                    if this.IsDEBUG
                        %disp("Robot collide with obstacles")
                    end
                    IsValid = false;
                    return;
                end
            end
        end
        
        function IsValid = CheckCollisionWithTask(this, node, varargin)
            if isempty(varargin)
                % Robot hit box pose:
                ro_T = [node.pose(3), -node.pose(4), node.pose(1);
                        node.pose(4),  node.pose(3), node.pose(2);
                                   0,             0,            1];

                Robot_Poly = (ro_T*this.robot_hitbox)';
            else
                Robot_Poly = varargin{1};
            end
            
            % Check if robot collide with task
            IsValid = true;
            if node.pose(5) ~= 0
                s_idx = min(length(this.task_coord), round(length(this.task_coord)*node.pose(5)));
            else
                s_idx = 1;
            end
            
            if isempty(varargin)
                que_x       = this.task_coord(1:s_idx, 1);
                que_y       = this.task_coord(1:s_idx, 2);
            else
                Forward_or_Backward = varargin{1};
                if strcmp(Forward_or_Backward, 'forward_progress_sq_norm') || strcmp(Forward_or_Backward, 'sq_norm')
                    que_x       = this.task_coord(1:s_idx, 1);
                    que_y       = this.task_coord(1:s_idx, 2);
                elseif strcmp(Forward_or_Backward, 'backward_progress_sq_norm')
                    que_x       = this.task_coord(s_idx:end, 1);
                    que_y       = this.task_coord(s_idx:end, 2);
                end
            end
            
            IsCollide   = any(inpolygon(que_x, que_y, Robot_Poly(:,1), Robot_Poly(:,2)));
            if IsCollide
                if this.IsDEBUG
                    %disp("Robot collide with task")
                end
                IsValid = false;
                return
            end
        end
        
        function IsValid = isInIRM(this, node)
            % Check if point is in IRM
            Ts_idx  = min(size(this.task_T, 3), max(1, ceil(size(this.task_T, 3)*node.pose(5))));
            T_s     = this.task_T(:,:,Ts_idx);
            IsValid = this.IRM.PtInIRM(T_s, node.pose);
        end
        
        function val = Extract_item(this, nodes, item)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Return all items stored in nodes
        % Inputs:
        % nodes:    nx1 array of nodes
        % item:     string, name of item to be extracted
        % Outputs:
        % val:      Naked array of size nxm, m is the dimension of the item
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if strcmp(item, 'pose')
                data_size = size(nodes(1).pose);
                val       = reshape(cell2mat({nodes(:).pose}'), [], data_size(2));
            elseif strcmp(item, 'cost')
                data_size = size(nodes(1).cost);
                val       = reshape(cell2mat({nodes(:).cost}'), [], data_size(2));
            elseif strcmp(item, 'parent')
                data_size = size(nodes(1).parent);
                val       = reshape(cell2mat({nodes(:).parent}'), [], data_size(2));
            end
        end
        
        function break_pts = Breakpoints_obs_intersect(this) % NOT IN USE
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Inputs:
        % N/A
        %Outputs:
        % break_pts:    s value (progress) that correspond to possible breakpoints
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
        %%% !!!!!!!!!region beyond the obstacle cannot reach, so not
        %%% available --> FIX
            % intersection for obstacles
            obs_availability  = zeros(size(this.s));
            task_availability = zeros(size(this.s));
            IRM_area          = area(this.IRM.IRM_poly);
            s_size            = length(this.s);
            for s_idx = 1:s_size
                obs_invalid_area  = 0;
                task_invalid_area = 0;
                
                % Get IRM at progress s
                T_s_coord         = this.task_coord(s_idx, :);
                
                
                % Translate IRM
                this.IRM.IRM_poly.Vertices = [this.IRM.IRM_poly.Vertices(:,1) + T_s_coord(1), this.IRM.IRM_poly.Vertices(:,2) + T_s_coord(2)];
                for o_idx = 1:length(this.obstacles)
                    inter_poly       = intersect(this.obstacles{o_idx}, this.IRM.IRM_poly);
                    obs_invalid_area = obs_invalid_area + area(inter_poly);
                end
                % Backward translate IRM
                this.IRM.IRM_poly.Vertices = [this.IRM.IRM_poly.Vertices(:,1) - T_s_coord(1), this.IRM.IRM_poly.Vertices(:,2) - T_s_coord(2)];
                
                % Task
                [in, on]                 = inpolygon(this.task_coord(:,1), this.task_coord(:,2), this.IRM.IRM_poly.Vertices(:,1), this.IRM.IRM_poly.Vertices(:,2));
                task_invalid_area        = task_invalid_area + sum(in) + sum(on);
                
                obs_availability(s_idx)  = 1 - obs_invalid_area/IRM_area;
                task_availability(s_idx) = 1 - task_invalid_area/s_size;
            end
            obs_availability    = round(obs_availability, 10);
            TF_obs              = islocalmin(obs_availability, 'MinSeparation', 20, 'FlatSelection', 'first');
            TF_obs              = TF_obs | islocalmin(obs_availability, 'MinSeparation', 20, 'FlatSelection', 'last');
            break_pts           = this.s(TF_obs);
            task_availability   = round(task_availability, 10);
            TF_task             = islocalmin(task_availability, 'MinSeparation', 20, 'FlatSelection', 'first');
            TF_task             = TF_task | islocalmin(task_availability, 'MinSeparation', 20, 'FlatSelection', 'last');
            break_pts           = sort([break_pts; this.s(TF_task)]);
            
            if this.IsDEBUG
                scatter(this.task_coord(TF_obs,1), this.task_coord(TF_obs,2))
                scatter(this.task_coord(TF_task,1), this.task_coord(TF_task,2))
                ax2 = subplot(1,2,2);
                x   = this.s';
                plot(x, obs_availability, x(TF_obs), obs_availability(TF_obs), 'r*')
                hold on
                plot(x, task_availability, x(TF_task), task_availability(TF_task), 'g*')
                legend({'availability considering obstacles only', 'obstacles local minima', 'availability considering task only', 'task local minima'})
            end
        end
        
        function break_pts = Breakpoints_IRM(this) % NOT IN USE
            % Use the number of valid pose in the IRM as availability
            % Probably can replace the above function
            s_size      = length(this.s);
            IRM_cp_size = length(this.IRM.IRM_checking_pts);
            
            % Approachability map:
            obs_approachability_map  = false(IRM_cp_size, s_size);
            task_approachability_map = false(IRM_cp_size, s_size);
            
            % Initial approachability:
            T_s_coord                 = this.task_coord(1, :);
            this.IRM.IRM_checking_pts = [this.IRM.IRM_checking_pts(:,1) + T_s_coord(1), this.IRM.IRM_checking_pts(:,2) + T_s_coord(2)];
            initial_orientation       = -pi/2; % can be other orientations, need to discuss
            for cp_idx = 1:IRM_cp_size
                [IsObstaclefree, IsTaskfree] = this.Decomposition_ValidityCheck(this.IRM.IRM_checking_pts(cp_idx,:), initial_orientation);
                obs_approachability_map(cp_idx, 1)  = IsObstaclefree;
                task_approachability_map(cp_idx, 1) = IsTaskfree;
            end
            this.IRM.IRM_checking_pts = [this.IRM.IRM_checking_pts(:,1) - T_s_coord(1), this.IRM.IRM_checking_pts(:,2) - T_s_coord(2)];
            
            for s_idx = 2:s_size
                % consider progress s and s - 1:
                T_s_coord_prev = this.task_coord(s_idx - 1, :);
                T_s_coord      = this.task_coord(s_idx, :);
                diff_coord     = T_s_coord - T_s_coord_prev;
                orientation    = atan2(diff_coord(2), diff_coord(1));
                
                this.IRM.IRM_checking_pts = [this.IRM.IRM_checking_pts(:,1) + T_s_coord(1), this.IRM.IRM_checking_pts(:,2) + T_s_coord(2)];
                for cp_idx = 1:IRM_cp_size
                     [IsObstaclefree, IsTaskfree] = this.Decomposition_ValidityCheck(this.IRM.IRM_checking_pts(cp_idx,:), orientation);
                     if (obs_approachability_map(cp_idx, s_idx - 1) && IsObstaclefree) || (~obs_approachability_map(cp_idx, s_idx - 1) && IsObstaclefree)
                        obs_approachability_map(cp_idx, s_idx) = true;
                     end
                     
                     if (task_approachability_map(cp_idx, s_idx - 1) && IsTaskfree) || (~task_approachability_map(cp_idx, s_idx - 1) && IsTaskfree)
                        task_approachability_map(cp_idx, s_idx) = true;
                     end
                end
                this.IRM.IRM_checking_pts = [this.IRM.IRM_checking_pts(:,1) - T_s_coord(1), this.IRM.IRM_checking_pts(:,2) - T_s_coord(2)];
            end
            obs_approachability  = sum(obs_approachability_map);
            TF_obs               = islocalmin(obs_approachability, 'FlatSelection', 'first');
            TF_obs               = TF_obs | islocalmin(obs_approachability, 'FlatSelection', 'last');
            break_pts            = this.s(TF_obs);
            task_approachability = sum(task_approachability_map);
            TF_task              = islocalmin(task_approachability, 'FlatSelection', 'first');
            TF_task              = TF_task | islocalmin(task_approachability, 'FlatSelection', 'last');
            break_pts            = sort([break_pts; this.s(TF_task)]);
            
            if this.IsDEBUG
                scatter(this.task_coord(TF_obs,1), this.task_coord(TF_obs,2))
                scatter(this.task_coord(TF_task,1), this.task_coord(TF_task,2))
                ax2 = subplot(1,2,2);
                x = this.s';
                plot(x, obs_approachability, x(TF_obs), obs_approachability(TF_obs), 'r*')
                hold on
                plot(x, task_approachability, x(TF_task), task_approachability(TF_task), 'g*')
                legend({'availability considering obstacles only', 'obstacles local minima', 'availability considering task only', 'task local minima'})
            end
        end
        
        function [IsObstaclefree, IsTaskfree] = Decomposition_ValidityCheck(this, checking_pt, orientation) % NOT IN USE
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Check if the point pt is valid
        % 1. No collision with task
        % 2. No collision with environment
        % ** Different from Validity Check:
        % 1. Points received are expected to be in IRM
        % 2. Here we assume the task is fully drawn to detect possible
        % decomposition due to task collision
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
        
            IsObstaclefree = true;
            IsTaskfree     = true;
            % Robot hit box pose:
            ro_T    = [cos(orientation), -sin(orientation), checking_pt(1);
                       sin(orientation),  cos(orientation), checking_pt(2);
                                      0,                 0,             1];
            
            Robot   = (ro_T*this.robot_hitbox)';
            
            % Check if robot collide with obstacles
            for o_idx = 1:length(this.obstacles)
                IsCollide = any(inpolygon(this.obstacles{o_idx}.Vertices(:,1), this.obstacles{o_idx}.Vertices(:,2), Robot(:,1), Robot(:,2)));
                Iscollide = IsCollide || any(inpolygon(Robot(:,1), Robot(:,2), this.obstacles{o_idx}.Vertices(:,1), this.obstacles{o_idx}.Vertices(:,2)));
                
                if Iscollide
                    if this.IsDEBUG
                        disp("Decomposition - Robot collide with obstacles")
                    end
                    IsObstaclefree = false;
                end
            end
            
            IsCollide = any(inpolygon(this.task_coord(:, 1), this.task_coord(:, 2), Robot(:,1), Robot(:,2)));
            if IsCollide
                if this.IsDEBUG
                    disp("Decomposition - Robot collide with task")
                end
                IsTaskfree = false;
            end
        end
        
        function break_pts = Breakpoints_obs_task(this) % NOT IN USE
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Find shortest distance between vertex <--> task of every
        % obstacle
        % If the distances is smaller than the robot's diagonal size
        % --> break point
        % In case it is an edge, take the corners
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            
            % robot's diagonal size
            robot_diag_size = sqrt((max(this.robot_hitbox(1,:)) - min(this.robot_hitbox(1,:)))^2 + (max(this.robot_hitbox(2,:)) - min(this.robot_hitbox(2,:)))^2);
            
            % obstacle <--> task
            idxs = [];
            for o_idx = 1:length(this.obstacles)
                obs_shortest_dist_idxs = [];
                current_min_dist = Inf;
                for vertex_idx = 1:size(this.obstacles{o_idx}.Vertices, 1)
                    next_vertex_idx = vertex_idx + 1;
                    if next_vertex_idx > size(this.obstacles{o_idx}.Vertices, 1)
                        next_vertex_idx = 1;
                    end
                    dists    = dist_metric.point_to_line(this.task_coord, this.obstacles{o_idx}.Vertices(vertex_idx,:), this.obstacles{o_idx}.Vertices(next_vertex_idx,:));
                    min_dist = min(dists);
                    if min_dist <= current_min_dist && min_dist <= robot_diag_size
                        current_min_dist       = min_dist;
                        obs_shortest_dist_idxs = find(dists == min_dist);
                        % If indices are consecutive it means the edge is
                        % parallel to task --> take first and last indices
                        % will be sufficient
                        if length(obs_shortest_dist_idxs) > 1 && all(obs_shortest_dist_idxs(2:end) - obs_shortest_dist_idxs(1:end-1))
                            obs_shortest_dist_idxs = [obs_shortest_dist_idxs(1); obs_shortest_dist_idxs(end)];
                        end
                    end
                end
                idxs = unique([idxs; obs_shortest_dist_idxs]);
            end
            % final check for consecutive indices
            diff                = idxs(2:end) - idxs(1:end-1);
            consec_idxs         = find(diff == 1);
            idxs(consec_idxs+1) = [];
            break_pts           = this.s(idxs);
            
            if this.IsDEBUG
                scatter(this.task_coord(idxs,1), this.task_coord(idxs,2))
                % scatter(this.task_coord(TF_task,1), this.task_coord(TF_task,2))
                ax2 = subplot(1,2,2);
            end
        end
        
        function break_pts = Breakpoints_IRM_obs(this, IRM_overlap_threshold, task_ROI_opening_angle)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % This function compute breakpoints for the path planning algorithm
        % to run from multiple points along a large task in parallel
        %
        % Inputs:
        % IRM_overlap_threshold:    Check out Merge_wIRM()
        % task_ROI_opening_angle:   Check out TaskNavigate()
        % Outputs:
        % break_pts:                nx1 vector. Break points / starting points for path
        %                           planning algorithm
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            
            % Robot's diagonal size
            robot_diag_size     = sqrt((max(this.robot_hitbox(1,:)) - min(this.robot_hitbox(1,:)))^2 + (max(this.robot_hitbox(2,:)) - min(this.robot_hitbox(2,:)))^2);
            % robot_min_size      = min(max(this.robot_hitbox(1,:)) - min(this.robot_hitbox(1,:)), max(this.robot_hitbox(2,:)) - min(this.robot_hitbox(2,:)));
            % Consider obstacles
            edges               = this.FindInvolvedEdges();                                  % 1. Find all involved edges
            check_lines         = this.CheckLines(edges, robot_diag_size);                   % 2. Find all checklines with shortest distances between edges < robot size
            progress_idxs       = this.ProgressIdx(check_lines);                             % 3. Find nearest progress locations to the interpolated lines
            % break_pts_idxs      = this.PointsOfDecomposition(progress_idxs, IRM_overlap_threshold);
            obs_break_pts_idxs  = this.Merge_wIRM(progress_idxs, IRM_overlap_threshold);     % 4. If IRM overlap ?% up & availability > ?% --> merge
            
            % Consider task
%             task_break_pts_idxs = this.TaskNavigate(task_ROI_opening_angle);
            
            % Merge break points
            break_pt_idxs       = sort(obs_break_pts_idxs);% , task_break_pts_idxs]);
            break_pts           = this.s(break_pt_idxs);
        end
        
        function task_break_pts_idxs = TaskNavigate(this, task_ROI_opening_angle)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % This function open a fan shape ROI pointing at the direction
        % of the next point of the progress and check if there is any
        % task points fall into the ROI. If yes, that means it is
        % possible that the robot will run into the task at that point.
        % Then finding the local maxima will give possible break points
        % with respect to the task.
        %
        % Example:
        %
        %     task
        %      |          x
        %      |        x   +
        %      v      x      +
        %   --------x ) <--- task_ROI_opening_angle
        %             x      +
        %               x   +
        %                 x
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            
            % Open angle
            angle = [-(task_ROI_opening_angle/2)*pi/180:pi/50:(task_ROI_opening_angle/2)*pi/180];
            ROI   = [0, this.IRM.r_max*cos(angle), 0; 0, this.IRM.r_max*sin(angle), 0; ones(1, size(angle,2) + 2)];
            
            forward_task_density = zeros(size(this.s));
            for s_idx = 2:size(this.task_coord, 1)
                diff_coord   = this.task_coord(s_idx,:) - this.task_coord(s_idx-1,:);
                orientation  = atan2(diff_coord(2), diff_coord(1));
                rot_mat_ROI  = [cos(orientation), -sin(orientation), this.task_coord(s_idx,1);
                                sin(orientation),  cos(orientation), this.task_coord(s_idx,2);
                                              0,                 0,                        1];
                Adjusted_ROI = rot_mat_ROI*ROI;
                
                % Check if any task points forward blocking the way
                [in, on]                    = inpolygon(this.task_coord(1:s_idx,1), this.task_coord(1:s_idx,2), Adjusted_ROI(1,:), Adjusted_ROI(2,:));
                forward_task_density(s_idx) = sum(in|on) - 1; % Ignore the one point at current location
            end
            
            TF_task             = islocalmax(forward_task_density, 'FlatSelection', 'center');
            task_break_pts_idxs = find(TF_task == 1);
            
            if this.IsDEBUG
                scatter(this.task_coord(TF_task,1), this.task_coord(TF_task,2))
                ax2 = subplot(1,2,2);
                x   = this.s';
                plot(x, forward_task_density, x(TF_task), forward_task_density(TF_task), 'r*')
                xlabel('progress')
                legend({'Number of points forward (task)', 'break point based on task only'})
            end
        end
        
        function break_pts_idxs = PointsOfDecomposition(this, progress_idxs, IRM_overlap_threshold)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % 1. Break progress_idxs into clusters
        % 2. For each cluster, if number of indices > 1, take the first and
        %    last indices.
        % 3. Then use RM, check reachability from i - 1 to i, then i to i +
        %    1 until the last index n, then check n --> n + 1
        % 4. If come to a point where i + k --> i + k + 1 is not
        %    successful, record and break
        % 5. Then start from n + 1, check reachability to n ... n - k --> n
        %    - k - 1
        % 6. If i + k + 1 = n - k - 1, then there will only be one
        %    decomposition point, else there are 2. In this case, the region
        %    between i + k + 1 and n - k - 1 is unprintable.
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
             progress_idxs
        end
        
        function [SDCM_All, GF] = get_jenks_interface(this, Array)
            total = length (Array);
            SDCM_All = zeros(1,total);
            GF = zeros(1,total);
            % step 1: Calculate the sum of the squared deviations from the mean (SDAM)
            SDAM = this.get_sum_deviations(Array);
            % step 2: Calculate the sum of the squared deviations for every possible
            % combination of classes
            for i=1:total    
                class_1 = Array(1:i);
                class_2 = Array(i+1:total);

                s1 = this.get_sum_deviations(class_1);
                s2 = this.get_sum_deviations(class_2);

                SDCM_All(i) = s1 + s2;
                % Calculate the goodness of variance fit 
                GF(i) = ((SDAM - SDCM_All(i)) / SDAM) ;
            end
        end
        
        function [sum_deviations] = get_sum_deviations(this, Array)
            mean_array = mean (Array);
            sum_deviations = 0;
            for i=1:length(Array)
                %sum all the deviations from the mean of the array
                sum_deviations = sum_deviations + ((Array(i) - mean_array)^2);
            end
        end
        
        function break_pts_idxs = Merge_wIRM(this, progress_idxs, IRM_overlap_threshold)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % This function merge progress_idxs that are close together by
        % using their IRMs. If the intersecting area of the IRMs >
        % IRM_overlap_threshold then the two points will merge to one point
        % in the middle.
        %
        % Inputs:
        % progress_idxs:            nx1 vector from ProgressIdx()
        % IRM_overlap_threshold:    0 - 1 scalar, threshold for IRMs
        %                           overlapping area
        % Outputs:
        % break_pts_idxs:           Final set of break points considering
        %                           obstacles only
        %
        % Example:
        %          overlapping area (* filled)
        %                       |
        %                       v
        %            +----------+----------+
        %  IRM1 --> /          / \          \ <-- IRM2
        %          /    +---+ / * \ +---+    \
        %         +     | x |+ * * +| x |     +
        %          \    +---+ \ * / +---+    /
        %           \          \ /          /
        %            +----------+----------+
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
        
            IRM_area = area(this.IRM.IRM_poly);
            
            % loop condition:
            while true
                progress_overlap_mat = NaN(size(progress_idxs, 1), size(progress_idxs, 1));
                
                % Check all combinations and put the jth progress in the matrix
                % if larger than threshold
                for idx = 1:size(progress_idxs, 1)
                    for jdx = 1:size(progress_idxs, 1)
                        IRM1          = this.IRM.IRM_poly;
                        IRM1.Vertices = IRM1.Vertices + this.task_coord(progress_idxs(idx),:);
                        IRM2          = this.IRM.IRM_poly;
                        IRM2.Vertices = IRM2.Vertices + this.task_coord(progress_idxs(jdx),:);

                        inter_area    = area(intersect(IRM1, IRM2))/IRM_area;

                        if inter_area > IRM_overlap_threshold
                            progress_overlap_mat(idx, jdx) = this.s(progress_idxs(jdx));
                        end
                    end
                end
                
                % Take mean --> auto clustering
                progress_to_check = mean(progress_overlap_mat, 2, 'omitnan');
                
                % TODO: Include Availability !!!//////////////////
                % progress_to_check = this.VerifyAvailability(progress_to_check, progress_overlap_mat, progress_idxs);
                
                if all(mean(progress_overlap_mat - diag(diag(progress_overlap_mat)), 2, 'omitnan') == 0) % i.e. The points left is very far apart
                    break
                end
                
                % Get corresponding indices and take the set
                break_pts_idxs = [];
                for kdx = 1:size(progress_to_check, 1)
                    break_pts_idxs = [break_pts_idxs; min(length(this.s), round(length(this.s)*progress_to_check(kdx)))];
                end
                progress_idxs = unique(break_pts_idxs);
            end
            
            break_pts_idxs = [];
            for kdx = 1:size(progress_to_check, 1)
                break_pts_idxs = [break_pts_idxs; min(length(this.s), ceil(length(this.s)*progress_to_check(kdx)))];
            end
            break_pts_idxs = unique(break_pts_idxs);
            
            if this.IsDEBUG
                scatter(this.task_coord(break_pts_idxs, 1), this.task_coord(break_pts_idxs, 2), 'MarkerEdgeColor', [0 .5 .5], 'MarkerFaceColor', [0 .7 .7], 'LineWidth',1.5)
            end
        end
        
        function [progress_to_check, progress_overlap_mat] = VerifyAvailability(this, progress_to_check, progress_overlap_mat, progress_idxs)
            % Wakarimasenn :) ichidann mushi shimasu
            % use IRM_poly to check availability
            % progress_to_check           = unique(progress_to_check);
            validated_progress_to_check = [];
            for idx = 1:size(progress_to_check, 1)
                s_idx        = min(length(this.s), ceil(length(this.s)*progress_to_check(idx)));
                T_coord_prev = this.task_coord(s_idx - 1, :);
                T_coord      = this.task_coord(s_idx, :);
                diff_coord   = T_coord - T_coord_prev;
                orientation  = atan2(diff_coord(2), diff_coord(1));
                
                this.IRM.IRM_checking_pts = [this.IRM.IRM_checking_pts(:,1) + T_coord(1), this.IRM.IRM_checking_pts(:,1) + T_coord(2)];
                Is_obs_free_vec           = false(size(this.IRM.IRM_checking_pts, 1), 1);
                for jdx = 1:size(this.IRM.IRM_checking_pts, 1)
                    Is_obs_free_vec(jdx) = IsObstacleFree(this, this.IRM.IRM_checking_pts(jdx,:), orientation);
                end
                this.IRM.IRM_checking_pts = [this.IRM.IRM_checking_pts(:,1) - T_coord(1), this.IRM.IRM_checking_pts(:,1) - T_coord(2)];
                if sum(Is_obs_free_vec)/size(this.IRM.IRM_checking_pts, 1) < 0.25 % threshold
                    % Hard to reach, revert
                    % 1. find
                    temp_progress = sort([progress_idxs; s_idx]);
                    location      = find(temp_progress == s_idx);
                    try
                        validated_progress_to_check = [validated_progress_to_check; temp_progress(location - 1)];
                    catch
                    end
                    try
                        validated_progress_to_check = [validated_progress_to_check; temp_progress(location + 1)];
                    catch
                    end
                    % 
                    temp_diag_val                 = progress_overlap_mat(idx,idx);
                    progress_overlap_mat(idx,:)   = NaN;
                    progress_overlap_mat(idx,idx) = temp_diag_val;
                else
                    validated_progress_to_check = [validated_progress_to_check; s_idx];
                end
            end
            validated_progress_to_check = unique(validated_progress_to_check);
            corrected_progress_to_check = zeros(size(validated_progress_to_check));
            for kdx = 1:size(validated_progress_to_check, 1)
                corrected_progress_to_check(kdx) = this.s(validated_progress_to_check(kdx));
            end
            progress_to_check = corrected_progress_to_check;
        end
        
        function Is_obstacle_free = IsObstacleFree(this, checking_pt, orientation)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Check if the point pt has no collision with environment
        % ** Different from Validity Check:
        % 1. Points received are expected to be in IRM
        % 2. Here we assume the task is fully drawn to detect possible
        % decomposition due to task collision
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
        
            Is_obstacle_free = true;
            
            % Robot hit box pose:
            ro_T    = [cos(orientation), -sin(orientation), checking_pt(1);
                       sin(orientation),  cos(orientation), checking_pt(2);
                                      0,                 0,             1];
            
            Robot   = (ro_T*this.robot_hitbox)';
            
            % Check if robot collide with obstacles
            for o_idx = 1:length(this.obstacles)
                IsCollide = any(inpolygon(this.obstacles{o_idx}.Vertices(:,1), this.obstacles{o_idx}.Vertices(:,2), Robot(:,1), Robot(:,2)));
                Iscollide = IsCollide || any(inpolygon(Robot(:,1), Robot(:,2), this.obstacles{o_idx}.Vertices(:,1), this.obstacles{o_idx}.Vertices(:,2)));
                
                if Iscollide
                    if this.IsDEBUG
                        disp("Decomposition - Robot collide with obstacles")
                    end
                    Is_obstacle_free = false;
                end
            end            
        end
        
        function progress_idxs = ProgressIdx(this, check_lines)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % This function find the indices of the task points which are
        % nearest to the checklines and are within a tolerance of a
        % distance = mean distance between task points (progress_delta_dist)
        % 
        % Inputs:
        % checklines:       nx4 matrix from CheckLines()
        % Outputs:
        % progress_idxs:    nx1 vector storing all the indices that
        %                   corresponds to the points which are nearest to
        %                   the checklines
        %
        % Example:
        %                      +-- nearest progress
        %  ---------+       : /     +----------
        %           |       :/      |
        %  obstacle x- - - -+- - - -x obstacle
        %           |       :    ^  |
        %         task ---> :    |  +----------
        %           |       :    +--- checkline
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

            progress_idxs = [];
            progress_delta_dist = mean(vecnorm(this.task_coord(1:end-1,:) - this.task_coord(2:end,:), 2, 2));
            for cl_idx = 1:size(check_lines, 1)
                % check the cloest progress and ensure it is <= progress_delta_dist
                [dists, ~] = dist_metric.point_to_line(this.task_coord, check_lines(cl_idx, 1:2), check_lines(cl_idx, 3:4));
                [min_dist, idx] = min(dists);
                if min_dist < progress_delta_dist % To take care places where there is no task
                    progress_idxs = [progress_idxs; idx];
                end
            end
            progress_idxs = unique(progress_idxs);
            
            if this.IsDEBUG
                scatter(this.task_coord(progress_idxs, 1), this.task_coord(progress_idxs, 2))
            end
        end
        
        function check_lines = CheckLines(this, edges, robot_diag_size)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Consider edges found in FindInvolvedEdges(), this function
        % construct lines between obstacles that has a length <= robot size
        % these lines are then use to find nearest point on the task to
        % give a rough sense of where the break points are.
        % 
        % Inputs:
        % edges:            nx5 matrix from FindInvolvedEdges()
        % robot_diag_size:  scalar, length of the robot's diagonal length
        % Outputs:
        % checklines:       nx4 matrix, each row is a checkline [x_1 y_1 x_2 y_2]
        %
        % Example:
        %                          :
        %              task -----> : +-- checkline
        %   -------------------+   : |
        %                      |   : v
        %                      x- -:- - x-----------------
        %      obstacle 1      |   :    |
        %                      |   :    |    obstacle 2
        %                      |   :    |
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
        
            check_lines = [];
            for idx = 1:size(edges,1)
                for jdx = 1:size(edges,1)
                    if idx ~= jdx && edges(idx, end) ~= edges(jdx, end) % ignore checking within one obstacle
                        [d1, pt_per1] = dist_metric.point_to_line(edges(jdx, 1:2), edges(idx, 1:2), edges(idx, 3:4));
                        [d2, pt_per2] = dist_metric.point_to_line(edges(jdx, 3:4), edges(idx, 1:2), edges(idx, 3:4));
                        
                        if d1 <= robot_diag_size % && d1 > 0
                            check_lines = [check_lines; edges(jdx, 1:2), pt_per1];
                        end
                        if d2 <= robot_diag_size % && d2 > 0
                            check_lines = [check_lines; edges(jdx, 3:4), pt_per2];
                        end
                    end
                end
            end
            check_lines = unique(check_lines, 'rows');
            
            if this.IsDEBUG
                for idx = 1:length(check_lines)
                    line([check_lines(idx, 1), check_lines(idx,3)], [check_lines(idx,2), check_lines(idx,4)], 'Color', '#77AC30', 'LineStyle', '--')
                end
            end
        end
        
        function edges = FindInvolvedEdges(this)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Check every edge on every obstacles to see if they are in the range of IRMs along the task
        % Inputs:
        % this:  class object
        % Outputs:
        % edges: nx5 matrix with every row representating a line [x_1 y_1 x_2 y_2 obstacleID]
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
        
            edges = [];
            for o_idx = 1:length(this.obstacles)
                for vertex_idx = 1:size(this.obstacles{o_idx}, 1)
                    next_vertex_idx = vertex_idx + 1;
                    if next_vertex_idx > size(this.obstacles{o_idx}, 1)
                        next_vertex_idx = 1;
                    end
                    [dists, ~] = dist_metric.point_to_line(this.task_coord, this.obstacles{o_idx}(vertex_idx,:), this.obstacles{o_idx}(next_vertex_idx,:));
                    if any(dists <= this.IRM.r_max)
                        edges = [edges; this.obstacles{o_idx}(vertex_idx,:), this.obstacles{o_idx}(next_vertex_idx,:), o_idx];
                    end
                end
            end
            
            if this.IsDEBUG
                for idx = 1:size(edges, 1)
                    line([edges(idx, 1), edges(idx, 3)], [edges(idx,2), edges(idx,4)], 'color', '#D95319', 'LineWidth', 2)
                end
                drawnow
            end
        end
    end
end