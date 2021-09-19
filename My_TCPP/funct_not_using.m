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
        function new_pt_pose = TD_Extend(this, to_pt, from_pt, delta_l_step, delta_l, dist_method)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % 
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            new_pt    = node_SE2.copy(from_pt);
            
            % angle should add/subtract in radians
            d_theta	  = atan2(to_pt.pose(4), to_pt.pose(3)) - atan2(from_pt.pose(4), from_pt.pose(3));
            d_theta   = atan2(sin(d_theta), cos(d_theta));

            line_step = ([to_pt.pose(1:2) - from_pt.pose(1:2), d_theta, to_pt.pose(5) - from_pt.pose(5)].*delta_l_step); %/dist_metric.method(to_pt, from_pt, method);
            while dist_metric.method(to_pt, new_pt, dist_method) > delta_l_step
                % from_pt o--->--->---x to_pt
                % angle should add/subtract in radians
                angle       = atan2(new_pt.pose(4), new_pt.pose(3)) + line_step(3);
                new_pt.pose = [new_pt.pose(1:2) + line_step(1:2), cos(angle), sin(angle), new_pt.pose(5) + line_step(4)];
                
                if this.IsDEBUG
                    assert(sum(new_pt.pose ~= from_pt.pose) > 0)
                end
                
                if ~this.CollisionCheck(new_pt) || dist_metric.method(new_pt, from_pt, dist_method) > delta_l
                    % Revert, save, return
                    % angle should add/subtract in radians
                    angle       = atan2(new_pt.pose(4), new_pt.pose(3)) - line_step(3);
                    new_pt.pose = [new_pt.pose(1:2) - line_step(1:2), cos(angle), sin(angle), new_pt.pose(5) - line_step(4)];
                    new_pt_pose = new_pt.pose;
                    return
                end
            end
            % sampled point is less then one step away:
            new_pt_pose = to_pt.pose;
        end
        
        function IsValid = CollisionCheck(this, node)
            %% TO BE DELETED
            IsValid = true;
            % Robot hit box pose:
            ro_T = [node.pose(3), -node.pose(4), node.pose(1);
                    node.pose(4),  node.pose(3), node.pose(2);
                               0,             0,            1];
            
            Robot_Poly = (ro_T*this.Env.robot_hitbox)';
            
            % Check if robot collide with obstacles
            for o_idx = 1:length(this.Env.obstacles)
                IsCollide = any(inpolygon(this.Env.obstacles{o_idx}.Vertices(:,1), this.Env.obstacles{o_idx}.Vertices(:,2), Robot_Poly(:,1), Robot_Poly(:,2)));
                Iscollide = IsCollide || any(inpolygon(Robot_Poly(:,1), Robot_Poly(:,2), this.Env.obstacles{o_idx}.Vertices(:,1), this.Env.obstacles{o_idx}.Vertices(:,2)));
                
                if Iscollide
                    if this.IsDEBUG
                        %disp("Robot collide with obstacles")
                    end
                    IsValid = false;
                    return;
                end
            end
        end
        
        function nodes = TD_sample_pts(this, n, varargin)
            %% TO BE DELETED
            if ~isempty(varargin)
                nodes   = varargin{1};
            else
                nodes   = [];
            end
            
            for idx = 1:n
                pt = [randi(6)*rand(1,2), 1, 0, 0];
                pt = node_SE2(pt);
                
                if isempty(nodes)
                    nodes = pt;
                else
                    CLS_KDTree.insert(pt, nodes);
                end
            end
        end
        