classdef CLS_RRTStar
    properties
        Env                                         % Environment that RRT* is exploring with
        start_nodes                                 % n sampled starting nodes with structure node_SE2
        NN_method = 'KDTree_progress_sq_norm';      % Nearest Neighbour search method
        s_f                                         % Ending progress (can be a number other than 1)
        break_pts                                   % break points of the whole task
        IsDEBUG                                     % Debug Flag
    end
    
    methods
        function this = CLS_RRTStar(Env, start_nodes, varargin)
        %% Initialize://////////////////////////////////////////////////////////////////////////////
            this.Env           = Env;
            this.IsDEBUG       = this.Env.IsDEBUG;
            this.start_nodes   = start_nodes;
            if ~isempty(varargin)
                this.break_pts = varargin{1};
                this.s_f       = this.break_pts(2:end);
            else
                this.s_f       = max(this.Env.s);
            end
        end
        
        function [path, ite] = RRT_Star(this)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % RRT* algorithm begins by initializing a tree T = (V, E) with a
        % vertex at the initial configuration (i.e. V = {q_I}). 
        % At each iteration the RRT algorithm then performs the following steps:
        % 1. The planner samples a random state x_rand in the state space.
        % 2. The planner finds a state x_near that is already in the search tree and is closest 
        %    (based on the distance definition in the state space) to x_rand.
        % 3. The planner expands from x_near towards x_rand, until a state x_new is reached.
        % 4. Then new state x_new is added to the search tree.
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            % ***********************Search task ***********************
            nodes                     = this.start_nodes;               % Init start nodes
            num_paths                 = length(nodes);                  % Number of path exploring simutaneously
            % ***********************Search task ***********************
            
            % *********************** Parameters ***********************
            dist_method               = 'progress_sq_norm';             % Method name for calculating distance
            max_iter                  = 1e4;                            % Max. iteration
            delta_l                   = 0.1;                            % RRT edge length
            delta_relink_l            = 0.2;                            % Relink edge length
            delta_l_step              = delta_l/5;                      % RRT edge step size
            delta_relink_l_step       = delta_l_step;                   % Relink edge step size
            if ~isempty(this.break_pts)
                s_max                 = this.break_pts(1:end-1);        % Current furthest progress
            else
                s_max                 = 0;
            end
            % *********************** Parameters ***********************
            
            ite                       = 1;
            Reached_Goal              = false(length(nodes), 1);
            % while this.Reached_Goal(nodes, pt_goal, 0.3, this.NN_method) == 0 && ite <= max_iter
            while ~all(Reached_Goal) && ite <= max_iter
                %% Main loop
                for idx = 1:length(nodes)
                    if ~Reached_Goal(idx)
                        % Step 1:
                        [pt_new, NN_pt, nodes{idx}, current_IRM] = this.S1_SampleAndLink(nodes{idx}, dist_method, this.break_pts(idx), s_max(idx), this.s_f(idx), delta_l_step, delta_l);
                        if this.IsDEBUG
                            try
                                delete(current_IRM_plot);
                            catch
                            end
                            current_IRM_plot = fill(current_IRM(1,:), current_IRM(2,:), 'b', 'FaceAlpha', 0.2, 'EdgeColor',"none");
                        end

                        % Step 2: Relink:
                        [pt_new, min_pt, nodes{idx}] = this.S2_Relink(pt_new, NN_pt, nodes{idx}, dist_method, delta_relink_l_step, delta_relink_l);

                        % Step 3: build KD Tree relations:
                        [isReached, isInserted] = CLS_KDTree.insert(pt_new, nodes{idx}(1));
                        if isReached && isInserted
                            nodes{idx} = [nodes{idx}; pt_new];
                        end

                        % Step 4: Update current furthest progress:
                        if pt_new.pose(5) > s_max(idx)
                            s_max(idx) = pt_new.pose(5);
                        end

                        if pt_new.pose(5) >= this.s_f(idx)
                            Reached_Goal(idx) = true;
                            Goals{idx}        = pt_new;
                        end
                        ite = ite + 1;

                        if this.IsDEBUG
                            try
                                delete(progress_plot);
                                delete(Robot_plot);
                                delete(latest_IRM_plot);
                            catch
                            end
                            [progress_plot, Robot_plot, latest_IRM_plot] = this.Plot_state(min_pt, pt_new, s_max(idx));
                            hold on
                            assert(pt_new.parent ~= 0);
                        end
                    end
                end
            end
            
            for jdx = 1:length(Goals)
                % Step 5: Find path
                q_end       = node_SE2.copy(Goals{jdx});
                path{jdx}	= q_end;

                while ~isempty(q_end.parent)
                    next_parent = node_SE2.copy(q_end.parent);
                    path{jdx}   = [next_parent; path{jdx}];
                    q_end       = next_parent;
                end
                
                if this.IsDEBUG
                    for kdx = length(path{jdx}):-1:2
                        line([path{jdx}(kdx-1).pose(1), path{jdx}(kdx).pose(1)], [path{jdx}(kdx-1).pose(2), path{jdx}(kdx).pose(2)], [0,0], 'Color', '#E1701A', 'LineWidth', 4);
                    end
                    POSES = this.Extract_item(path{jdx}, 'pose');
                    quiver(POSES(:,1), POSES(:,2), POSES(:,3), POSES(:,4), 'Color', '#F7A440', 'LineWidth', 2);
                end
            end
        end
        
        function [pt_new, min_pt, nodes] = S2_Relink(this, pt_new, NN_pt, nodes, dist_method, delta_relink_l_step, delta_relink_l)
            %% Description://///////////////////////////////////////////////////////////////////////////
            % Step 2 of RRT*: Relink
            % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            % Find all nodes within a radius of r with respect to the new point:
            nodes_new_node_dists = dist_metric.method(pt_new, nodes, dist_method);
            nodes_inROI          = node_SE2.copy(nodes(nodes_new_node_dists <= delta_relink_l));

            % Search all nodes in ROI to find paths with lower costs:
            [min_cost, min_idx]  = min(this.Extract_item(nodes_inROI, 'cost') + dist_metric.method(pt_new, nodes_inROI, dist_method));
            if isempty(min_idx)
                min_pt = node_SE2.copy(NN_pt);
            else
                min_pt = node_SE2.copy(nodes_inROI(min_idx));
            end

            % (recurrsion heirarchy smoothing)
            pt_new.pose = this.Extend(pt_new, min_pt, delta_relink_l_step, delta_relink_l, dist_method);

            % Choose Parent:
            % Just search the exact index, no need angle difference --> KDTree_sq_norm:
            [pt_new.parent, ~, ~] = this.Nearest_Neighbour(min_pt, nodes, 'KDTree_sq_norm');
        end
        
        function [pt_new, NN_pt, nodes, current_IRM] = S1_SampleAndLink(this, nodes, dist_method, s_break, s_max, s_f, delta_l_step, delta_l)
            %% Description://///////////////////////////////////////////////////////////////////////////
            % Step 1 of RRT*: Sample and link
            % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            
            % Initialize:
            NN_val      = Inf;
            pt_new      = node_SE2([]);
            pt_new.cost = Inf;

            while NN_val == Inf || pt_new.cost == Inf
                % Generate random point:
                IsValid = false;
                while ~IsValid
                    [IsGoal, rand_pt, current_IRM]  = this.Rand_Config(s_break, s_max, s_f);
                    rand_pt                         = node_SE2(rand_pt);
                    IsValid                         = this.Env.ValidityCheck(rand_pt);
                end


                % Find nearest point to rand_pt:
                [NN_pt, NN_val, NN_idx] = this.Nearest_Neighbour(rand_pt, nodes, this.NN_method);

                if NN_val ~= Inf
                    % Find nearest point to rand_pt:
                    pt_new.pose = this.Extend(rand_pt, NN_pt, delta_l_step, delta_l, dist_method);
                    pt_new.cost = dist_metric.method(pt_new, NN_pt, dist_method) + NN_pt.cost;
                end
            end
        end
        
        function [IsGoal, rand_pt, current_IRM] = Rand_Config(this, s_break, s_max, s_f)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Return a random sampled point, current Inverse Reachability Map
        % (IRM) and IsGoal
        % 
        % Input:
        % s_max:        Current furthest progress
        % 
        % Output:
        % IsGoal:       A point is sampled in the last IRM of the printing
        % rand_pt:      Random point within the searching boundaries
        % current_IRM:  Current IRM
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            mean   = s_max;                 % Current furthest progress
            s_var  = 0.03*s_f;         % Variance
            sigma  = sqrt(s_var);           % Wiggle around Current furthest progress
            s_Norm = sigma*randn(1) + mean; % Normal distribution sampling

            s_rand = max(s_break, min(s_f, s_Norm));
            
            % Find correspond nearest progress:
            Ts_idx                 = min(size(this.Env.task_T, 3), max(1, ceil(size(this.Env.task_T, 3)*s_rand)));
            T_s                    = this.Env.task_T(:, :, Ts_idx);
            [rand_pt, current_IRM] = this.Env.IRM.sample(T_s(1:3, 4), s_rand);
            
            if s_rand >= 1
                IsGoal = true;
            else
                IsGoal = false;
            end
        end
        

        function val = Reached_Goal(this, nodes, q_goal, delta_l, NN_method) % NOT USING
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Check if goal reached or not
        % Return boolean (val) where value true indicates goal point is reached.
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if strcmp(NN_method, 'sq_norm')
                val = any(this.dist_metric(q_goal, nodes, NN_method) <= delta_l);

            elseif strcmp(NN_method, 'KDTree_progress_sq_norm')
                [~, min_dist] = CLS_KDTree.Nearest_Neighbour(q_goal, nodes(1), 'progress_sq_norm');
                val           = (min_dist <= delta_l);
            end
        end
        
        function val = Extract_item(this, nodes, item)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Return all items stored
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
        
        function new_pt_pose = Extend(this, to_pt, from_pt, delta_l_step, delta_l, dist_method)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % 
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            new_pt    = node_SE2.copy(from_pt);
            
            % angle should add/subtract in radians
            d_theta	  = atan2(to_pt.pose(4), to_pt.pose(3)) - atan2(from_pt.pose(4), from_pt.pose(3));
            d_theta   = atan2(sin(d_theta), cos(d_theta));

            line_step = ([to_pt.pose(1:2) - from_pt.pose(1:2), d_theta, to_pt.pose(5) - from_pt.pose(5)].*delta_l_step); %/dist_metric.method(to_pt, from_pt, method);
            while dist_metric.method(to_pt, from_pt, dist_method) > delta_l_step
                % from_pt o--->--->---x to_pt
                % angle should add/subtract in radians
                angle       = atan2(new_pt.pose(4), new_pt.pose(3)) + line_step(3);
                new_pt.pose = [new_pt.pose(1:2) + line_step(1:2), cos(angle), sin(angle), new_pt.pose(5) + line_step(4)];
                
                if this.IsDEBUG
                    assert(sum(new_pt.pose ~= from_pt.pose) > 0)
                end
                
                if ~this.Env.ValidityCheck(new_pt) || dist_metric.method(new_pt, from_pt, dist_method) > delta_l
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
        
        function [NN_pt, NN_val, NN_idx] = Nearest_Neighbour(this, pos, nodes, method)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Return information of the nearest neighbour of pos 
        % Inputs:
        % 1. pos: [x y cos(theta) sin(theta) t]
        % 2. neighbours: X_neigh \in R^(nx3)
        % Methods:
        % 1. Distance: ||x_i - p||
        % 2. KDTree
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if strcmp(method, 'sq_norm')
                dists            = dist_metric.method(pos, nodes, method);
                [NN_val, NN_idx] = min(dists);
                NN_pt            = node_SE2.copy(nodes(NN_idx));
                
            elseif strcmp(method, 'KDTree_sq_norm')
                [NN_pt, NN_val]  = CLS_KDTree.Nearest_Neighbour(pos, nodes(1), 'sq_norm', 1);
                NN_idx = NaN;
            elseif strcmp(method, 'KDTree_progress_sq_norm')
                [NN_pt, NN_val]  = CLS_KDTree.Nearest_Neighbour(pos, nodes(1), 'progress_sq_norm', 1);
                NN_idx = NaN;
            end
        end
        
        function [progress_plot, Robot_plot, latest_IRM_plot] = Plot_state(this, min_pt, pt_new, s_max)
            % Plot current progress
            progress_plot = this.Env.task_OBJ.plot(min(size(this.Env.task_OBJ.path, 1), max(1, ceil(size(this.Env.task_OBJ.path, 1)*s_max))));

            % Plot edge and direction
            line([min_pt.pose(1), pt_new.pose(1)], [min_pt.pose(2), pt_new.pose(2)], [0, 0], 'Color', '#316879', 'LineWidth', 2);
            quiver(pt_new.pose(:,1), pt_new.pose(:,2), pt_new.pose(:,3), pt_new.pose(:,4), 0.2, 'LineWidth', 2)

            % Plot robot
            ro_T            = [pt_new.pose(3), -pt_new.pose(4), pt_new.pose(1);
                               pt_new.pose(4),  pt_new.pose(3), pt_new.pose(2);
                                            0,               0,             1];
            Robot_Poly      = (ro_T*this.Env.robot_hitbox)';
            Robot_plot      = fill(Robot_Poly(:,1), Robot_Poly(:,2), 'g', 'FaceAlpha', 0.3);
            
            % Plot IRM
            Ts_idx          = min(size(this.Env.task_T, 3), max(1, ceil(size(this.Env.task_T, 3)*s_max)));
            T_s_coord       = this.Env.task_coord(Ts_idx, :);
            latest_IRM      = this.Env.IRM.IRM_Poly(T_s_coord);
            latest_IRM_plot = fill(latest_IRM(1,:), latest_IRM(2,:), 'y', 'FaceAlpha', 0.3, 'EdgeColor',"none");
            drawnow
        end
    end
end