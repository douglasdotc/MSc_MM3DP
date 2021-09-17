classdef CLS_2DRRTStar
    properties
        Env                                         % Environment that RRT* is exploring with
        start_nodes                                 % n sampled starting nodes with structure node_SE2
        edges
%         NN_method = 'KDTree_sq_norm';
        NN_method = 'KDTree_progress_sq_norm';      % Nearest Neighbour search method
        s_i
        s_f                                         % Ending progress (can be a number other than 1)
        break_pts                                   % break points of the whole task
        IsDEBUG                                     % Debug Flag
        
        file_name
    end
    
    methods
        function this = CLS_2DRRTStar(Env, start_nodes, varargin)
        %% Initialize://////////////////////////////////////////////////////////////////////////////
            this.Env           = Env;
            this.IsDEBUG       = this.Env.IsDEBUG;
            this.start_nodes   = start_nodes;
            if ~isempty(varargin)
                this.s_i       = varargin{1}(1:end-1);
                this.s_f       = varargin{1}(2:end);
                this.file_name = varargin{2};
            else
                this.s_i       = min(this.Env.s);
                this.s_f       = max(this.Env.s);
            end
        end
        
        function [paths, ite] = RRT_Star(this)
            total_cost      = 0;
            num_path_nodes  = [];
            tic
                [paths, ite, record] = this.RRT_Star_Forward();
            time = toc;
            
            for pdx = 1:length(paths)
                for kdx = length(paths{pdx}):-1:2
                    line([paths{pdx}(kdx-1).pose(1), paths{pdx}(kdx).pose(1)], [paths{pdx}(kdx-1).pose(2), paths{pdx}(kdx).pose(2)], [paths{pdx}(kdx-1).pose(5), paths{pdx}(kdx).pose(5)], 'Color', '#316879', 'LineWidth', 4);
                end
                POSES = this.Env.Extract_item(paths{pdx}, 'pose');
                quiver(POSES(:,1), POSES(:,2), POSES(:,3), POSES(:,4), 0.5, 'Color', '#316879', 'LineWidth', 2);
                drawnow
                
                num_path_nodes  = [num_path_nodes, length(paths{pdx})];
                total_cost      = total_cost + paths{pdx}(end).cost;
            end
            sum_path_nodes = sum(num_path_nodes);
            fprintf("RRT* search done in %.4f sec, %d iterations with path cost %.4f and %d total number of path nodes.\n", time, ite, total_cost, sum_path_nodes);
            fprintf("number of path nodes break down:\n");
            disp(num_path_nodes)
            
            % Save data
            writematrix(num_path_nodes,this.file_name+"path_cost_break_down.txt",'Delimiter',',')
            fileID = fopen(this.file_name+".txt",'w');
            fprintf(fileID, "Time: %.4f\n", time);
            fprintf(fileID, "Iterations %d\n", ite);
            fprintf(fileID, "Path cost %.4f\n", total_cost);
            fprintf(fileID, "Total number of nodes: %d\n", sum_path_nodes);
            fclose(fileID);
            save(this.file_name+".mat");
        end
        
        function [paths, ite, record] = RRT_Star_Forward(this)
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
%             nodes                     = this.TD_sample_pts(1);            % TEMP FOR TEST !!! Init start nodes
            num_paths                 = length(nodes);                  % Number of path exploring simutaneously
            % ***********************Search task ***********************
            
            % *********************** Parameters ***********************
            dist_method               = 'forward_progress_sq_norm';     % Method name for calculating distance
%             dist_method = 'sq_norm';
            max_iter                  = 1e4;                            % Max. iteration
            delta_l                   = 0.05;                            % RRT edge length
            delta_l_step              = 0.01;                      % RRT edge step size
            eps_neigh                 = 0.25;
            eps_neigh_rewire          = 0.25;
            
            if ~isempty(this.s_i)
                s_max                 = this.s_i;        % Current furthest progress
            else
                s_max                 = min(this.Env.s);
            end
            % *********************** Parameters ***********************
            
            ite                       = 1;
            Reached_Goal              = false(num_paths, 1);
            stall_count               = zeros(num_paths, 1);
            Goals                     = cell(num_paths, 1);
            paths                     = cell(1, num_paths);
            record                    = [];
%             while ~all(Reached_Goal)
                %% Main loop
            for pdx = 1:num_paths
                if ~isempty(nodes{pdx})
                    while ~Reached_Goal(pdx)
                        [~, q_rand, ~]      = this.Rand_Config(this.s_i(pdx), s_max(pdx), this.s_f(pdx));
                        q_rand              = node_SE2(q_rand);

                        [q_nearest, ~, ~]   = this.Nearest_Neighbour(q_rand, nodes{pdx}, this.NN_method);
                        q_new               = node_SE2(this.Extend(q_rand, q_nearest, delta_l_step, delta_l, dist_method));
                        
                        if this.Env.ValidityCheck(q_new)
                            [q_neighs, q_neighs_costs]  = this.Near(q_new, nodes{pdx}, eps_neigh, 'KDTree_radius_sq_norm'); % KDTree_radius_forward_progress_sq_norm
                            q_min                       = this.ChoseParent(q_new, q_nearest, q_neighs, q_neighs_costs, dist_method);
                            q_new.parent                = q_min;
                            if this.IsDEBUG
                                this.edges = [this.edges; [q_new.pose, q_new.parent.pose]];
                            end
                            CLS_KDTree.insert(q_new, nodes{pdx}(1));
                            this.Rewire(q_new, q_min, nodes{pdx}(1), 'sq_norm', eps_neigh_rewire); % forward_progress_sq_norm

                            if q_new.pose(5) > s_max(pdx)
                                s_max(pdx) = q_new.pose(5);
                                stall_count(pdx) = 0;
                            else
                                stall_count(pdx) = stall_count(pdx) + 1;
                            end

                            if round(abs(q_new.pose(5) - this.s_f(pdx)), 4) < 1e-4 || stall_count(pdx) >= 2000 % stall count > 100 --> tried hard enough
                                Reached_Goal(pdx) = true;
                                Goals{pdx}        = q_new;
                            end
                            fprintf('Section %d: Looking at progress %.4f \t|s_max: %.4f \t| stall_count %d\n', pdx, q_new.pose(5), s_max(pdx), stall_count(pdx));
                        end
                        if this.IsDEBUG
                            record  = [record; pdx, s_max(pdx), stall_count(pdx)];
                        end
                        ite     = ite + 1;
                    end
                end
            end
%             end
            
            % draw tree
            if this.IsDEBUG
                for idx = 1:size(this.edges, 1)
                    line([this.edges(idx,1), this.edges(idx,6)], [this.edges(idx,2), this.edges(idx,7)], [this.edges(idx,5), this.edges(idx,10)], 'Color', '#E1701A', 'LineWidth', 1);
                end
                quiver([this.edges(:,1);this.edges(:,6)], [this.edges(:,2);this.edges(:,7)], [this.edges(:,3);this.edges(:,8)], [this.edges(:,4);this.edges(:,9)]);
                drawnow
            end
            
            for pdx = 1:length(Goals)
                if ~isempty(Goals{pdx})
                    q_end       = node_SE2.copy(Goals{pdx});
                    paths{pdx}  = q_end;
                    while ~isempty(q_end.parent)
                        next_parent = node_SE2.copy(q_end.parent);
                        paths{pdx}  = [next_parent; paths{pdx}];
                        q_end       = next_parent;
                    end
                end
            end
        end
        
        function Rewire(this, q_new, q_min, node, dist_method, r, varargin)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % called on the set q_neigh \ q_min which loops through the set
        % and changes the parents of vertices to q_new if that is a less costly path.
        % Input:
        % 1. pt:    Point of interest
        % 2. node:  the node to be examined [init: node(1)]
        % 3. varargin:
        %    - current_dim: Current dimension
        %    - min_dist:    Minimum distance
        %    - NN:          Nearest neighbour found at the moment
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if isempty(varargin)
                current_dim  = 1;
            else
                current_dim  = varargin{1};
            end
            n_dim = size(q_new.pose, 2);
            
            % Capture leaf node
            if isempty(node)
                return
            end
            
            % dist extending from q_new to node within a radius r
            dist = dist_metric.method(node, q_new, dist_method);
            if any(dist <= r)
                if all(round(node.pose - q_min.pose, 10) == 0) || all(round(node.pose - q_new.pose, 10) == 0) % check numerical error
                    % Exclude q_min, do nothing
                else
                    % node is a neighbour
                    % Check if extendable from q_new --> node
                    q_check = node_SE2(this.Extend(node, q_new, dist/5, dist, dist_method));
                    % If extendable and the cost from start --> q_new --> node
                    % is less than the cost from start to node
                    if all(round(q_check.pose - node.pose, 10) == 0) && dist + q_new.cost < node.cost
                        if this.IsDEBUG
                            check_A = [node.pose, node.parent.pose];
                            del_idx = find(sum(this.edges - check_A == 0, 2) == 10);
                            if isempty(del_idx)
                                check_B = [node.parent.pose, node.pose];
                                del_idx = find(sum(this.edges - check_B == 0, 2) == 10);
                            end
                            if ~isempty(del_idx)
                                this.edges(del_idx,:) = [];
                            end
                            this.edges  = [this.edges; [q_new.pose, node.parent.pose]];
                        end
                        node.parent = q_new; % change parent
                    end
                end
            end
            
            % Calculate next dimension
            next_dim = mod(current_dim, n_dim) + 1;
            
            % Visit subtree
            if q_new.pose(current_dim) < node.pose(current_dim)
                % Go left side
                this.Rewire(q_new, q_min, node.KDT_Lchild, dist_method, r, next_dim);
                if r >= abs(q_new.pose(current_dim) - node.pose(current_dim))
                	this.Rewire(q_new, q_min, node.KDT_Rchild, dist_method, r, next_dim);
                end
            else
                % Go right side
                this.Rewire(q_new, q_min, node.KDT_Rchild, dist_method, r, next_dim);
                if r >= abs(q_new.pose(current_dim) - node.pose(current_dim))
                    this.Rewire(q_new, q_min, node.KDT_Lchild, dist_method, r, next_dim);
                end
            end
        end
        
        function q_min = ChoseParent(this, q_new, q_nearest, q_neighs, q_neighs_costs, dist_method)
            %%
            q_nearest_neighs        = [q_nearest; q_neighs];
            q_nearest_neighs_costs  = [q_nearest.cost; q_neighs_costs];
            [min_cost, q_min_idx]   = min(q_nearest_neighs_costs + dist_metric.method(q_new, q_nearest_neighs, dist_method));
            q_min                   = q_nearest_neighs(q_min_idx);
            q_new.cost              = min_cost;
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
        if rand() < 0.01
            s_rand = s_f;
            IsGoal = true;
        else
            mean   = s_max;                 % Current furthest progress
            s_var  = 0.1*s_f;               % Variance
            sigma  = sqrt(s_var);           % Wiggle around Current furthest progress
            s_Norm = s_var*randn(1) + mean; % Normal distribution sampling
            
            s_rand = max(s_break, min(s_f, s_Norm));
            IsGoal = false;
        end
        
            % Find correspond nearest progress:
            Ts_idx                 = min(size(this.Env.task_T, 3), max(1, ceil(size(this.Env.task_T, 3)*s_rand)));
            T_s                    = this.Env.task_T(:, :, Ts_idx);
            [rand_pt, current_IRM] = this.Env.IRM.sample(T_s(1:3, 4), s_rand);
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
            while dist_metric.method(to_pt, new_pt, dist_method) > delta_l_step
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
        
        function [neighbours, costs] = Near(this, pos, nodes, r, method)
            if strcmp(method, 'KDTree_radius_sq_norm')
                [neighbours, costs] = CLS_KDTree.Near_Neighbour(pos, nodes(1), 'sq_norm', r);
                
            elseif strcmp(method, 'KDTree_radius_forward_progress_sq_norm')
                [neighbours, costs] = CLS_KDTree.Near_Neighbour(pos, nodes(1), 'forward_progress_sq_norm', r);
                
            end
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
                [NN_pt, NN_val]  = CLS_KDTree.Nearest_Neighbour(pos, nodes(1), 'forward_progress_sq_norm', 1);
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