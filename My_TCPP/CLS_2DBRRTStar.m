classdef CLS_2DBRRTStar
    properties
        Env                                         % Environment that RRT* is exploring with
        start_nodes                                 % n sampled starting nodes with structure node_SE2
        edges
        NN_method = 'KDTree_sq_norm';
%         NN_method = 'KDTree_progress_sq_norm';      % Nearest Neighbour search method
        s_f                                         % Ending progress (can be a number other than 1)
        break_pts                                   % break points of the whole task
        cost_best  = Inf;
        sigma_best = [];
        IsDEBUG                                     % Debug Flag
    end
    
    methods
        function this = CLS_2DBRRTStar(Env, start_nodes, varargin)
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
        
        function [path, ite] = BRRT_Star(this)
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
%             nodes                     = this.start_nodes;               % Init start nodes
            tree_forward              = this.TD_sample_pts(1);            % TEMP FOR TEST !!! Init start nodes
            tree_backward             = this.TD_sample_pts(1);
            scatter(tree_forward.pose(1), tree_forward.pose(2))
            scatter(tree_backward.pose(1), tree_backward.pose(2))
            drawnow
            num_paths                 = length(tree_forward);                  % Number of path exploring simutaneously
            % ***********************Search task ***********************
            
            % *********************** Parameters ***********************
%             dist_method               = 'forward_progress_sq_norm';     % Method name for calculating distance
            dist_method = 'sq_norm';
            max_iter                  = 1e4;                            % Max. iteration
            delta_l                   = 0.1;                            % RRT edge length
            delta_l_step              = delta_l/5;                      % RRT edge step size
            eps_neigh                 = 0.2;
            eps_neigh_rewire          = 0.2;
            
            if ~isempty(this.break_pts)
                s_max                 = this.break_pts(1:end-1);        % Current furthest progress
            else
                s_max                 = 0;
            end
            % *********************** Parameters ***********************
            
            ite                       = 1;
            Reached_Goal              = false(length(tree_forward), 1);
            
            while ~all(Reached_Goal)
                %% Main loop RRT* routine
                q_rand = this.TD_sample_pts(1); % no goal for now
%                 [~, q_rand, ~]      = this.Rand_Config(0, s_max, this.s_f);
%                 q_rand              = node_SE2(q_rand);
                
                [q_nearest, ~, ~]   = this.Nearest_Neighbour(q_rand, tree_forward, this.NN_method);
%                 q_new               = node_SE2(this.Extend(q_rand, q_nearest, delta_l_step, delta_l, dist_method));
                q_new               = node_SE2(this.TD_Extend(q_rand, q_nearest, delta_l_step, delta_l, dist_method));
                
                % if this.Env.ValidityCheck(q_new)
                if this.CollisionCheck(q_new)
                    [q_neighs, q_neighs_costs]  = this.Near(q_new, tree_forward, eps_neigh, 'KDTree_radius_sq_norm'); % KDTree_radius_forward_progress_sq_norm
                    q_min                       = this.ChooseParent(q_new, q_nearest, q_neighs, q_neighs_costs, dist_method);
                    q_new.parent                = q_min;
                    this.edges                  = [this.edges; [q_new.pose, q_new.parent.pose]];
                    CLS_KDTree.insert(q_new, tree_forward(1));
                    this.Rewire(q_new, q_min, tree_forward(1), 'sq_norm', eps_neigh_rewire); % forward_progress_sq_norm
                    
                    % Bidirectional routine
                    % q_connect <- NearestVertex(q_new, tree_backward)
                    % Find the nearest neighbour of q_new in forward tree
                    % in the backward tree
                    [q_connect, ~, ~]                 = this.Nearest_Neighbour(q_new, tree_backward, this.NN_method);
                    % edge_new  <- Connect(q_new, q_connect, tree_backward)
                    % Try to connect q_new in the forward tree to q_connect
                    % in the backward tree: q_new --> q_connect
                    [sigma_sol, cost_sol, this.edges] = this.Connect(q_new, q_connect, tree_backward, delta_l_step, delta_l, eps_neigh, dist_method);
                    if cost_sol < this.cost_best
                        this.sigma_best = sigma_sol;
                        this.cost_best  = cost_sol;
                    end
                    % SwapTrees(tree_forward, tree_backward)
                    [tree_forward, tree_backward] = this.SwapTrees(tree_forward, tree_backward);
                    
                    % Task consistency stuff
                    if q_new.pose(5) > s_max
                        s_max = q_new.pose(5);
                    end
                    if ~isempty(this.sigma_best)
                        Reached_Goal(1) = true;
                    end
                    fprintf('Looking at progress: %.4f, s_max: %.4f\n', q_new.pose(5), s_max);
                end
                ite = ite + 1;
            end
            
            % draw tree
            for idx = 1:size(this.edges, 1)
                line([this.edges(idx,1), this.edges(idx,6)], [this.edges(idx,2), this.edges(idx,7)], [this.edges(idx,5), this.edges(idx,10)], 'Color', '#316879', 'LineWidth', 1);
            end
            quiver([this.edges(:,1);this.edges(:,6)], [this.edges(:,2);this.edges(:,7)], [this.edges(:,3);this.edges(:,8)], [this.edges(:,4);this.edges(:,9)]);
            drawnow
            
            path = this.sigma_best;
            for kdx = length(path):-1:2
                line([path(kdx-1).pose(1), path(kdx).pose(1)], [path(kdx-1).pose(2), path(kdx).pose(2)], [0,0], 'Color', '#E1701A', 'LineWidth', 4);
            end
            POSES = this.Env.Extract_item(path, 'pose');
            quiver(POSES(:,1), POSES(:,2), POSES(:,3), POSES(:,4), 'Color', '#F7A440', 'LineWidth', 2);
            drawnow
        end
        
        function [tree_forward, tree_backward] = SwapTrees(this, tree_forward, tree_backward)
            %%
            temp_tree       = tree_backward;
            tree_backward   = tree_forward;
            tree_forward    = temp_tree;
        end
        
        function [sigma_sol, cost_sol, edges] = Connect(this, q_new, q_connect, tree_backward, delta_l_step, delta_l, eps_neigh, dist_method)
            %%
            % x_new is a point between q_new (T_a) --> q_connect (T_b)
            x_new         = node_SE2(this.TD_Extend(q_connect, q_new, delta_l_step, delta_l, 'sq_norm'));
            % Find the neighbours around x_new (Extend from q_new) in T_b
            [x_neighs, ~] = this.Near(x_new, tree_backward, eps_neigh, 'KDTree_radius_sq_norm');
            
            X_near_no_collision = []; % store involved x_near
            C_near_no_collision = []; % store the cost of the paths
            % for every neighbour calculate the cost of start --> q_new --> x_neigh --> end
            for idx = 1:length(x_neighs)
                % Try to extend from q_new to x_neigh
                dist_q_new_x_neigh  = dist_metric.method(x_neighs(idx), q_new, 'sq_norm');
                x_neigh_check       = node_SE2(this.TD_Extend(x_neighs(idx), q_new, dist_q_new_x_neigh/5, dist_q_new_x_neigh, 'sq_norm'));
                if all(round(x_neigh_check.pose - x_neighs(idx).pose, 10) == 0) % can extend
                    % path cost
                    c_near              = q_new.cost + dist_q_new_x_neigh + x_neighs(idx).cost;
                    X_near_no_collision = [X_near_no_collision; x_neighs(idx)];
                    C_near_no_collision = [C_near_no_collision; c_near];
                end
            end
            [C_near_no_collision, sorting_idxs] = sort(C_near_no_collision);
            X_near_no_collision                 = X_near_no_collision(sorting_idxs);
            
            % Choose a parent in x_nieghs with minimum cost for q_new
            for jdx = 1:length(X_near_no_collision)
                if C_near_no_collision(jdx) < this.cost_best
                    edges       = [this.edges; [q_new.pose, X_near_no_collision(jdx).pose]];
                    cost_sol    = C_near_no_collision(jdx);
                    % returns a path linking start --> q_new --> x_neigh --> end
                    sigma_sol   = this.GeneratePath(q_new, X_near_no_collision(jdx));
                    return
                end
            end
            edges     = this.edges;
            sigma_sol = [];
            cost_sol  = Inf;
        end
        
        function sigma_sol = GeneratePath(this, q_new, x_neigh)
            %%
            path_A = []; % path from start to q_new
            path_B = []; % path from goal to x_neigh
            
            % For path_A
            q_end  = node_SE2.copy(q_new);
            path_A = q_end;
            while ~isempty(q_end.parent)
                next_parent = node_SE2.copy(q_end.parent);
                path_A      = [next_parent; path_A];
                q_end       = next_parent;
            end
            
            % For path_B
            q_end  = node_SE2.copy(x_neigh);
            path_B = q_end;
            while ~isempty(q_end.parent)
                next_parent = node_SE2.copy(q_end.parent);
                path_B      = [next_parent; path_B];
                q_end       = next_parent;
            end
            
            sigma_sol = [path_A; flip(path_B)];
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
                    q_check = node_SE2(this.TD_Extend(node, q_new, dist/5, dist, dist_method));
                    % If extendable and the cost from start --> q_new --> node
                    % is less than the cost from start to node
                    if all(round(q_check.pose - node.pose, 10) == 0) && dist + q_new.cost < node.cost
                        check_A = [node.pose, node.parent.pose];
                        del_idx = find(sum(this.edges - check_A == 0, 2) == 10);
                        if isempty(del_idx)
                            check_B = [node.parent.pose, node.pose];
                            del_idx = find(sum(this.edges - check_B == 0, 2) == 10);
                        end
                        if ~isempty(del_idx)
                            this.edges(del_idx,:) = [];
                        end
                        node.parent = q_new; % change parent
                        this.edges  = [this.edges; [q_new.pose, node.parent.pose]];
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
        
        function q_min = ChooseParent(this, q_new, q_nearest, q_neighs, q_neighs_costs, dist_method)
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
            s_Norm = sigma*randn(1) + mean; % Normal distribution sampling
            
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