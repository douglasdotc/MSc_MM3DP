classdef CLS_KDRRTStar
    properties
        %%
        % Knowledge of task space:
        r_start                         % Starting point
        s_f                             % Ending progress (can be a number other than 1)
        NN_method                       % Nearest Neighbour method
        IsDEBUG                         % Flag: Run in debug mode
        
        % Knowledge of printing task:
        Printing_Poly                   % Printing Polygons
        Obstacles_Poly                  % Obstacle Polygons
        printing_path_checkpoints       % Printing path checkpoints
        printing_progress               % Printing progress
    end
    
    methods
        function [path, ite] = RRT_Star(this)
        %% //////////////////////////////////////////////////////////////////////////////////////////
        % Description:
        % RRT* algorithm begins by initializing a tree T = (V, E) with a
        % vertex at the initial configuration (i.e. V = {q_I}). 
        % At each iteration the RRT algorithm then performs the following steps:
        % 1. The planner samples a random state x_rand in the state space.
        % 2. The planner finds a state x_near that is already in the search tree and is closest 
        %    (based on the distance definition in the state space) to x_rand.
        % 3. The planner expands from x_near towards x_rand, until a state x_new is reached.
        % 4. Then new state x_new is added to the search tree.
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            % ***********************Search task***********************
            pt_start.pose             = this.r_start;                   % / Start node info
            pt_start.cost             = 0;                              % :
            pt_start.RRT_parentIdx    = 0;                              % :
            pt_start.KDTree_parentIdx = 0;                              % :
            pt_start.KDTree_LchildIdx = NaN;                            % :
            pt_start.KDTree_RchildIdx = NaN;                            % \ Start node info

            pt_goal.pose              = Inf*ones(size(this.r_start));   % Init End node info
            % ***********************Search task***********************
            
            figure(1);
            dist_method               = 'angle_diff_sq_norm';           % Method name for calculating distance
            max_iter                  = 1e4;                            % Max. iteration
            delta_l                   = 0.3;                            % RRT edge length
            delta_l_step              = delta_l/5;                      % RRT edge step size
            delta_relink_l            = 0.5;                            % Relink edge length
            delta_relink_l_step       = delta_relink_l/5;               % Relink edge step size
            s_max                     = 0;                              % Current furthest progress
            
            % Start node struct:
            nodes(1)                  = pt_start;
            ite                       = pt_start.RRT_parentIdx;
            GoalFound                 = false;
            while this.Reached_Goal(nodes, pt_goal, delta_l, this.NN_method) == 0 && ite <= max_iter
                % Initialize:
                NN_val      = Inf;
                pt_new.pose = [];
                pt_new.cost = Inf;
                
                while NN_val == Inf || pt_new.cost == Inf
                    % Generate random point:
                    IsCollide = true;
                    while IsCollide
                        [IsGoal, rand_pt, current_IRM] = Rand_Config(this, s_max);
                        IsCollide                      = IsPtCollide(this.Printing_Poly, this.Obstacles_Poly, rand_pt(1:2));
                    end
                    
                    if ~GoalFound && IsGoal
                        pt_goal.pose = rand_pt;
                        GoalFound    = true;
                        if this.IsDEBUG
                            scatter(pt_goal.pose(1), pt_goal.pose(2), 'filled', 'MarkerFaceColor', '#77AC30');
                        end
                    end
                    
                    % Find nearest point to rand_pt:
                    [NN_val, NN_idx] = this.Nearest_Neighbour(rand_pt, nodes, this.NN_method);
                    
                    if NN_val ~= Inf
                        % Find nearest point to rand_pt:
                        NN_pt       = nodes(NN_idx);
                        pt_new.pose = this.Extend(rand_pt, NN_pt, delta_l_step, delta_l, dist_method);
                        pt_new.cost = this.dist_metric(pt_new.pose, NN_pt, dist_method) + NN_pt.cost;
                    end
                    
                    if this.IsDEBUG
                        try
                            delete(current_IRM_plot);
                        catch
                        end
                        current_IRM_plot = fill(current_IRM(1,:), current_IRM(2,:), 'b', 'FaceAlpha', 0.2, 'EdgeColor',"none");
                    end
                end
                
                % Rewire:
                % Find all nodes within a radius of r with respect to the new point:
                nodes_new_node_dists = this.dist_metric(pt_new.pose, nodes, dist_method);
                nodes_inROI          = nodes(nodes_new_node_dists <= delta_relink_l);
                
                % Search all nodes in ROI to find paths with lower costs:
                % DON'T GO BACKWARD --> use angle_diff_sq_norm:
                [min_cost, min_idx]  = min(this.Extract_item(nodes_inROI, 'cost') + this.dist_metric(pt_new.pose, nodes_inROI, dist_method));
                if isempty(min_idx)
                    min_pt = NN_pt;
                else
                    min_pt = nodes_inROI(min_idx);
                end
                
                % !!? (add orientation quiver plot)(recurrsion heirarchy
                % smoothing)
                pt_new.pose = this.Extend(pt_new.pose, min_pt, delta_relink_l_step, delta_relink_l, dist_method);
                
                % Choose Parent:
                % Just search the exact index, no need angle difference --> KDTree_sq_norm:
                [~, pt_new.RRT_parentIdx] = this.Nearest_Neighbour(min_pt.pose, nodes, 'KDTree_sq_norm');
                
                if this.IsDEBUG
                    line([min_pt.pose(1), pt_new.pose(1)], [min_pt.pose(2), pt_new.pose(2)], [0, 0], 'Color', '#316879', 'LineWidth', 2);
                    drawnow
                    hold on
                    assert(pt_new.RRT_parentIdx ~= 0);
                end
                
                % build KD Tree relations:
                [pt_new, nodes] = this.KD_tree_insert(pt_new, nodes);
                nodes           = [nodes, pt_new];

                % Update current furthest progress:
                if pt_new.pose(5) > s_max
                    s_max = pt_new.pose(5);
                end
                ite = ite + 1;
            end
            
            [goal_nn_val, goal_nn_idx] = this.Nearest_Neighbour(pt_goal.pose, nodes, this.NN_method);

            % Search backwards from goal to start to find the optimal least cost path
            pt_goal.cost             = goal_nn_val + nodes(goal_nn_idx).cost;
            pt_goal.RRT_parentIdx    = goal_nn_idx;
            pt_goal.KDTree_parentIdx = 0;
            pt_goal.KDTree_LchildIdx = NaN;
            pt_goal.KDTree_RchildIdx = NaN;
            q_end                    = pt_goal;
            nodes                    = [nodes pt_goal];
            path                     = q_end;

            while q_end.RRT_parentIdx ~= 0
                start = q_end.RRT_parentIdx;
                q_end = nodes(start);
                path  = [q_end, path];
            end

            if this.IsDEBUG
                for idx = length(path):-1:2
                    line([path(idx-1).pose(1), path(idx).pose(1)], [path(idx-1).pose(2), path(idx).pose(2)], [0,0], 'Color', '#F47A60', 'LineWidth', 4);
                end
                POSES = this.Extract_item(path, 'pose');
                quiver(POSES(:,1), POSES(:,2), POSES(:,3), POSES(:,4), 0.5, 'LineWidth', 2);
            end
        end
        
        function [IsGoal, rand_pt, current_IRM] = Rand_Config(this, s_max)
        %% //////////////////////////////////////////////////////////////////////////////////////////
        % Description:
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
            s_var  = 0.03*this.s_f;         % Variance
            sigma  = sqrt(s_var);           % Wiggle around Current furthest progress
            s_Norm = sigma*randn(1) + mean; % Normal distribution sampling

            s_rand = max(0, min(this.s_f, s_Norm));
            
            % Find correspond nearest progress:
            [~,idx]                = min(abs(this.printing_progress - s_rand));
            [rand_pt, current_IRM] = Fake_sampleIRM(this.printing_path_checkpoints(idx,:), this.printing_progress(idx), this.IsDEBUG);
            
            if s_rand >= 1
                IsGoal = true;
            else
                IsGoal = false;
            end
        end
        

        function val = Reached_Goal(this, nodes, q_goal, delta_l, NN_method)
        %% //////////////////////////////////////////////////////////////////////////////////////////
        % Description:
        % Check if goal reached or not
        % Return boolean (val) where value true indicates goal point is reached.
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if strcmp(NN_method, 'sq_norm')
                val = any(this.dist_metric(q_goal.pose, nodes, NN_method) <= delta_l);

            elseif strcmp(NN_method, 'KDTree_angle_diff_sq_norm')
                [min_dist, ~] = this.KD_Nearest_Neighbour(q_goal.pose, nodes, 'angle_diff_sq_norm');
                val           = (min_dist <= delta_l);

            end
        end

        function dists = dist_metric(this, pos, nodes, method)
        %% //////////////////////////////////////////////////////////////////////////////////////////
        % Description:
        % A list of different methods to find distance between two set of points
        % Methods:
        % Euclidean distance: ||x_i - p||
        % Angle difference euclidean distance ||x_i - p|| if s_2 > s_1 else Inf
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            dists = [];
            if strcmp(method, 'sq_norm')
                for i = 1:length(nodes)
                    dists = [dists; nodes(i).pose - pos];
                end
                dists = vecnorm(dists, 2, 2);
                
            elseif strcmp(method, 'angle_diff_sq_norm')
                for i = 1:length(nodes)
                    % handle numerical error by rounding
                    s_1 = round(nodes(i).pose(5), 10);
                    s_2 = round(pos(5), 10);
                    if s_2 > s_1
                        dists = [dists; norm(nodes(i).pose - pos)];
                    else
                        dists = [dists; Inf];
                    end
                end
                
            end
        end
        
        function val = Extract_item(this, nodes, item)
        %% //////////////////////////////////////////////////////////////////////////////////////////
        % Description:
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
        
        function new_pt_pose = Extend(this, to_pt, from_pt, delta_l_step, delta_l, method)
        %% //////////////////////////////////////////////////////////////////////////////////////////
        % Description:
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            new_pt    = from_pt;
            line_step = ((to_pt - from_pt.pose).*delta_l_step); %/this.dist_metric(to_pt, from_pt, method);
            while this.dist_metric(to_pt, from_pt, method) > delta_l_step
                % from_pt x--->--->---x to_pt
                new_pt.pose = new_pt.pose + line_step;
                
                if this.IsDEBUG
                    assert(new_pt.pose(5) > from_pt.pose(5))
                end
                
                if ~PtInIRM(this.printing_path_checkpoints, this.printing_progress, new_pt.pose)...
                        || IsPtCollide(this.Printing_Poly, this.Obstacles_Poly, new_pt.pose(1:2))...
                        || this.dist_metric(new_pt.pose, from_pt, method) > delta_l
                    % Revert, save, return
                    new_pt.pose = new_pt.pose - line_step;
                    new_pt_pose = new_pt.pose;
                    return
                end
            end
            % sampled point is less then one step away:
            new_pt_pose = to_pt;
            
%             if NN_val > delta_l
%                 dist        = this.dist_metric(to_pt, from_pt, method);
%                 new_pt_pose = from_pt.pose + ((to_pt - from_pt.pose).*delta_l)/dist;
%             else
%                 new_pt_pose = to_pt;
%             end
        end
        
        function [NN_val, NN_idx] = Nearest_Neighbour(this, pos, nodes, method)
        %% //////////////////////////////////////////////////////////////////////////////////////////
        % Description:
        % Return information of the nearest neighbour of pos 
        % Inputs:
        % 1. pos: [x y cos(theta) sin(theta) t]
        % 2. neighbours: X_neigh \in R^(nx3)
        % Methods:
        % 1. Distance: ||x_i - p||
        % 2. KDTree
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if strcmp(method, 'sq_norm')
                dists            = this.dist_metric(pos, nodes, method);
                [NN_val, NN_idx] = min(dists);

            elseif strcmp(method, 'KDTree_sq_norm')
                [NN_val, NN_idx] = this.KD_Nearest_Neighbour(pos, nodes, 'sq_norm');
                
            elseif strcmp(method, 'KDTree_angle_diff_sq_norm')
                [NN_val, NN_idx] = this.KD_Nearest_Neighbour(pos, nodes, 'angle_diff_sq_norm');
            end
        end
        
        function [pt_new, nodes] = KD_tree_insert(this, pt_new, nodes, varargin)
        %% //////////////////////////////////////////////////////////////////////////////////////////
        % Description:
        % Recurrsively search for the the leaf in the KD tree where the new point (pt_new) fits.
        % When the place is found, define:
        % KDTree_LchildIdx OR KDTree_RchildIdx in parent node depends on condition
        % KDTree_parentIdx in the new node
        % KDTree_LchildIdx AND KDTree_RchildIdx in new node as NaN
        % 
        % Inputs:
        % 1. pt_new: the node being considered
        % 2. nodes: the storage of nodes
        % 3. varargin
        %    - childIdx:    Index of child node in the array nodes
        %    - LorR:        Indicates the direction (L / R) the parent node is point towards
        %    - parentIdx:   Index of parent node in the array nodes
        %    - current_dim: The dimension that the function is looking at
        % Output:
        % 1. pt_new: new node
        % 2. nodes: modified nodes array
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if isempty(varargin)
                childIdx    = 1;
                LorR        = 'NA';
                parentIdx   = nodes(1).KDTree_parentIdx;
                current_dim = 0;
            else
                childIdx    = varargin{1};
                LorR        = varargin{2};
                parentIdx   = varargin{3};
                current_dim = varargin{4};
            end

            n_dim = size(pt_new.pose, 2);
            if isnan(childIdx)
                % Update parent's child index:
                if strcmp(LorR, 'L')
                    nodes(parentIdx).KDTree_LchildIdx = length(nodes) + 1;

                elseif strcmp(LorR, 'R')
                    nodes(parentIdx).KDTree_RchildIdx = length(nodes) + 1;

                end

                % Update new nodes KD Tree indices:
                pt_new.KDTree_parentIdx = parentIdx;
                pt_new.KDTree_LchildIdx = NaN;
                pt_new.KDTree_RchildIdx = NaN;

            elseif pt_new.pose == nodes(childIdx).pose
                % Duplicated

            elseif pt_new.pose(current_dim + 1) < nodes(childIdx).pose(current_dim + 1)
                parentIdx       = childIdx;
                current_dim     = mod((current_dim + 1), n_dim);
                [pt_new, nodes] = this.KD_tree_insert(pt_new, nodes, nodes(childIdx).KDTree_LchildIdx, 'L', parentIdx, current_dim);

            elseif pt_new.pose(current_dim + 1) >= nodes(childIdx).pose(current_dim + 1)
                parentIdx       = childIdx;
                current_dim     = mod((current_dim + 1), n_dim);
                [pt_new, nodes] = this.KD_tree_insert(pt_new, nodes, nodes(childIdx).KDTree_RchildIdx, 'R', parentIdx, current_dim);

            end
        end
        
        function [min_dist, min_idx] = KD_Nearest_Neighbour(this, POI, nodes, dist_method, varargin)
        %% //////////////////////////////////////////////////////////////////////////////////////////
        % Description:
        % Input:
        % 1. POI: Point of interest
        % 2. nodes: RRT* array of nodes
        % 3. varargin:
        %    - current_dim: current dimension
        %    - childIdx: Either left or right index of KD tree. i.e. KDTree_LchildIdx, KDTree_RchildIdx
        %    - min_dist: minimum distance
        %    - min_idx: index of the entry in array nodes
        % Output:
        % 1. min_dist: minimum distance
        % 2. min_idx: index of the entry in array nodes
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if isempty(varargin)
                current_dim = 1;
                childIdx    = 1;
                min_dist    = inf;
                min_idx     = 0;
            else
                current_dim = varargin{1};
                childIdx    = varargin{2};
                min_dist    = varargin{3};
                min_idx     = varargin{4};
            end
            n_dim = size(POI, 2);

            % Capture leaf node
            if isnan(childIdx)
                return
            end

            % Found point nearer than the nearest:
            dist = this.dist_metric(POI, nodes(childIdx), dist_method);
            if dist < min_dist
                min_dist = dist;
                min_idx  = childIdx;
            end

             next_dim = mod(current_dim, n_dim)+1;
            % Visit subtrees:
            if POI(current_dim) < nodes(childIdx).pose(current_dim) % Left       
                [min_dist, min_idx] = this.KD_Nearest_Neighbour(POI, nodes, dist_method, next_dim, nodes(childIdx).KDTree_LchildIdx, min_dist, min_idx);                
                if min_dist >= abs(POI(current_dim) - nodes(childIdx).pose(current_dim))
                    [min_dist, min_idx] = this.KD_Nearest_Neighbour(POI, nodes, dist_method, next_dim, nodes(childIdx).KDTree_RchildIdx, min_dist, min_idx);                
                end
            else
                [min_dist, min_idx] = this.KD_Nearest_Neighbour(POI, nodes, dist_method, next_dim, nodes(childIdx).KDTree_RchildIdx, min_dist, min_idx);
                if min_dist >= abs(POI(current_dim) - nodes(childIdx).pose(current_dim))
                    [min_dist, min_idx] = this.KD_Nearest_Neighbour(POI, nodes, dist_method, next_dim, nodes(childIdx).KDTree_LchildIdx, min_dist, min_idx);
                end
            end
        end
    end
end