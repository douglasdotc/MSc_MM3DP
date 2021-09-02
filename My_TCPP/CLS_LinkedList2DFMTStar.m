classdef CLS_2DFMTStar
    properties
        Env                                         % Environment that RRT* is exploring with
        sampling_intensity                          % Number of points that will sample in each IRM
        r_search
        s_f                                         % Ending progress (can be a number other than 1)
        max_trial                                   % maximum number of trials for a progress, skip this point of progress when reached
        break_pts                                   % break points of the whole task
        V                                           % Vertex Set
        E                                           % Edge Set
        V_unvisited                                 % The set maintains the nodes which have not been added to the tree
        V_open                                      % Keeps track of nodes which have already been added to
                                                    % the tree, it drops nodes that are not near enough to the edge of the 
                                                    % expanding tree to actually have any new connections made
        V_closed
                                                    
        NN_method = 'KDTree_radius_sq_norm';        % Nearest Neighbour search method
        IsDEBUG                                     % Debug Flag
    end
    
    methods
        function this = CLS_2DFMTStar(Env, sampling_intensity, r_search, max_trial, varargin)
        %% Initialize://////////////////////////////////////////////////////////////////////////////
            this.Env                = Env;
            this.sampling_intensity = sampling_intensity;
            this.r_search           = r_search;
            this.max_trial          = max_trial;
            this.IsDEBUG            = this.Env.IsDEBUG;
            if ~isempty(varargin)
                this.break_pts = varargin{1};
                this.s_f       = this.break_pts(2:end);
            else
                this.s_f       = max(this.Env.s);
            end
        end
        
        function [paths, ite] = FMT_Star(this)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % 
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            % 1. Sample, use break points to narrow down sampling
            % efforts
            % If progress proceed to any value in break_pts -->
            % adaptive sampling uptill a certain value n
            % Skip that point of progress if sampled > n until progress
            % where the sampling can be done < certain threshold -->
            % turn off adaptive sampling
            
            [paths, ite] = this.Forward_FMT_Star();
            
            starts = [];
            ends   = [];
            for d_idx = 1:length(paths)
                starts = [starts; paths{d_idx}(1).pose(5)];
                ends   = [ends; paths{d_idx}(end).pose(5)];
            end
%             starts = starts(2:end);
%             ends   = ends(1:end-1);
%             
%             [paths, ite] = this.Backward_FMT_Star(starts, ends, paths, ite);
            
            % Free memory
            this.X_open    = [];
            this.X_fringe  = {};
            this.nodes     = {};
            this.sects     = {};
                        
            %if this.IsDEBUG
            for pdx = 1:length(paths)
                for kdx = length(paths{pdx}):-1:2
                    line([paths{pdx}(kdx-1).pose(1), paths{pdx}(kdx).pose(1)], [paths{pdx}(kdx-1).pose(2), paths{pdx}(kdx).pose(2)], [0,0], 'Color', '#E1701A', 'LineWidth', 4);
                end
                POSES = this.Env.Extract_item(paths{pdx}, 'pose');
                quiver(POSES(:,1), POSES(:,2), POSES(:,3), POSES(:,4), 'Color', '#F7A440', 'LineWidth', 2);
            end
            %end
        end
        
        function scatter_plot(this, node)
            %%
            if isempty(node)
                return
            end
            scatter(node.pose(1), node.pose(2))
            this.scatter_plot(node.KDT_Lchild)
            this.scatter_plot(node.KDT_Rchild)
        end
        
        function [paths, ite] = Forward_FMT_Star(this)
            %%
            if ~isempty(this.break_pts)
                s_max = this.break_pts(1:end-1);        % Current furthest progress
            else
                s_max = min(this.Env.s);
            end
            
            % Init:
            x_init = this.TD_sample_pts(1);
            scatter(x_init.pose(1), x_init.pose(2), 'x')
            fprintf('Sampling...');
            this.V = this.TD_sample_pts(100);
%             for idx = 1:100
%                 this.V = [this.V; this.TD_sample_pts(1)];
%             end
            fprintf('done\n');
            
            poses = this.Env.Extract_item(this.V, 'pose');
            scatter(poses(:,1), poses(:,2))
            
            CLS_KDTree.insert(x_init, this.V);
            this.E = [];
            this.V_unvisited = node_SE2.deep_copy(this.V);
            CLS_KDTree.Delete(x_init, this.V_unvisited);
            
%             this.V              = [x_init; this.V];
%             this.E              = [];
%             this.V_unvisited    = this.V(2:end);

            this.V_open         = node_SE2.deep_copy(x_init);
            z                   = node_SE2.deep_copy(x_init);
                        
            % dist_method  = 'progress_sq_norm';
            dist_method  = 'sq_norm';
            delta_l      = 0.1;
            delta_l_step = delta_l/5;
            ite          = 1;
            sect_idx     = 1;
            trial        = 0;   % + 1 when cannot extend
            
            % Main loop
            while true % all(z.pose ~= goal.pose)
                V_open_new   = [];
                % link list stuff
                temp_V = node_SE2.deep_copy(this.V);
                CLS_KDTree.Delete(z, temp_V);
                [N_z, ~, ~] = this.Nearest_Neighbour(z, temp_V, this.NN_method, this.r_search);
                clear temp_V % free memory
                X_near = this.LIntersect(N_z, this.V_unvisited);
                
%                 [N_z, dists] = this.Near(this.V, z, this.r_search);
%                 X_near       = this.Intersect(N_z, this.V_unvisited);
                for ndx = 1:length(X_near)
                    x                   = X_near(ndx);
                    % link list stuff
                    temp_V = node_SE2.deep_copy(this.V);
                    CLS_KDTree.Delete(x, temp_V);
                    [N_x, ~, ~] = this.Nearest_Neighbour(x, temp_V, this.NN_method, this.r_search);
                    clear temp_V % free memory
                    Y_near = this.LIntersect(N_x, this.V_open);
                    
%                     [N_x, dists]        = this.Near(this.V, x, this.r_search);
%                     Y_near              = this.Intersect(N_x, this.V_open);
                    if ~isempty(Y_near)
                        [cost, y_min_idx]   = min(this.Env.Extract_item(Y_near, 'cost') + dist_metric.method(x, Y_near, dist_method));
                        y_min               = Y_near(y_min_idx);

                        if this.CollisionCheck(x)
                            this.E      = [this.E; [y_min.pose, x.pose]];
                            x.cost      = cost;
                            x.parent    = node_SE2.copy(y_min);
                            V_open_new  = [V_open_new; x];
                            % link list stuff
                            CLS_KDTree.Delete(x, this.V_unvisited);
                            
%                             this.V_unvisited = this.DeleteNode(this.V_unvisited, x);
                            
                            line([y_min.pose(1), x.pose(1)], [y_min.pose(2), x.pose(2)], [0,0], 'Color', '#316879', 'LineWidth', 2);
                            drawnow
                        end
                    end
                end
                
                % link list stuff
                for o_idx = 1:length(V_open_new)
                    CLS_KDTree.insert(V_open_new(o_idx), this.V_open);
                end
                CLS_KDTree.Delete(z, this.V_open);
                CLS_KDTree.insert(z, this.V_closed);
                
%                 this.V_open = [this.V_open; V_open_new];
%                 this.V_open = this.DeleteNode(this.V_open, z);
%                 this.V_closed = [this.V_closed; z];
                if isempty(this.V_open) || isempty(this.V_unvisited)
                    fprintf("Failure/Searching Exhausted\n")
                    break
                end
                
                % link list stuff
                [poses, costs] = node_SE2.ExtractData(this.V_open);
                [min_cost, min_cost_idx] = min(costs);
                [m_pt, m_val, ~] = this.Nearest_Neighbour(node_SE2(poses(min_cost_idx)), this.V_open, 'KDTree_sq_norm', 1);
                if m_val == 0
                    z = node_SE2.deep_copy(m_pt);
                end
%                 costs                    = this.Env.Extract_item(this.V_open, 'cost');
%                 [min_cost, min_cost_idx] = min(costs);
%                 z                        = this.V_open(min_cost_idx);
            end            
            
            if this.IsDEBUG % Draw tree:
                line([this.E(:,1), this.E(:,6)], [this.E(:,2), this.E(:,7)], [0,0], 'Color', '#316879', 'LineWidth', 2);
                quiver(this.E(:,6), this.E(:,7), this.E(:,8), this.E(:,9), 0.2, 'LineWidth', 2);
                drawnow
            end
            this.E = [];
            
            % Trace path
            prev  = node_SE2.copy(z);
            paths = prev;
            while ~isempty(prev.parent)
                next         = node_SE2.copy(prev.parent);
                paths = [next; paths];
                prev         = next;
            end
        end
        
        function m_nodes = DeleteNode(this, nodes, pt)
            %%
            for idx = 1:length(nodes)
                if pt.pose == nodes(idx).pose
                    nodes(idx) = [];
                    break
                end
            end
            m_nodes = nodes;
        end
        
        function [N_nodes, dists] = Near(this, nodes, pt, r)
            %%
            m_nodes = DeleteNode(this, nodes, pt);
            dists   = dist_metric.method(pt, m_nodes, 'sq_norm');
            N_nodes = m_nodes(dists <= r);
            dists   = dists(dists <= r);
        end
        
        function UpdateCost(this, x, node)
            %%
            if isempty(node)
                return
            end
            
            if all(x.pose == node.pose)
                node.cost = x.cost;
            else
                this.UpdateCost(x, node.KDT_Lchild);
                this.UpdateCost(x, node.KDT_Rchild);
            end
        end
        
        function inter_nodes = LIntersect(this, nodes, list)
            %%
            inter_nodes = [];
            for idx = 1:length(nodes)
                [m_pt, m_val, ~] = Nearest_Neighbour(this, nodes(idx), list, 'KDTree_sq_norm', 1);
                if m_val == 0
                    inter_nodes = [inter_nodes; m_pt];
                end
            end
        end
        function inter_nodes = Intersect(this, nodes, list)
            %%
            inter_nodes = [];
            poses_nodes = this.Env.Extract_item(nodes, 'pose');
            poses_list  = this.Env.Extract_item(list, 'pose');
            [~, ~, ib]  = intersect(poses_nodes, poses_list, 'row');
            inter_nodes = list(ib,:);
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
        
        function IsValid = CollisionCheck(this, node)
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
                
        function nodes = sample_pts(this, s, n, varargin)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Sample n points from the IRM corresponds to progress s
        % Inputs:
        % s:    progress, scalar
        % n:    Number of points to sample, scalar
        % Outputs:
        % nodes: An array of nodes with size nx1
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            
            % Find the transformation matrix correspond to the progress s:
            T_s         = this.Env.task_T(:, :, min(size(this.Env.task_T, 3), max(1, ceil(size(this.Env.task_T, 3)*s))));
            if ~isempty(varargin)
                nodes   = varargin{1};
            else
                nodes   = [];
            end
            
            for idx = 1:n % sample n pts
                [pt, ~] = this.Env.IRM.sample(T_s(1:3, 4), s);
                pt      = node_SE2(pt);
                while ~this.Env.ValidityCheck(pt)
                    [pt, ~] = this.Env.IRM.sample(T_s(1:3, 4), s);
                    pt      = node_SE2(pt);
                end
                
                % KD Tree
                if isempty(nodes)
                    nodes = pt;
                else
                    CLS_KDTree.insert(pt, nodes);
                end
            end
        end
        
        function [NN_pt, NN_val, NN_idx] = Nearest_Neighbour(this, pos, nodes, method, k)
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
                [NN_pt, NN_val]  = CLS_KDTree.Nearest_Neighbour(pos, nodes(1), 'sq_norm', k);
                NN_idx = NaN;
                
            elseif strcmp(method, 'KDTree_progress_sq_norm')
                [NN_pt, NN_val]  = CLS_KDTree.Nearest_Neighbour(pos, nodes(1), 'FMT_progress_sq_norm', k);
                NN_idx = NaN;
            elseif strcmp(method, 'KDTree_radius_sq_norm')
                NN_pt  = CLS_KDTree.Near_Neighbour(pos, nodes, 'sq_norm', k);
                NN_val = NaN;
                NN_idx = NaN;
            end
        end
    end
end