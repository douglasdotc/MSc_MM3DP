classdef CLS_2DFMTStar
    properties
        Env                                         % Environment that RRT* is exploring with
        sampling_intensity                          % Number of points that will sample in each IRM
        adaptive_sampling_intensity
        r_search
        r_search_original
        s_f                                         % Ending progress (can be a number other than 1)
        max_trials
        V                                           % Vertex Set
        E                                           % Edge Set
        V_unvisited                                 % The set maintains the nodes which have not been added to the tree
        V_open                                      % Keeps track of nodes which have already been added to
                                                    % the tree, it drops nodes that are not near enough to the edge of the 
                                                    % expanding tree to actually have any new connections made
        V_closed
                                                    
        NN_method = 'KDTree_radius_sq_norm';        % Nearest Neighbour search method
        IsDEBUG                                     % Debug Flag
        
        file_name
    end
    
    methods
        function this = CLS_2DFMTStar(Env, sampling_intensity, adaptive_sampling_intensity, r_search, max_trials, varargin)
        %% Initialize://////////////////////////////////////////////////////////////////////////////
            this.Env                         = Env;
            this.sampling_intensity          = sampling_intensity;
            this.adaptive_sampling_intensity = adaptive_sampling_intensity;
            this.r_search                    = r_search;
            this.r_search_original           = r_search;
            this.s_f                         = max(this.Env.s);
            this.max_trials                  = max_trials;
            this.IsDEBUG                     = this.Env.IsDEBUG;
            if ~isempty(varargin)
                this.file_name = varargin{1};
            else
                this.file_name = "";
            end
        end
        
        function [paths, ite, total_cost, time, record] = FMT_Star(this)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % 
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            total_cost      = 0;
            num_path_nodes  = [];
            
            ax1 = figure(1); %
            this.Env.task_OBJ.plot();
            box on
            hold on
            xlabel('x (m)')
            ylabel('y (m)')
            for jdx = 1:length(this.Env.obstacles_break)
                Obstacles_Poly{jdx} = polyshape(this.Env.obstacles_break{jdx});
                plot(Obstacles_Poly{jdx}, 'FaceColor', 'y')
                hold on
            end
            drawnow
            tic
                [paths, ite, sampling_time, record, rec_edges] = this.Forward_FMT_Star();
            time = toc;
            if this.IsDEBUG % Draw tree:
                if ~isempty(rec_edges)
                    for edx = 1:length(rec_edges)
                        for eedx = 1:size(rec_edges{edx},1)
                            line([rec_edges{edx}(eedx,1), rec_edges{edx}(eedx,6)], [rec_edges{edx}(eedx,2), rec_edges{edx}(eedx,7)], [rec_edges{edx}(eedx,5), rec_edges{edx}(eedx,10)], 'Color', '#E1701A', 'LineWidth', 0.5);
                        end
                        quiver(rec_edges{edx}(:,6), rec_edges{edx}(:,7), rec_edges{edx}(:,8), rec_edges{edx}(:,9), 0.2, 'LineWidth', 2);
                        drawnow
                    end
                end
            end
            if this.IsDEBUG
                for pdx = 1:length(paths)
                    for kdx = length(paths{pdx}):-1:2
                        line([paths{pdx}(kdx-1).pose(1), paths{pdx}(kdx).pose(1)], [paths{pdx}(kdx-1).pose(2), paths{pdx}(kdx).pose(2)], [0,0], 'Color', '#316879', 'LineWidth', 4);
                    end
                    POSES = this.Env.Extract_item(paths{pdx}, 'pose');
                    quiver(POSES(:,1), POSES(:,2), POSES(:,3), POSES(:,4), 0.5, 'Color', '#316879', 'LineWidth', 2);
                    
                    num_path_nodes  = [num_path_nodes, length(paths{pdx})];
                    total_cost      = total_cost + paths{pdx}(end).cost;
                end
            end
            sum_path_nodes = sum(num_path_nodes);
            fprintf("FMT* search done in %.4f sec, %d iterations with path cost %.4f and %d total number of path nodes.\n", time, ite, total_cost, sum_path_nodes);
            fprintf("number of path nodes break down:\n");
            disp(num_path_nodes)
            
            % Save data
            writematrix(num_path_nodes,this.file_name+"path_cost_break_down.txt",'Delimiter',',')
            fileID = fopen(this.file_name+".txt",'w');
            fprintf(fileID, "Total time: %.4f\n", time);
            fprintf(fileID, "Sampling time: %.4f\n", sampling_time);
            fprintf(fileID, "Iterations %d\n", ite);
            fprintf(fileID, "Path cost %.4f\n", total_cost);
            fprintf(fileID, "Total number of nodes: %d\n", sum_path_nodes);
            fclose(fileID);
            save(this.file_name+".mat");
            saveas(ax1, this.file_name+".fig")
            hold off %
            delete(ax1); %
            
            ax1 = figure(1); %
            this.Env.task_OBJ.plot();
            box on
            hold on
            xlabel('x (m)')
            ylabel('y (m)')
            for jdx = 1:length(this.Env.obstacles_break)
                Obstacles_Poly{jdx} = polyshape(this.Env.obstacles_break{jdx});
                plot(Obstacles_Poly{jdx}, 'FaceColor', 'y')
                hold on
            end
            if this.IsDEBUG
                for pdx = 1:length(paths)
                    for kdx = length(paths{pdx}):-1:2
                        line([paths{pdx}(kdx-1).pose(1), paths{pdx}(kdx).pose(1)], [paths{pdx}(kdx-1).pose(2), paths{pdx}(kdx).pose(2)], [0,0], 'Color', '#316879', 'LineWidth', 4);
                    end
                    POSES = this.Env.Extract_item(paths{pdx}, 'pose');
                    quiver(POSES(:,1), POSES(:,2), POSES(:,3), POSES(:,4), 0.5, 'Color', '#316879', 'LineWidth', 2);
                end
                drawnow
            end
            saveas(ax1, this.file_name+"_Path_only.fig")
            hold off %
            delete(ax1); %
        end
        
        function [paths, ite, sampling_time, record, rec_edges] = Forward_FMT_Star(this)
            %%
            % Init:
            IsDifficultRegion_init = true;
            while IsDifficultRegion_init % Must not collide with obstacles
                [x_init, IsDifficultRegion_init] = this.sample_pts(this.Env.s(1), 1);
            end
            
            fprintf('Sampling...\n');
            tic
            for s_idx = 1:length(this.Env.s)
                s = this.Env.s(s_idx);
                fprintf('Sampling progress: %.4f\n', s);
                for idx = 1:this.sampling_intensity
                    [pt, IsDifficultRegion] = this.sample_pts(s, 1);
                    this.V = [this.V; pt];
                end
                if IsDifficultRegion % If found colliding with obstacles, try sample more
                    fprintf('Encounter a region with obstacles, trying to sample more...\n');
                    for jdx = 1:this.adaptive_sampling_intensity
                        [pt, ~] = this.sample_pts(s, 1);
                        this.V = [this.V; pt];
                    end
                end
            end
            sampling_time = toc;
            fprintf('done\n');
            
            path_store          = [];
            paths               = {};
            this.V              = [x_init; this.V];
            temp_E              = [];
            this.E              = {};
            this.V_unvisited    = this.V(2:end);
            this.V_open         = node_SE2.deep_copy(x_init);
            z                   = node_SE2.deep_copy(x_init);
                        
            dist_method         = 'forward_progress_sq_norm'; % 'sq_norm';
            s_max               = min(this.Env.s);
            ite                 = 1;
            stall_count         = 0;
            record              = [];
            
            % Main loop
            while ~all(abs(z.pose(5) - this.s_f) < 1e-5)
                V_open_new                  = [];
                [N_z, dists, this.r_search] = this.Near(this.V, z, this.r_search, 'backward_progress_sq_norm');
                X_near                      = this.Intersect(N_z, this.V_unvisited);
                for ndx = 1:length(X_near)
                    x                           = X_near(ndx);
                    [N_x, dists, this.r_search] = this.Near(this.V, x, this.r_search, dist_method);
                    Y_near                      = this.Intersect(N_x, this.V_open);
                    if ~isempty(Y_near)
                        [cost, y_min_idx]   = min(this.Env.Extract_item(Y_near, 'cost') + dist_metric.method(x, Y_near, dist_method));
                        y_min               = Y_near(y_min_idx);
                        % Extend from y_min --> x
                        xymin_dist          = dist_metric.method(x, y_min, dist_method);
                        new_pt_pose         = this.Extend(x, y_min, xymin_dist/(this.r_search*10), xymin_dist, dist_method);
                        new_pt              = node_SE2(new_pt_pose);
                        if ~all(round(new_pt.pose - x.pose, 10)) % if all == 0 means it can extend --> boolean = 1
                            temp_E              = [temp_E; [y_min.pose, new_pt.pose]];
                            x.cost              = y_min.cost + dist_metric.method(x, y_min, dist_method); % cost;
                            assert(x.cost ~= Inf)
                            x.parent            = node_SE2.copy(y_min);
                            V_open_new          = [V_open_new; x];
                            this.V_unvisited    = this.DeleteNode(this.V_unvisited, x);
                        end
                    else
                        fprintf("Y_near is empty\n")
                    end
                end
                
                this.V_open     = [this.V_open; V_open_new];
                this.V_open     = this.DeleteNode(this.V_open, z);
                this.V_closed   = [this.V_closed; z];
                
                if isempty(this.V_unvisited)
                    fprintf("V_unvisited is empty!\n")
                    break
                end
                if isempty(this.V_open)
                    fprintf('Progress: %.4f | V_open size: %d \t| V_closed size: %d \t | V_unvisited size: %d \t | r_search %.4f \t | stall_count %d \t| s_max %.4f\n', z.pose(5), length(this.V_open), length(this.V_closed), length(this.V_unvisited), this.r_search, stall_count, s_max);
                    fprintf("V_open is empty, search for new start in V_unvisited...\n")
                    
                    % Enquire V_unvisited
                    temp_poses  = this.Env.Extract_item(this.V_closed, 'pose');
                    p           = this.V_closed(find(round(temp_poses(:,5) - s_max, 10) == 0, 1));
                    
                    % If p is a path
                    if ~isempty(p.parent) % At least have two nodes
                        path_store      = [path_store; p]; % store section of path
                        this.E{end + 1} = temp_E;
                        temp_E          = [];
                    end
                    
                    % Update V_open and z
                    poses           = this.Env.Extract_item(this.V_unvisited, 'pose');
                    un_idx          = find(poses(:,5) > s_max, 1);
                    this.V_open     = node_SE2.deep_copy(this.V_unvisited(un_idx));
                    z               = node_SE2.deep_copy(this.V_unvisited(un_idx));
                    
                    % Record:
                    record = [record; length(this.V_open), length(this.V_closed), length(this.V_unvisited), this.r_search, stall_count, s_max];
                else
                    fprintf('Progress: %.4f | V_open size: %d \t| V_closed size: %d \t | V_unvisited size: %d \t | r_search %.4f \t | stall_count %d \t| s_max %.4f\n', z.pose(5), length(this.V_open), length(this.V_closed), length(this.V_unvisited), this.r_search, stall_count, s_max);
                    z = this.Config(s_max);
                    
                    % Record:
                    record = [record; length(this.V_open), length(this.V_closed), length(this.V_unvisited), this.r_search, stall_count, s_max];
                end
                
                if z.pose(5) > s_max
                    s_max       = z.pose(5);
                    stall_count = 1;
                else
                    stall_count = stall_count + 1;
                end
                
                if this.r_search > this.r_search_original
                    this.r_search = this.r_search_original;
                end
                ite = ite + 1;
            end
            
            if ~isempty(z.parent)
                path_store      = [path_store; z]; % store the last section
                this.E{end + 1} = temp_E;
            end
            rec_edges = this.E;
            
            % Trace path
            for pdx = 1:length(path_store)
                prev       = node_SE2.copy(path_store(pdx));
                paths{pdx} = prev;
                while ~isempty(prev.parent)
                    next         = node_SE2.copy(prev.parent);
                    paths{pdx}   = [next; paths{pdx}];
                    prev         = next;
                end
            end
        end
        
        function z = Config(this, s_max)
            %% push forward
            % choose a progress max at s_max, choosing a progress > s_max will intentionally break the path.
            % choose a random progress max at s_max then choose the node with minimum cost
            costs                    = this.Env.Extract_item(this.V_open, 'cost');
            [min_cost, min_cost_idx] = min(costs);
            z                        = this.V_open(min_cost_idx);
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
        
        function [N_nodes, dists, r] = Near(this, nodes, pt, r, dist_method)
            %%
            r_inc   = 0.01;
            m_nodes = this.DeleteNode(nodes, pt);
            dists   = dist_metric.method(pt, m_nodes, dist_method);
            N_nodes = m_nodes(dists <= r);
            while isempty(N_nodes)
                r       = r + r_inc;
                N_nodes = m_nodes(dists <= r);
            end
            dists   = dists(dists <= r);
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
                
        function [nodes, IsDifficultRegion] = sample_pts(this, s, n, varargin)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Sample n points from the IRM corresponds to progress s
        % Inputs:
        % s:    progress, scalar
        % n:    Number of points to sample, scalar
        % Outputs:
        % nodes: An array of nodes with size nx1
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if ~isempty(varargin)
                nodes   = varargin{1};
            else
                nodes   = [];
            end
            
            % Find the transformation matrix correspond to the progress s:
            T_s               = this.Env.task_T(:, :, min(size(this.Env.task_T, 3), max(1, ceil(size(this.Env.task_T, 3)*s))));
            IsDifficultRegion = false;
            for idx = 1:n % sample n pts
                trial                 = 0;
                IsInIRM_and_TaskValid = false;
                
                while ~IsInIRM_and_TaskValid || trial <= this.max_trials
                    [pt, ~] = this.Env.IRM.sample(T_s(1:3, 4), s);
                    pt      = node_SE2(pt);
                    IsInIRM_and_TaskValid = this.Env.isInIRM(pt) && this.Env.CheckCollisionWithTask(pt);
                    if IsInIRM_and_TaskValid
                        if ~this.Env.CheckCollisionWithObstacles(pt)
                            trial = trial + 1; % Collide with obstacles
                        else
                            break; % No collision with obstacles
                        end
                    end
                end
                
                if trial >= this.max_trials % Tried so many times 
                    IsDifficultRegion = true; % Notify FMT* to sample more here.
                end
                % KD Tree
                if isempty(nodes)
                    nodes = pt;
                else
                    CLS_KDTree.insert(pt, nodes);
                end
            end
        end
    end
end