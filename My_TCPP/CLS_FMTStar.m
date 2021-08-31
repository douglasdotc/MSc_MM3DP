classdef CLS_FMTStar
    properties
        Env                                         % Environment that RRT* is exploring with
        sampling_intensity                          % Number of points that will sample in each IRM
        kNN_num
        s_f                                         % Ending progress (can be a number other than 1)
        max_trial                                   % maximum number of trials for a progress, skip this point of progress when reached
        break_pts                                   % break points of the whole task
        X_open    = [];                             % 
        X_fringe  = {};
        nodes     = {};
        sects     = {};
        NN_method = 'KDTree_progress_sq_norm';      % Nearest Neighbour search method
        IsDEBUG                                     % Debug Flag
    end
    
    methods
        function this = CLS_FMTStar(Env, sampling_intensity, kNN_num, max_trial, varargin)
        %% Initialize://////////////////////////////////////////////////////////////////////////////
            this.Env                = Env;
            this.sampling_intensity = sampling_intensity;
            this.kNN_num            = kNN_num;
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
        
        function [paths, ite] = Backward_FMT_Star(this, starts, ends, paths, ite)
            s_min
        end
        
        function [paths, ite] = Forward_FMT_Star(this)
            if ~isempty(this.break_pts)
                s_max = this.break_pts(1:end-1);        % Current furthest progress
            else
                s_max = min(this.Env.s);
            end
            
            % Init:
            this.nodes       = this.sample_pts(s_max, 1);
            this.X_fringe    = {};
            curr             = node_SE2.copy(this.nodes);
            draw_cache       = [];
            
            fprintf('Sampling...');
            for s_idx = 2:length(this.Env.s)
                this.X_open = this.sample_pts(this.Env.s(s_idx), this.sampling_intensity, this.X_open);
            end
            fprintf('done\n');
            
            dist_method  = 'progress_sq_norm';
            delta_l      = 0.1;
            delta_l_step = delta_l/5;
            ite          = 1;
            sect_idx     = 1;
            trial        = 0;   % + 1 when cannot extend
            
            % Main loop
            while true
                % Find k nearest neighbours
                [kNN_pt, kNN_val, kNN_idx] = this.Nearest_Neighbour(curr, this.X_open, this.NN_method, this.kNN_num);
                kNN_pt                     = kNN_pt(kNN_val ~= 0 & kNN_val ~= Inf);
                CanExtend                  = false(1, length(kNN_pt));
                for jdx = 1:length(kNN_pt)
                    % Collision check
                    if this.Env.ValidityCheck(kNN_pt(jdx))
                        % Extend curr to kNN_pt(idx)
                        new_pt_pose = Extend(this, kNN_pt(jdx), curr, delta_l_step, delta_l, dist_method);
                        % Add new_pt_pose to X_fringe if there is progress
                        % (can extend)
                        if round(new_pt_pose(5), 5) > round(curr.pose(5), 5)
                            CanExtend(jdx)      = true;
                            new_pt              = node_SE2(new_pt_pose);
                            new_pt.cost         = curr.cost + dist_metric.method(kNN_pt(jdx), curr, dist_method);
                            new_pt.parent       = curr;
                            this.X_fringe  = this.X_fringe_addNode(new_pt);
                        end

                        if all(new_pt.pose ~= kNN_pt(jdx).pose) % Cannot reach --> remove
                            % Remove kNN_pt(idx) from X_open
                            CLS_KDTree.Delete(kNN_pt(jdx), this.X_open);
                            if this.IsDEBUG
                                % Check if really removed:
                                [~, c_val, ~] = this.Nearest_Neighbour(kNN_pt(jdx), this.X_open, 'KDTree_sq_norm', 1);
                                assert(c_val ~= 0) % the nearest neighbour will not be itself if removed --> c_val != 0
                            end
                        end

                    else
                        % remove kNN_pt(idx) from X_open
                        CLS_KDTree.Delete(kNN_pt(jdx), this.X_open);

                        if this.IsDEBUG
                            % Check if really removed:
                            [~, c_val, ~] = this.Nearest_Neighbour(kNN_pt(jdx), this.X_open, 'KDTree_sq_norm', 1);
                            assert(c_val ~= 0) % the nearest neighbour will not be itself if removed --> c_val != 0
                        end
                    end

                    if this.IsDEBUG
                        if CanExtend(jdx)
                            draw_cache = [draw_cache; [curr.pose, new_pt.pose]];
                        end
                    end
                end

                if any(CanExtend)
                    % Update curr:
                    [curr, this.X_fringe] = this.X_fringe_removeMin(s_max, delta_l);
                    % Add new pt:
                    this.nodes  = [this.nodes; node_SE2.copy(curr)];
                    trial       = 0; % Reset trial
                    %if this.IsDEBUG
                        fprintf('Progress (Can Extend): %.4f\n', curr.pose(5));
                    %end
                else
                    % Adaptive Sampling:
                    AS_min = find(this.Env.s > curr.pose(5), 1); % Sample from progress + n
                    
                    for AS_idx = AS_min:min(AS_min + 30, length(this.Env.s))
                        this.X_open = this.sample_pts(this.Env.s(AS_idx), this.sampling_intensity, this.X_open);
                    end
                    
                    trial = trial + 1;
                    %if this.IsDEBUG
                        fprintf('Progress (Can NOT Extend): %.4f\n', curr.pose(5));
                    %end
                end

                if trial > this.max_trial
                    this.sects{sect_idx} = curr;
                    NS_idx               = find(this.Env.s > curr.pose(5), 1);
                    curr                 = this.sample_pts(this.Env.s(NS_idx), 1); % just sample one seems not enough
                    sect_idx             = sect_idx + 1;
                    fprintf('Decomposed, next section start at %.4f\n', curr.pose(5));
                end
                
                if curr.pose(5) > s_max
                    s_max = curr.pose(5);
                end
                ite = ite + 1;
                if round(curr.pose(5), 5) >= round(this.s_f, 5) % || isempty(this.X_fringe)
                    break;
                end
            end
            this.sects{sect_idx} = curr;
            
            
            if this.IsDEBUG % Draw tree:
                line([draw_cache(:,1), draw_cache(:,6)], [draw_cache(:,2), draw_cache(:,7)], [0,0], 'Color', '#316879', 'LineWidth', 2);
                quiver(draw_cache(:,6), draw_cache(:,7), draw_cache(:,8), draw_cache(:,9), 0.2, 'LineWidth', 2);
                drawnow
            end
            draw_cache = [];
            
            % Trace path
            for d_idx = 1:length(this.sects)
                prev         = node_SE2.copy(this.sects{d_idx});
                paths{d_idx} = prev;
                while ~isempty(prev.parent)
                    next         = node_SE2.copy(prev.parent);
                    paths{d_idx} = [next; paths{d_idx}];
                    prev         = next;
                end
            end
        end
        
        function X_fringe = X_fringe_addNode(this, new_node)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % 
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if isempty(this.X_fringe)
                X_fringe = [this.X_fringe; new_node];
            else
                costs    = this.Env.Extract_item(this.X_fringe, 'cost');
                idxs     = find(new_node.cost < costs);
                if isempty(idxs)
                    X_fringe = [this.X_fringe; new_node];
                else
                    split    = idxs(1);
                    X_fringe = [this.X_fringe(1:split - 1); node_SE2.copy(new_node); this.X_fringe(split:end)];
                end
            end
        end
        
        function [curr, X_fringe] = X_fringe_removeMin(this, s_max, delta_l)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % 
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            mean     = s_max;                 % Current furthest progress
            s_var    = 0.1*this.s_f;        % Variance
            sigma    = sqrt(s_var);           % Wiggle around Current furthest progress
            s_Norm   = sigma*randn(1) + mean; % Normal distribution sampling
            s_rand   = max(0, min(this.s_f, s_Norm));
            s_idx    = min(size(this.Env.s, 1), max(1, ceil(size(this.Env.s, 1)*s_rand)));
            
            poses              = this.Env.Extract_item(this.X_fringe, 'pose');
            selected_X_fringe  = this.X_fringe(abs(poses(:,5) - this.Env.s(s_idx)) <= delta_l); % sorted list
            if ~isempty(selected_X_fringe)
                curr = node_SE2.copy(selected_X_fringe(1));   % select the one with the lowest cost
            else
                [val, idx] = min(abs(poses(:,5) - this.Env.s(s_idx)));
                curr       = this.X_fringe(idx);
            end
            idx                = find(curr.pose == poses, 1);
            this.X_fringe(idx) = [];                                    % Delete the selected one in X_fringe
%             poses                               = this.Env.Extract_item(this.X_fringe, 'pose');
%             this.X_fringe(poses(:,5) < s_max)	= [];
%             curr                                = this.X_fringe(1);
%             this.X_fringe(1)                 	= [];
            X_fringe                            = this.X_fringe;
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
%                     nodes = [nodes; pt];
                else
                    CLS_KDTree.insert(pt, nodes);
%                     [isReached, isInserted] = CLS_KDTree.insert(pt, nodes(1));
%                     if isReached && isInserted
%                         nodes = [nodes; pt];
%                     end
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
            end
        end
    end
end