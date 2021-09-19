classdef CLS_ENV_SE2 < handle
    properties
        task_OBJ        % task
        task_T          % Transformation matrices 4x4xn
        s               % Task progress
        task_coord      % Task coordinates nx3
        robot_hitbox    % robot hit box
        IRM             % Inverse Reachability Map
        obstacles       % Obstacles
        obstacles_break
        IsDEBUG         % Debug flag
%         x_lim           % x-boundary of environment
%         y_lim           % y-boundary of environment
    end
    
    methods
        function this = CLS_ENV_SE2(task, task_T, s, robot_hitbox, IRM, obstacles, obstacles_break, IsDEBUG)
        %% Initialize://///////////////////////////////////////////////////////////////////////////
            this.task_OBJ        = task;
            this.task_T          = task_T;
            this.s               = s;
            this.task_coord      = squeeze(this.task_T(1:2, 4, :))';
            this.robot_hitbox    = robot_hitbox;
            this.IRM             = IRM;
            this.obstacles       = obstacles;
            this.obstacles_break = obstacles_break;
            this.IsDEBUG         = IsDEBUG;
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
            Robot_Poly_x = Robot_Poly(:,1);
            Robot_Poly_y = Robot_Poly(:,2);
            
            IsValid = true;
            % Check if robot collide with obstacles
            if ~isempty(this.obstacles)
                obss = vertcat(this.obstacles{:});
                xq = obss(:,1);
                yq = obss(:,2);
                IsCollide = any(inpolygon(xq, yq, Robot_Poly_x, Robot_Poly_y));
                % IsCollide = IsCollide || any(inpolygon(Robot_Poly_x, Robot_Poly_y, xq, yq));
                if IsCollide
                    if this.IsDEBUG
                        %disp("Robot collide with obstacles")
                    end
                    IsValid = false;
                end
            end
            
%             for o_idx = 1:length(this.obstacles)
%                 IsCollide = any(inpolygon(this.obstacles{o_idx}(:,1), this.obstacles{o_idx}(:,2), Robot_Poly_x, Robot_Poly_y));
%                 Iscollide = IsCollide || any(inpolygon(Robot_Poly_x, Robot_Poly_y, this.obstacles{o_idx}(:,1), this.obstacles{o_idx}(:,2)));
%                 
%                 if Iscollide
%                     if this.IsDEBUG
%                         %disp("Robot collide with obstacles")
%                     end
%                     IsValid = false;
%                     return;
%                 end
%             end
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
            
            que_x       = this.task_coord(1:s_idx, 1);
            que_y       = this.task_coord(1:s_idx, 2);
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
        
        function break_pts = Breakpoints_IRM_obs(this, IRM_overlap_threshold)
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
            for o_idx = 1:length(this.obstacles_break)
                for vertex_idx = 1:size(this.obstacles_break{o_idx}, 1)
                    next_vertex_idx = vertex_idx + 1;
                    if next_vertex_idx > size(this.obstacles_break{o_idx}, 1)
                        next_vertex_idx = 1;
                    end
                    [dists, ~] = dist_metric.point_to_line(this.task_coord, this.obstacles_break{o_idx}(vertex_idx,:), this.obstacles_break{o_idx}(next_vertex_idx,:));
                    if any(dists <= this.IRM.r_max)
                        edges = [edges; this.obstacles_break{o_idx}(vertex_idx,:), this.obstacles_break{o_idx}(next_vertex_idx,:), o_idx];
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