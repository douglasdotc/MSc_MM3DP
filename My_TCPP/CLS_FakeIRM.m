classdef CLS_FakeIRM
    properties
        min_task_robot_dist = 0.15;         % Minimum distance between robot and task
        angle               = 0:pi/50:2*pi; % Pre-allocate for calculation
        angle_min           = -pi;          % Min angle
        angle_max           = pi;           % Max angle
        res                 = 0.1;          % resolution of IRM_checking_pts
        r_min                               % Inner radius of IRM
        r_max                               % Outer radius of IRM
        inner_circle                        % 2xn matrix of point describing the inner circle of IRM
        outer_circle                        % 2xn matrix of point describing the outer circle of IRM
        IRM_poly                            % IRM polyshape()
        IRM_checking_pts                    % points in IRM used to check approachability (Breakpoints)
        IsDEBUG                             % Debug flag
    end
    
    methods
        function this = CLS_FakeIRM(min_task_robot_dist, IsDEBUG)
        %% Initialize://///////////////////////////////////////////////////////////////////////////
            this.min_task_robot_dist = min_task_robot_dist;
            this.r_min               = min_task_robot_dist + 0.05;
            this.r_max               = this.r_min + min_task_robot_dist*4;
            this.inner_circle        = [this.r_min*cos(this.angle);
                                        this.r_min*sin(this.angle)];
            this.outer_circle        = [this.r_max*cos(this.angle);
                                        this.r_max*sin(this.angle)];
            this.IRM_poly            = polyshape([this.inner_circle(1,:), this.outer_circle(1,:);
                                                  this.inner_circle(2,:), this.outer_circle(2,:)]');
                                              
            x                        = -this.r_max:this.res:this.r_max;
            y                        = x';
            que                      = [reshape(ones(size(y)).*x, 1, []); reshape(ones(size(x)).*y, 1, [])]';
            IRM_checking_pts         = inpolygon(que(:,1), que(:,2), this.IRM_poly.Vertices(:,1), this.IRM_poly.Vertices(:,2));
            this.IRM_checking_pts    = que(IRM_checking_pts,:);
            
            this.IsDEBUG             = IsDEBUG;
        end
        
        function [rand_pt, current_IRM] = sample(this, T_s_coord, s)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Sample one point from the IRM at progress s
        % Inputs:
        % T_s_coord:    Coordinate of progress s
        % s:            Progress
        % Outputs:
        % rand_pt:      Random sampled pose with 5 states[x, y, cos(theta), sin(theta),s]
        % current_IRM   Array that describe current IRM
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            r       = (this.r_max - this.r_min)*rand(1) + this.r_min;
            r_angle = (this.angle_max - this.angle_min)*rand(1) + this.angle_min;
            theta   = (this.angle_max - this.angle_min)*rand(1) + this.angle_min;
            
            rand_pt = [r*cos(r_angle) + T_s_coord(1), r*sin(r_angle) + T_s_coord(2), cos(theta), sin(theta), s];
            
            if this.IsDEBUG
                current_IRM = this.IRM_Poly(T_s_coord);
            else
                current_IRM = NaN;
            end
        end
        
        function IsInIRM = PtInIRM(this, T_s, pose)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Check if pose is in the IRM at progress s
        % Inputs:
        % T_s:      4x4 transformation matrix
        % pose:     Pose to check
        % Outputs:
        % IsInIRM:  Flag indicate if pose in IRM or not
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            th_thres         = 45*pi/180;
            q                = pose(1:2);
            task             = T_s(1:2, 4)';
            task_q_diff      = task - q;
            task_q_norm_diff = norm(task_q_diff);

            % Cosine similarity: <x_i, x_j>/||x_i||*||x_j||
            cosine_angle     = task_q_diff*pose(3:4)'/(task_q_norm_diff*norm(pose(3:4)));

            % Check:
            % 1. If query point is in range r_min and r_max.
            % 2. If the angle between the vector from the query point to 
            %    the task progress point and the unit vector of the
            %    query point is within a specific thershold:
            if (acos(cosine_angle) < th_thres) && ((this.r_min < task_q_norm_diff) && (task_q_norm_diff < this.r_max))
                IsInIRM = true;
            else
                IsInIRM = false;
            end
        end
        
        function current_IRM_Poly = IRM_Poly(this, T_s_coord)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Form IRM array for plots
        % Inputs:
        % T_s_coord:        Coordinate at progress s
        % Outputs:
        % current_IRM_Poly: Array that describe the IRM at progress s
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            current_IRM_Poly    = [this.inner_circle(1,:) + T_s_coord(1), this.outer_circle(1,:) + T_s_coord(1);
                                   this.inner_circle(2,:) + T_s_coord(2), this.outer_circle(2,:) + T_s_coord(2)];
        end
    end
end