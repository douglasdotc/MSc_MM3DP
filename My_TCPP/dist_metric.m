classdef dist_metric
    methods (Static)
        function dists = method(pt, nodes, method)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Switch methods
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            
            if strcmp(method, 'sq_norm')
                dists = dist_metric.sq_norm(pt, nodes);
                
            elseif strcmp(method, 'progress_sq_norm')
                dists = dist_metric.progress_sq_norm(pt, nodes);
                
            elseif strcmp(method, 'FMT_progress_sq_norm')
                dists = dist_metric.FMT_progress_sq_norm(pt, nodes);
            end
        end
        
        function dists = sq_norm(pt, nodes)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Euclidean distance: ||x_i - p||
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
%             theta_diffs          = arrayfun(@(node)atan2(node.pose(4), node.pose(3)), nodes) - atan2(pt.pose(4), pt.pose(3));
%             adjusted_theta_diffs = atan2(sin(theta_diffs), cos(theta_diffs));
%             diff                 = cat(1, nodes.pose) - pt.pose;
%             diff_state           = [diff(:,1:2), adjusted_theta_diffs, diff(:,5)];
%             dists                = vecnorm(diff_state, 2, 2);
            
            dists = zeros(size(nodes));
            for idx = 1:length(nodes)
                theta_diff  = atan2(nodes(idx).pose(4), nodes(idx).pose(3)) - atan2(pt.pose(4), pt.pose(3));
                theta_diff  = atan2(sin(theta_diff), cos(theta_diff));
                diff        = nodes(idx).pose - pt.pose;
                dists(idx)  = norm([diff(1:2), theta_diff, diff(5)]);
            end
            % dists = vecnorm(dists, 2, 2);
        end
        
        function dists = progress_sq_norm(pt, nodes)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Angle difference euclidean distance ||x_i(q_1) - p(q_2)|| if s_1 < s_2 else Inf
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
%             theta_diffs                 = arrayfun(@(node)atan2(node.pose(4), node.pose(3)), nodes) - atan2(pt.pose(4), pt.pose(3));
%             adjusted_theta_diffs        = atan2(sin(theta_diffs), cos(theta_diffs));
%             diff                        = cat(1, nodes.pose) - pt.pose;
%             diff_state                  = [diff(:,1:2), adjusted_theta_diffs, round(diff(:,5), 10)];
%             dists                       = vecnorm(diff_state, 2, 2);
%             % Check progress difference: If s_1 >= s_2 --> s_1 - s_2 >= 0 --> Inf
%             dists(diff_state(:,4) >= 0) = Inf;
            
            dists = zeros(size(nodes));
            for idx = 1:length(nodes)
                % handle numerical error by rounding
                s_1 = round(nodes(idx).pose(5), 10);
                s_2 = round(pt.pose(5), 10);
                if s_2 > s_1
                    theta_diff  = atan2(nodes(idx).pose(4), nodes(idx).pose(3)) - atan2(pt.pose(4), pt.pose(3));
                    theta_diff  = atan2(sin(theta_diff), cos(theta_diff));
                    diff        = nodes(idx).pose - pt.pose;
                    dists(idx)  = norm([diff(1:2), theta_diff]); % , round(diff(5), 10)]);
                else
                    dists(idx)  = Inf;
                end
            end
%             assert(sum(dists == dists) == length(nodes))
        end
        
        function dists = FMT_progress_sq_norm(pt, nodes)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Angle difference euclidean distance ||x_i(q_1) - p(q_2)|| if s_1 < s_2 else Inf
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
%             theta_diffs                 = arrayfun(@(node)atan2(node.pose(4), node.pose(3)), nodes) - atan2(pt.pose(4), pt.pose(3));
%             adjusted_theta_diffs        = atan2(sin(theta_diffs), cos(theta_diffs));
%             diff                        = cat(1, nodes.pose) - pt.pose;
%             diff_state                  = [diff(:,1:2), adjusted_theta_diffs, round(diff(:,5), 10)];
%             dists                       = vecnorm(diff_state, 2, 2);
%             % Check progress difference: If s_1 >= s_2 --> s_1 - s_2 >= 0 --> Inf
%             dists(diff_state(:,4) >= 0) = Inf;
            
            dists = zeros(size(nodes));
            for idx = 1:length(nodes)
                % handle numerical error by rounding
                s_1 = round(nodes(idx).pose(5), 10);
                s_2 = round(pt.pose(5), 10);
                if s_1 > s_2
                    theta_diff  = atan2(nodes(idx).pose(4), nodes(idx).pose(3)) - atan2(pt.pose(4), pt.pose(3));
                    theta_diff  = atan2(sin(theta_diff), cos(theta_diff));
                    diff        = nodes(idx).pose - pt.pose;
                    dists(idx)  = norm([diff(1:2), theta_diff]); % , round(diff(5), 10)]);
                else
                    dists(idx)  = Inf;
                end
            end
%             assert(sum(dists == dists) == length(nodes))
        end
        
        function [d, pt_per] = point_to_line(pt, v1, v2)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % This function calculate the shortest distance between a point and
        % a line
        %
        % Inputs:
        % pt:       mxn matrix. Every row is a vector span dimension n
        % v1:       1xn vector, first vertex of the line
        % v2:       1xn vector, second vertex of the line
        % Outputs:
        % d:        distance, scalar
        % pt_per:   1xn vector of the position on the line that corresponds
        %           to the shortest distance
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
          vec_obs_edge = v1 - v2;
          vec_obs_task = v2 - pt;
          
          % mu = -<vec_obs_edge, vec_obs_task>/||vec_obs_edge||^2
          mu = -(vec_obs_task*vec_obs_edge')/(vec_obs_edge*vec_obs_edge');
          
          % Shortest distance between edge (v1 <--> v2) and pt
          % d = ||pt - (mu <= 1)v2 - (mu >= 0 & mu <= 1)*mu*(v1 - v2) - (mu > 1)*v1||
          pt_per = (mu <= 1).*v2 + (mu >= 0 & mu <= 1).*mu.*vec_obs_edge + (mu > 1).*v1;
          d = vecnorm(pt - pt_per, 2, 2);
        end
    end
end