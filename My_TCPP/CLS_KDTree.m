classdef CLS_KDTree
    properties
        %% Description://///////////////////////////////////////////////////////////////////////////
        % CLS_KDTree expects nodes defined with Superclass "handle" (classdef nodes < handle)
        % And expect the nodes defined with properties along with Data:
        % node --- Data_1
        %       |- ...
        %       |- Data_n
        %       |- KDT_Lchild
        %       -- KDT_Rchild
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
    end
    
    methods (Static)
        function [isReached, isInserted] = insert(pt_new, node, varargin)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Recurrsively search for the the leaf in the KD tree where the new point (pt_new) fits.
        % When the place is found, define:
        %   node.KDT_Lchild OR node.KDT_Rchild = pt_new (*)
        % 
        % Inputs:
        % 1. pt_new:    the node to be insert
        % 2. node:      the node to be examined [init: node(1)]
        % 3. varargin
        %    - current_dim: The dimension that the function is looking at
        %    - isReached:   Flag for founding the insert location
        %    - isInserted:  Flag for inserting pt_new (*)
        % Outputs:
        % 1. isReached:     --
        % 2. isInserted:    --
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if isempty(varargin)
                current_dim = 0;
                isReached   = false;
                isInserted  = false;
            else
                current_dim = varargin{1};
                isReached   = varargin{2};
                isInserted  = varargin{3};
            end
            n_dim = size(pt_new.pose, 2);
            
            if isempty(node)
                isReached = true;
                
            elseif pt_new.pose == node.pose
                % Duplicated, do nothing
                
            elseif pt_new.pose(current_dim + 1) < node.pose(current_dim + 1)
                % Go left side
                current_dim             = mod((current_dim + 1), n_dim);
                [isReached, isInserted] = CLS_KDTree.insert(pt_new, node.KDT_Lchild, current_dim, isReached, isInserted);
                if isReached && ~isInserted
                    node.KDT_Lchild = pt_new;
                    isInserted      = true;
                end
            else
                % Go right side
                current_dim             = mod((current_dim + 1), n_dim);
                [isReached, isInserted] = CLS_KDTree.insert(pt_new, node.KDT_Rchild, current_dim, isReached, isInserted);
                if isReached && ~isInserted
                    node.KDT_Rchild = pt_new;
                    isInserted      = true;
                end
            end
        end
        
        function [kNN, min_dist] = Nearest_Neighbour(pt, node, dist_method, k, varargin)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Recurrsively find the nearest neighbour with KD tree structured
        % node
        % Input:
        % 1. pt:    Point of interest
        % 2. node:  the node to be examined [init: node(1)]
        % 3. varargin:
        %    - current_dim: Current dimension
        %    - min_dist:    Minimum distance
        %    - NN:          Nearest neighbour found at the moment
        % Outputs:
        % 2. NN:        --
        % 1. min_dist:  --
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if isempty(varargin)
                current_dim  = 1;
                min_dist     = Inf(k, 1);
                kNN          = [];
                for kdx = 1:k
                    kNN = [kNN; node_SE2(Inf(size(node.pose)))];
                end
            else
                current_dim  = varargin{1};
                kNN          = varargin{2};
                min_dist     = varargin{3};
            end
            n_dim = size(pt.pose, 2);
            
            % Capture leaf node
            if isempty(node)
                return
            end
            
            % Found point nearer than the nearest:
            dist = dist_metric.method(pt, node, dist_method);
            if any(dist < min_dist)
                idx = find(dist < min_dist);
                idx = idx(1);
                min_dist = [min_dist(1:idx - 1); dist; min_dist(idx:end - 1)];
                kNN      = [kNN(1:idx - 1); node_SE2.copy(node); kNN(idx:end - 1)];
%                 min_dist = dist;
%                 kNN      = node;
            end
            
            % Calculate next dimension
            next_dim = mod(current_dim, n_dim) + 1;
            
            % Visit subtree
            if pt.pose(current_dim) < node.pose(current_dim)
                % Go left side
                [kNN, min_dist] = CLS_KDTree.Nearest_Neighbour(pt, node.KDT_Lchild, dist_method, k, next_dim, kNN, min_dist);
                if min_dist >= abs(pt.pose(current_dim) - node.pose(current_dim))
                	[kNN, min_dist] = CLS_KDTree.Nearest_Neighbour(pt, node.KDT_Rchild, dist_method, k, next_dim, kNN, min_dist);
                end
            else
                % Go right side
                [kNN, min_dist] = CLS_KDTree.Nearest_Neighbour(pt, node.KDT_Rchild, dist_method, k, next_dim, kNN, min_dist);
                if min_dist >= abs(pt.pose(current_dim) - node.pose(current_dim))
                    [kNN, min_dist] = CLS_KDTree.Nearest_Neighbour(pt, node.KDT_Lchild, dist_method, k, next_dim, kNN, min_dist);
                end
            end
        end
        
        function [neighs, costs] = Near_Neighbour(pt, node, dist_method, r, varargin)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Recurrsively find the nearest neighbour with KD tree structured
        % node
        % Input:
        % 1. pt:    Point of interest
        % 2. node:  the node to be examined [init: node(1)]
        % 3. varargin:
        %    - current_dim: Current dimension
        %    - min_dist:    Minimum distance
        %    - NN:          Nearest neighbour found at the moment
        % Outputs:
        % 2. NN:        --
        % 1. min_dist:  --
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if isempty(varargin)
                current_dim  = 1;
                neighs       = [];
                costs        = [];
            else
                current_dim  = varargin{1};
                neighs       = varargin{2};
                costs        = varargin{3};
            end
            n_dim = size(pt.pose, 2);
            
            % Capture leaf node
            if isempty(node)
                return
            end
            
            % Found point nearer than the nearest:
            dist = dist_metric.method(pt, node, dist_method);
            if any(dist <= r)
                neighs = [neighs; node_SE2.copy(node)];
                costs  = [costs; node.cost];
            end
            
            % Calculate next dimension
            next_dim = mod(current_dim, n_dim) + 1;
            
            % Visit subtree
            if pt.pose(current_dim) < node.pose(current_dim)
                % Go left side
                [neighs, costs] = CLS_KDTree.Near_Neighbour(pt, node.KDT_Lchild, dist_method, r, next_dim, neighs, costs);
                if r >= abs(pt.pose(current_dim) - node.pose(current_dim))
                	[neighs, costs] = CLS_KDTree.Near_Neighbour(pt, node.KDT_Rchild, dist_method, r, next_dim, neighs, costs);
                end
            else
                % Go right side
                [neighs, costs] = CLS_KDTree.Near_Neighbour(pt, node.KDT_Rchild, dist_method, r, next_dim, neighs, costs);
                if r >= abs(pt.pose(current_dim) - node.pose(current_dim))
                    [neighs, costs] = CLS_KDTree.Near_Neighbour(pt, node.KDT_Lchild, dist_method, r, next_dim, neighs, costs);
                end
            end
        end
        
        function mind_pt = FindMin(node, dim, varargin)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Find the point with the smallest value in the dth dimension (dim)
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if isempty(varargin)
                current_dim  = 1;
                n_dim        = size(node.pose, 2);
            else
                current_dim  = varargin{1};
                n_dim        = varargin{2};
            end
            next_dim = mod(current_dim, n_dim) + 1;
            
            if isempty(node)
                mind_pt = node_SE2(NaN(1, n_dim));
                return;
            end
            
            if current_dim == dim
                if isempty(node.KDT_Lchild)
                    mind_pt = node;
                    return;
                else
                    mind_pt = CLS_KDTree.FindMin(node.KDT_Lchild, dim, next_dim, n_dim);
                    return;
                end
            else
                mind_pt_left  = CLS_KDTree.FindMin(node.KDT_Lchild, dim, next_dim, n_dim);
                mind_pt_right = CLS_KDTree.FindMin(node.KDT_Rchild, dim, next_dim, n_dim);
                
                compare_pts   = [mind_pt_left.pose; mind_pt_right.pose; node.pose];
                [~, idx]      = min(compare_pts(:, dim));
                
                if idx == 1
                    mind_pt = mind_pt_left;
                elseif idx == 2
                    mind_pt = mind_pt_right;
                else
                    mind_pt = node;
                end
                return;
            end
        end
        
        function node = Delete(pt, node, varargin)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Delete pt from KD tree
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if isempty(varargin)
                current_dim = 1;
                n_dim       = size(node.pose, 2);
                isReached   = false;
                isRemoved   = false;
            else
                current_dim = varargin{1};
                n_dim       = varargin{2};
                isReached   = varargin{3};
                isRemoved   = varargin{4};
            end
            next_dim = mod(current_dim, n_dim) + 1;
            
            if isempty(node)
                % Given point is not present
                return;
            end
        
            if all(pt.pose == node.pose)
                if ~isempty(node.KDT_Rchild)
                    % Find minimum of root's dimension in right subtree
                    mind_pt         = CLS_KDTree.FindMin(node.KDT_Rchild, current_dim, next_dim, n_dim);
                    % Copy the minimum to root (data only)
                    node            = node_SE2.copy_data(mind_pt, node);
                    % Recursively delete the minimum
                    node.KDT_Rchild = CLS_KDTree.Delete(mind_pt, node.KDT_Rchild, next_dim, n_dim, isReached, isRemoved);
                    
                elseif ~isempty(node.KDT_Lchild)
                    mind_pt         = CLS_KDTree.FindMin(node.KDT_Lchild, current_dim, next_dim, n_dim);
                    node            = node_SE2.copy_data(mind_pt, node);
                    node.KDT_Rchild = CLS_KDTree.Delete(mind_pt, node.KDT_Lchild, next_dim, n_dim, isReached, isRemoved);
                    % Delete left child or else duplicated
                    node.KDT_Lchild = [];
                    
                else
                    % Node to be deleted is leaf node
                    node = [];
                    return;
                end
                return;
                
            elseif pt.pose(current_dim) < node.pose(current_dim) % If current node doesn't contain point, search downward
                node.KDT_Lchild = CLS_KDTree.Delete(pt, node.KDT_Lchild, next_dim, n_dim, isReached, isRemoved);
                
            else
                node.KDT_Rchild = CLS_KDTree.Delete(pt, node.KDT_Rchild, next_dim, n_dim, isReached, isRemoved);
                
            end
            return;
        end
    end
end