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
        
        function [NN, min_dist] = Nearest_Neighbour(pt, node, dist_method, varargin)
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
                current_dim = 1;
                min_dist    = Inf;
                NN          = [];
            else
                current_dim = varargin{1};
                NN          = varargin{2};
                min_dist    = varargin{3};
            end
            n_dim = size(pt.pose, 2);
            
            % Capture leaf node
            if isempty(node)
                return
            end
            
            % Found point nearer than the nearest:
            dist = dist_metric.method(pt, node, dist_method);
            if dist < min_dist
                NN       = node;
                min_dist = dist;
            end
            
            % Calculate next dimension
            next_dim = mod(current_dim, n_dim) + 1;
            
            % Visit subtree
            if pt.pose(current_dim) < node.pose(current_dim)
                % Go left side
                [NN, min_dist] = CLS_KDTree.Nearest_Neighbour(pt, node.KDT_Lchild, dist_method, next_dim, NN, min_dist);
                if min_dist >= abs(pt.pose(current_dim) - node.pose(current_dim))
                	[NN, min_dist] = CLS_KDTree.Nearest_Neighbour(pt, node.KDT_Rchild, dist_method, next_dim, NN, min_dist);
                end
            else
                % Go right side
                [NN, min_dist] = CLS_KDTree.Nearest_Neighbour(pt, node.KDT_Rchild, dist_method, next_dim, NN, min_dist);
                if min_dist >= abs(pt.pose(current_dim) - node.pose(current_dim))
                    [NN, min_dist] = CLS_KDTree.Nearest_Neighbour(pt, node.KDT_Lchild, dist_method, next_dim, NN, min_dist);
                end
            end
        end
    end
end