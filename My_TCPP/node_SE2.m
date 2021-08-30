classdef node_SE2 < handle
    properties
        pose
        cost        = 0
        parent
        KDT_Lchild
        KDT_Rchild
    end
    
    methods
        function this = node_SE2(pose)
        %% Initialize://///////////////////////////////////////////////////////////////////////////
            this.pose = pose;
        end
    end
    
    methods (Static)
        function new_nodes = copy(nodes)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Copy nodes by value
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            % covert poses to node structure
            new_nodes = arrayfun(@(idx) node_SE2(nodes(idx).pose), 1:size(nodes, 1))';
            
            % copy other entries
            for idx = 1:size(nodes, 1)
                new_nodes(idx).cost       = nodes(idx).cost;
                new_nodes(idx).parent     = nodes(idx).parent;
                new_nodes(idx).KDT_Lchild = nodes(idx).KDT_Lchild;
                new_nodes(idx).KDT_Rchild = nodes(idx).KDT_Rchild;
            end
        end
        
        function to_pt = copy_data(from_pt, to_pt)
            to_pt.pose   = from_pt.pose;
            to_pt.cost   = from_pt.cost;
            to_pt.parent = from_pt.parent;
        end
    end
end