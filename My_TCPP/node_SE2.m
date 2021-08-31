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
        
        function copied_node = deep_copy(node)
            if isempty(node)
                copied_node = [];
                return
            else
                copied_node             = node_SE2(node.pose);
                copied_node.cost        = node.cost;
                copied_node.parent      = node_SE2.deep_copy(node.parent);
                copied_node.KDT_Lchild  = node_SE2.deep_copy(node.KDT_Lchild);
                copied_node.KDT_Rchild  = node_SE2.deep_copy(node.KDT_Rchild);
            end
        end
        
        function IsMatch = match_list(node1, node2)
            if ~isempty(node1) && ~isempty(node2)
                IsMatchPose     = all(node1.pose == node2.pose);
                IsMatchCost     = (node1.cost == node2.cost);
                IsMatchParent   = node_SE2.match_list(node1.parent, node2.parent);
                IsMatchLchild   = node_SE2.match_list(node1.KDT_Lchild, node2.KDT_Lchild);
                IsMatchRchild   = node_SE2.match_list(node1.KDT_Rchild, node2.KDT_Rchild);
                IsMatch         = IsMatchPose & IsMatchCost & IsMatchParent & IsMatchLchild & IsMatchRchild;
                
            elseif isempty(node1) && isempty(node2)
                IsMatch = true;
                return
            else
                IsMatch = false;
                return
            end
        end
        
        function [poses, costs] = ExtractData(node, varargin)
            if isempty(varargin)
                poses = [];
                costs = [];
            else
                poses = varargin{1};
                costs = varargin{2};
            end
            
            if isempty(node)
                return
            else
                poses = [poses; node.pose];
                costs = [costs; node.cost];
                [poses, costs] = node_SE2.ExtractData(node.KDT_Lchild, poses, costs);
                [poses, costs] = node_SE2.ExtractData(node.KDT_Rchild, poses, costs);
            end
        end
        
        function count = VerticeCount(node, varargin)
            if isempty(varargin)
                count = 0;
            else
                count = varargin{1};
            end
            
            if isempty(node)
                return
            else
                count  = count + 1;
                countL = node_SE2.VerticeCount(node.KDT_Lchild, count);
                countR = node_SE2.VerticeCount(node.KDT_Rchild, count);
            end
            count = countL + countR;
        end
    end
end