KD_time = [];
sq_time = [];

for n = 1:10:3000
    for r = 0:0.5:3
        x                 = TD_sample_pts(1);
        [KD_nodes, nodes] = TD_sample_pts(n);
        tic
        [KD_Near, ~]  = CLS_KDTree.Near_Neighbour(x, KD_nodes, 'sq_norm', r);
        KD_time = [KD_time;n,r,toc];
        
        tic
        Near_dist         = dist_metric.method(x, nodes, 'sq_norm');
        Near              = nodes(Near_dist <= r);
        sq_time = [sq_time;n,r,toc];
        
        if isempty(KD_Near) && isempty(Near)
        elseif isempty(KD_Near) && ~isempty(Near)
            error
        elseif ~isempty(KD_Near) && isempty(Near)
            error
        else
            KD_POSES = Extract_item(KD_Near, 'pose');
            POSES = Extract_item(Near, 'pose');
            assert(all(all(sort(KD_POSES) == sort(POSES))))
        end
    end
end

function [KD_nodes, nodes] = TD_sample_pts(n, varargin)
    %% TO BE DELETED
    if ~isempty(varargin)
        KD_nodes   = varargin{1};
    else
        KD_nodes   = [];
        nodes = [];
    end

    for idx = 1:n
        pt = [randi(3)*rand(1,2), 1, 0, 0];
        pt = node_SE2(pt);
        nodes = [nodes; pt];
        
        if isempty(KD_nodes)
            KD_nodes = pt;
        else
            CLS_KDTree.insert(pt, KD_nodes);
        end
    end
end

function val = Extract_item(nodes, item)
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