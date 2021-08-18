classdef RRTTaskOrdered < handle
    %PPROBLEM Summary of this class goes here
    %   Detailed explanation goes here

    properties
        C
        qs_list
        qg_list
        e_inc = 0.01;
        e_reach = 0.1;
        bias = 0.01;
        is_qg = []
        max_t
        max_t_node
        q_neigh_rad
        t_timeout
        t_k
        t_var = 0.1

    end

    methods

        function this = RRTTaskOrdered(C, qs_list, qg_list)
            this.C = C;
            this.qs_list = qs_list;
            this.qg_list = qg_list;

        end

        function [q_rand, isgoal] = bias_sample(this)
            %This gives a random, valid q_rand 99% of the time.  1 % of the time it samples from  thre goal set
            isgoal = false;

            if rand() < this.bias
                q_rand = this.C.sample(1); %TODO note that this does not guarantee sample from the t=1

            else
                q_rand = this.C.sample(this.max_t + randn() * this.t_var);
            end

            isgoal = q_rand.q(4) == 1;

        end

        function path = solve(this, K, varargin)
            %% Init vars
            tree = QOrderedArray(this.qs_list);
            this.max_t = 0;
            this.max_t_node = tree;
            k = 1;

            %% Init drawing:
            if ~isempty(varargin)
                draw = any(cellfun(@(c) strcmp('draw', c), varargin));
            else
                draw = false;
            end

            if draw
                h = this.C.updraw_tree([], tree.get());
            end

            %% Main solve loop
            while true

                q_near = [];

                while isempty(q_near)
                    [q_rand, isgoal] = this.bias_sample(); % Should alwayus give valid q_rand or a sample from q_goal
                    [arr, ~] = tree.get(q_rand.q(4), 0.1);
                    q_near = this.nearest(arr, q_rand); % find nearest neighbour
                    %                     q_near = this.nearest(tree(end-min(length(tree)-1,50):end), q_rand); % find nearest neighbour
                end

                [q_new, reach] = this.extend(q_near, q_rand); % find nearest neighbour

                if reach == -1
                    continue
                end

                assert(reach >= 0);
                % tree = [tree; q_new];
                tree.append(q_new)

                if q_new.q(4) > this.max_t
                    this.max_t = q_new.q(4);
                    this.max_t_node = q_new;
                    this.t_k = 1;
                else
                    this.t_k = this.t_k + 1;
                end

                if ~isempty(this.is_qg) && this.is_qg(q_new)
                    disp("sample IS goal")
                    break
                end

                if draw
                    h = this.C.updraw_tree(h, q_new);
                end

                k = k + 1;

                if k > K
                    disp("MAX ITER REACHED")
                    break
                end

                if this.t_k > this.t_timeout
                    disp("INTERUPTION stall")
                    tree = this.add_interruption(tree, draw, h);
                    this.t_k = 1;
                end

                if isgoal && reach
                    break
                end

            end

            %find path
            q = q_new;
            path = q;

            while ~isempty(q.parent) ||~isempty(q.iparent)

                if ~isempty(q.parent)
                    % means its iparent non empty

                    if draw
                        h = this.C.updraw_tree(h, q, 'red', 'green');
                    end

                    path = [q.parent path];
                    q = q.parent;
                else

                    tempq = q;
                    q.parent = q.iparent;

                    if draw
                        h = this.C.updraw_tree(h, tempq, 'green', 'red'); % Highligh the discontinuity!!!!
                    end

                    path = [q.iparent path];
                    q = q.iparent;
                end

            end

        end

        function tree = add_interruption(this, current_tree, draw, h)
            next_t = this.max_t + this.e_inc;
            n = size(this.qs_list, 1);

            for ii = 1:n
                next_qs(ii) = this.C.sample(next_t);
                % q_near = this.nearest(current_tree, next_qs(ii)); % find nearest neighbour
                next_qs(ii).iparent = this.max_t_node; %NOTE THIS SHOULD be all leaf nodes that can reach t_max

                if draw
                    qtemp = next_qs(ii);
                    qtemp.parent = this.max_t_node;
                    h = this.C.updraw_tree(h, qtemp, 'red', 'red');
                end

            end

            % tree = [current_tree; next_qs'];
            current_tree.append(next_qs')

        end

        function draw_path(this, path)
            h = [];

            for ii = 2:length(path)
                path(ii).parent = path(ii - 1);
            end

            %
            q = path(end);
            %             h=this.C.updraw_tree([], q);
            %             h.XData=[];
            %             h.YData=[];
            %             h.UData=[];
            %             h.VData=[];
            %             drawnow
            %             delete(h)
            %             drawnow
            hold on
            qdata = path(1).qmat(path');
            h = quiver(qdata(:, 1), qdata(:, 2), cos(qdata(:, 3)), sin(qdata(:, 3)), .2, 'color', [0.8500 0.3250 0.0980], 'LineWidth', 2);

            while ~isempty(q.parent)
                %                 h = this.C.updraw_tree(h, q, 'red', 'green');
                line([q.parent.q(1) q.q(1)], [q.parent.q(2) q.q(2)], [q.parent.q(4) q.q(4)], 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2);
                q = q.parent;
            end

            drawnow

        end

        function path = simplify(this, path, K)
            k = 1;

            while length(path) > 2

                ids = sort(randi(length(path), 1, 2)); % pick 2 random indices

                while ids(1) == ids(2)%if same picked. try  again
                    ids = sort(randi(length(path), 1, 2));
                end

                if this.isSimplifiable(path, ids(1), ids(2))
                    path(ids(1) + 1:ids(2) - 1) = []; % crop
                end

                k = k + 1;

                if k > K
                    disp("MAX ITER REACHED")
                    break
                end

            end

        end

    end

    methods (Access = private)

        function is = isSimplifiable(this, path, id_form, id_to)

            qs = path(id_form);
            qtt = path(id_to);
            q_travel = qs.new_from_vec(qs.q);

            ddq = this.e_inc * (qtt - qs) / norm(qtt - qs); % note the norm would include time.

            while this.C.dist(qtt, q_travel) > this.e_inc% q_new is more than one increment away

                q_travel.add(ddq); % this handles the wrap 2 pi

                if ~this.C.iisValid(q_travel)% This should capture any collision, out of bounds, or IK or whatever.
                    is = false;
                    return
                end

            end

            is = true;

        end

        function [q_new, reach] = extend(this, q_near, q_rand)
            assert(this.C.dist(q_rand, q_near) ~= inf)% Distance from q_near to q_rand should never be inf. This case should be caught before hand

            if this.C.iisValid(q_rand)% here, check for collisions
                q_new = q_near.new_from_vec(q_near.q); % q_new starts at q_near
                ddq = this.e_inc * (q_rand - q_near) / norm(q_rand - q_near); % note the norm would include time.

                while this.C.dist(q_rand, q_new) > this.e_inc% q_new is more than one increment away

                    q_new.add(ddq); % this handles the wrap 2 pi

                    if ~this.C.iisValid(q_new) || this.C.dist(q_new, q_near) > this.e_reach% This should capture any collision, out of bounds, or IK or whatever. or out of reach
                        q_new.add(-ddq);

                        if q_new.q(4) == q_near.q(4)
                            reach = -1;
                        else
                            reach = 0; % extended, hit obstacle
                            q_new.parent = q_near;
                            q_near.children = [q_near.children q_new];
                        end

                        return
                    end

                end

                q_new.parent = q_near;
                q_near.children = [q_near.children q_new];
                reach = 1; % reached,
                return
            end

            disp("Extend called with invalid q_rand. Should never happen")
            q_new = []; %set to empty
            reach = -1; % failed
            return
        end

        function q_near = nearest(this, tree, q_rand)
            [idx, dist] = knnsearch(tree, q_rand, 'K', 1, 'IncludeTies', false, 'Distance', @(x, y)this.C.dist(x, y));

            if dist ~= inf
                q_near = tree(idx);
            else
                q_near = [];

                if q_rand.q(4) ~= 0
                    disp("q_near not found. Dist-definition 0.3 ~=0")
                    %                     if this triggers check the distance definition at
                    %                     cse2task.
                end

            end

        end

    end

end
