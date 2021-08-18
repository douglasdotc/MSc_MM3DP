classdef RRTStarOrderedTask < handle
    %PPROBLEM Summary of this class goes here
    %   Detailed explanation goes here

    properties
        C
        qs_list
        qg_list
        e_inc = 0.01;
        e_reach = 0.1;
        bias = 0.01;
        q_neigh_rad = 0.1;
        ordered_look = 0.05;
        is_qg = []
        max_t
        max_t_node
        t_timeout
        t_k
        t_var = 0.1
        tree     
        

    end

    methods

        function this = RRTStarOrderedTask(C, qs_list, qg_list)
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
            this.tree = QOrderedArray(this.qs_list);
            [this.max_t_node, this.max_t] = this.tree.getmax();
            k = 1;

            %% Init drawing:
            if ~isempty(varargin)
                draw = true;
                %h=varargin{2};
            else
                draw = false;
            end

            if draw
                h = this.C.updraw_tree([], this.tree.get());
            end

            %% Main solve loop
            while true

                q_near = [];

                while isempty(q_near)
                    [q_rand, isgoal] = this.bias_sample(); % Should alwayus give valid q_rand or a sample from q_goal

                    [arr, ~] = this.tree.get(q_rand.q(4), this.ordered_look);
                    [q_near, q_neigh, ~] = this.nearest_neigh(arr, q_rand); % find nearest neighbour
                end

                q_best = this.pick_best(q_rand, [q_near; q_neigh]);

                [q_new, reach] = this.extend(q_best, q_rand); % find nearest neighbour

                if reach == -1
                    continue
                end

                assert(reach >= 0);

                if reach >= 0
                    q_new.parent = q_best;
                    q_new.cost = q_new.parent.cost + this.C.dist(q_new, q_best);

                    if q_new.cost == inf
                        disp("not ghoot")
                    end

                end

                % tree = [tree; q_new];
                this.tree.append(q_new)

                this.rewire(this.tree, q_new);
                %disp("rewire  commented out");

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

                if draw && mod(K,100)==0
                    h = this.C.updraw_tree(h, q_new);
                end

                k = k + 1;

                if k > K
                    disp("MAX ITER REACHED")
                    break
                end

                if this.t_k > this.t_timeout
                    disp("INTERUPTION stall")
                    this.tree = this.add_interruption(this.tree, draw);
                    this.t_k = 1;
                end

                if isgoal && (reach == 1)
                    break
                end

                if q_new.q(4) > 0.95
                    q_new_goal = QSE2T([q_rand.q(1:3) 1]);
                    [~, reach] = this.extend(q_new, q_new_goal); % find nearest neighbour

                    if reach == 1
                        q_new_goal.parent = q_new;
                        q_new_goal.cost = q_new.cost + this.C.dist(q_new_goal, q_new);
                        this.tree.append(q_new_goal);
                        q_new = q_new_goal;
                        break
                    end

                end

            end

            %find path
            path = this.trace_path(q_new);
            %             this.rewire_path(path);
            %             path = this.trace_path(q_new);

        end

        function smooth_path(this, path)
            m = 20;

            for ii = 1:length(path)

                for jj = 1:m
                    qqrand = path(ii).q + rand(1, 4) * this.e_reach;
                    qqrand(4) = max(0, min(qqrand(4), 1));

                    q_rand = QSE2T(qqrand);
                    [q_new, reach] = this.extend(path(ii), q_rand); % find nearest neighbour

                    if reach == 1

                        q_rand.cost = path(ii).cost + this.C.dist(q_rand, path(ii));

                        if ~(q_rand.cost < inf)
                            continue
                        end

                        q_rand.parent = path(ii);
                        this.tree.append(q_new)
                    end

                end

            end

        end

        function rewire_all(this)
            qarr = this.tree.get();

            for ii = 1:length(qarr)
                this.rewire(this.tree, qarr(ii));
            end

        end

        function rewire_path(this, path)
            % qarr = this.tree.get();

            for ii = 1:length(path)
                this.rewire(this.tree, path(ii), this.q_neigh_rad * 5);
            end

        end

        function path = trace_path(this, q_end)
            q = q_end;
            path = q;
            disp("generating path")

            while ~isempty(q.parent) ||~isempty(q.iparent)

                if length(path) > this.tree.sz
                    disp("LOOPP!! domething is bad")
                end

                if ~isempty(q.parent)
                    % means its iparent non empty

                    path = [q.parent path];
                    q = q.parent;
                else

                    % q.parent = q.iparent;

                    path = [q.iparent path];
                    q = q.iparent;
                end

            end

        end

        function current_tree = add_interruption(this, current_tree, draw)
            next_t = this.max_t + this.e_inc;
            n = size(this.qs_list, 1);

            for ii = 1:n
                next_qs(ii) = this.C.sample(next_t);
                % q_near = this.nearest(current_tree, next_qs(ii)); % find nearest neighbour
                next_qs(ii).iparent = this.max_t_node; %NOTE THIS SHOULD be all leaf nodes that can reach t_max
                next_qs(ii).cost = this.max_t_node.cost; %NOTE THIS SHOULD be all leaf nodes that can reach t_max

                if draw
                    qtemp = next_qs(ii);
                    qtemp.parent = this.max_t_node;
                    %                     h = this.C.updraw_tree(h, qtemp, 'red', 'red');
                end

            end

            % tree = [current_tree; next_qs'];
            current_tree.append(next_qs')

        end

        function draw_path(this, path, varargin)
            h = [];
            c = [0.9290 0.6940 0.1250];

            if ~isempty(varargin)
                c = varargin{1};
            end

            qarr = path;
            qmat = qarr(1).qmat(qarr');

            edgesx = zeros(2, length(path));
            edgesy = zeros(2, length(path));

            iedgesx = zeros(2, 0);
            iedgesy = zeros(2, 0);

            for ii = 1:length(path)
                if ~isempty(path(ii).iparent)
                    iedgesx = [iedgesx [path(ii).iparent.q(1); path(ii).q(1)]];
                    iedgesy = [iedgesy [path(ii).iparent.q(2); path(ii).q(2)]];
                elseif ~isempty(path(ii).parent)
                    edgesx(1, ii) = path(ii).parent.q(1);
                    edgesx(2, ii) = path(ii).q(1);
                    edgesy(1, ii) = path(ii).parent.q(2);
                    edgesy(2, ii) = path(ii).q(2);               

                end

            end

            hold on
            cc = colormap('jet');
            cc = cc * 0.7;
            ccolor = interp1(1:256, cc, linspace(2, 253, length(path))') * .98 +0.01;
            
            edgesx(:,3)=0.1;
          
            iedgesx(:,3)=0.1;
            iedgesy(:,3)=0.1;
            
            edgesz=ones(size(edgesx))*1;
            iedgesz=ones(size(iedgesy))*1;

            
            %l = line(edgesx, edgesy, 'LineWidth', 3);
            l = line(edgesx, edgesy,edgesz, 'LineWidth', 3.5);

            for ii = 1:length(l)
                l(ii).Color = ccolor(ii, :);
            end

            %il = line(iedgesx, iedgesy, 'LineWidth', 2.5, 'LineStyle', '--', 'Color', 'r');
            il = line(iedgesx, iedgesy,iedgesz, 'LineWidth', 3, 'LineStyle', '--', 'Color', 'r');
%             il = line(iedgesx, iedgesy, 'LineWidth', 2, 'LineStyle', '--', 'Color', [0.6350 0.0780 0.1840]);
            inds=1:5:size(qmat,1);
            h = quiver(qmat(inds, 1), qmat(inds, 2), cos(qmat(inds, 3)), sin(qmat(inds, 3)), .4, 'color', [0.8500 0.3250 0.0980] * .65, 'LineWidth', 1.5);

            drawnow
        end

        function draw_robot(this, task, states)
            mycube = @(x, y, dx, dy) [[(x - dx / 2), (y - dy / 2), 1]; [(x - dx / 2), (y +dy / 2), 1]; [(x +dx / 2), (y +dy / 2), 1]; [(x +dx / 2), (y -dy / 2), 1]];

            ywx = 0.55; ywy = 0.34;
            youwasp = mycube(0, 0, ywx, ywy);

            TT = @(t)task(:, :, 1 + ceil(t * (size(task, 3) - 1)));

            tform22d = @(T) [T(1:2, [1:2, 4]); 0 0 1];

            hold on

            for ii = 1:length(states)
                frame = eul2tform([states(ii).q(3) 0 0]);
                frame(1:2, 4) = states(ii).q(1:2);
                frame = tform22d(frame);
                p = (frame * (youwasp'))';
                plot(polyshape(p(:, 1:2)), 'FaceColor', [0.9608 0.5961 0.2588], 'FaceAlpha', 0.3);

                T = tform22d(TT(states(ii).q(4)));
                ang = atan2(T(2, 3) - states(ii).q(2), T(1, 3) - states(ii).q(1));

                ypx = norm(T(1:2, 3) - states(ii).q(1:2)');
                ypy = 0.1;
                arm = mycube(0, 0, ypx, ypy);
                arm(:, 1) = arm(:, 1) + 0.3;

                frame = eul2tform([ang 0 0]);
                frame(1:2, 4) = states(ii).q(1:2);
                frame = tform22d(frame);
                p = (frame * (arm'))';
                h = plot(polyshape(p(:, 1:2)), 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.3);

            end

            drawnow

        end

        function draw_tree(this)
            h = [];
            qarr = this.tree.get();

            qmat = qarr(1).qmat(qarr);
            edgesx = zeros(2, size(qarr, 1));
            edgesy = zeros(2, size(qarr, 1));
            % C.plot_environment()

            for ii = 1:size(qarr, 1)

                if ~isempty(qarr(ii).parent)
                    edgesx(1, ii) = qarr(ii).parent.q(1);
                    edgesx(2, ii) = qarr(ii).q(1);
                    edgesy(1, ii) = qarr(ii).parent.q(2);
                    edgesy(2, ii) = qarr(ii).q(2);
                end

            end

            l = line(edgesx, edgesy, 'Color', [ 0.6314    0.9137    1.0000]*0.8);
            hold on
            h = quiver(qmat(:, 1), qmat(:, 2), cos(qmat(:, 3)), sin(qmat(:, 3)), .3, 'color', [ 0.7314    0.9137    1.0000] * .95, 'LineWidth', 1);

            drawnow

        end

        function pathsimple = simplify(this, path, K)
            pathsimple = arrayfun(@(q) QSE2T(q.q), path);

            for ii = 1:length(path)
                pathsimple(ii).parent = path(ii).parent;
                pathsimple(ii).iparent = path(ii).iparent;
                pathsimple(ii).cost = path(ii).cost;
            end

            k = 1;

            while length(path) > 2

                ids = sort(randi(length(pathsimple), 1, 2)); % pick 2 random indices

                while ids(1) == ids(2)%if same picked. try  again
                    ids = sort(randi(length(pathsimple), 1, 2));
                end

                if this.isSimplifiable(pathsimple, ids(1), ids(2))
                    pathsimple(ids(2) - 1).parent = pathsimple(ids(1) + 1);
                    pathsimple(ids(1) + 1:ids(2) - 1) = []; % crop

                end

                k = k + 1;

                if k > K
                    disp("MAX ITER REACHED")
                    break
                end

            end

        end

        function pathsimple = smooth(this, path, K)
            pathsimple = arrayfun(@(q) QSE2T(q.q), path)';

            for ii = 1:length(path)
                pathsimple(ii).parent = path(ii).parent;
                pathsimple(ii).iparent = path(ii).iparent;
                pathsimple(ii).cost = path(ii).cost;
            end

            pathsimple = reshape(pathsimple, length(pathsimple), 1);

            k = 1;

            while true

                ids = sort(randi(length(pathsimple) - 2, 1, 2) + 1); % pick 2 random indices

                while ids(1) == ids(2)%if same picked. try  again
                    ids = sort(randi(length(pathsimple) - 2, 1, 2) + 1); % pick 2 random indices
                end

                isSimple = this.isSimplifiable(pathsimple, ids(1), ids(2));

                if isSimple

                    [is, smooth_path] = this.isSmoothable(pathsimple, ids(1), ids(2));

                    if is
                        pathsimple(ids(2) + 1).parent = smooth_path(end);
                        smooth_path(1).parent = pathsimple(ids(1) - 1);
                        pathsimple = vertcat(pathsimple(1:ids(1) - 1), smooth_path, pathsimple(ids(2) + 1:end));

                    end

                end

                k = k + 1

                if k > K
                    disp("MAX ITER REACHED")
                    break
                end

            end

            % Recompute costs

        end

    end

    methods % (Access = private)

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

        function [is, smooth_path] = isSmoothable(this, path, id_form, id_to)

            is = false;
            smooth_path = [];

            function [qmat, cumlength] = resample(qmat, density)
                cumlength = [0; cumsum(sqrt(sum(diff(qmat).^2, 2)))];
                [~, IA, ~] = uniquetol(cumlength);
                cumlength = cumlength(IA);
                qmat = qmat(IA, :);
                qmat = interp1(cumlength, qmat, linspace(0, cumlength(end), round(cumlength(end) / density)), 'pchip');
            end

            path = reshape(path, length(path), 1);

            q_path = path(id_form:id_to);
            qmat = q_path(1).qmat(q_path);
            qmat(:, 3) = unwrap(qmat(:, 3));
            [qmatr, cumlength] = resample(qmat, this.e_inc);
            qmats(:, 1) = smooth(qmatr(:, 1), 3 * this.e_reach / cumlength(end));
            qmats(:, 2) = smooth(qmatr(:, 2), 3 * this.e_reach / cumlength(end));
            qmats(:, 3) = smooth(qmatr(:, 3), 3 * this.e_reach / cumlength(end));
            qmats(:, 4) = smooth(qmatr(:, 4), 3 * this.e_reach / cumlength(end));
            n = length(qmats);
            assert(all(abs(qmats(1, :) -qmat(1, :)) < 0.0001));
            assert(all(abs(qmats(end, :) -qmat(end, :)) < 0.0001));

            qmatsq = arrayfun(@(~) QSE2T([-1, -1, -1, -1]), zeros(1, n))';

            for ii = 2:n
                qmatsq(ii).q = qmats(ii, :);

                if ~this.C.iisValid(qmatsq(ii))
                    is = false;
                    smooth_path = [];
                    return
                end

                qmatsq(ii).parent = qmatsq(ii - 1);
            end

            smooth_path = qmatsq;
            is = true;

        end

       
        
        
        function [q_new, reach] = extend(this, q_near, q_rand, varargin)
            assert(this.C.dist(q_rand, q_near) ~= inf)% Distance from q_near to q_rand should never be inf. This case should be caught before hand
            % if this.C.dist(q_rand, q_near) == inf
            %     q_new = []; %set to empty
            %     reach = -1; % failed
            %     return
            % end

            rr = this.e_reach;

            if ~isempty(varargin)
                rr = varargin{1};
            end

            if this.C.iisValid(q_rand)% here, check for collisions
                q_new = q_near.new_from_vec(q_near.q); % q_new starts at q_near
                ddq = this.e_inc * (q_rand - q_near) / norm(q_rand - q_near); % note the norm would include time.

                while this.C.dist(q_rand, q_new) > this.e_inc% q_new is more than one increment away

                    q_new.add(ddq); % this handles the wrap 2 pi

                    if ~this.C.iisValid(q_new) || this.C.dist(q_new, q_near) > rr% This should capture any collision, out of bounds, or IK or whatever. or out of reach
                        q_new.add(-ddq);
                        % q_new.parent = q_near;
                        % q_near.children = [q_near.children q_new];
                        if q_new.q(4) <= q_near.q(4)
                            reach = -1;
                        else
                            assert(this.C.dist(q_new, q_near) ~= inf)
                            reach = 0; % extended, hit obstacle
                        end

                        return
                    end

                end

                % q_new.parent = q_near;
                % q_near.children = [q_near.children q_new];
                reach = 1; % reached,
                return
            end

            disp("Extend called with invalid q_rand. Should never happen")
            q_new = []; %set to empty
            reach = -1; % failed
            return
        end

        function [q_near, q_neigh, DD] = nearest_neigh(this, tree, q_rand)
            %             qarr = tree.get();
            if isempty(tree)
                q_near = [];
                q_neigh = [];
                DD = [];
                return
            end

            DD = [];
            [D, I] = pdist2(tree, q_rand, @(x, y)this.C.dist(x, y), 'Smallest', length(tree));

            if D(1) ~= inf
                % disp("CHECK THIS")
                q_near = tree(I(1));
                q_neigh = tree(I(D < this.q_neigh_rad));
                DD = D(D < this.q_neigh_rad);
            else
                q_near = [];
                q_neigh = [];

                if q_rand.q(4) ~= 0
                    disp("q_near not found. Dist-definition 0.3 ~=0")
                    %                     if this triggers check the distance definition at
                    %                     cse2task.
                end

            end

        end

        function q_best = pick_best(this, q_rand, q_neigh)

            c = arrayfun(@(q) q.cost, q_neigh);
            D = pdist2(q_rand, q_neigh, @(x, y)this.C.dist(x, y));
            c = c + D;
            [~, I] = sort(c);
            q_best = q_neigh(I(1));

        end

        function tree = rewire(this, tree, q_new, varargin)

            if ~isempty(q_new.iparent)
                return
            end

            V = this.q_neigh_rad;

            if ~isempty(varargin)
                V = varargin{1};
            end

            [arr, ~] = tree.get(q_new.q(4),  this.ordered_look);
            [~, q_neigh, D] = this.nearest_neigh(arr, q_new); %% TODO I'm not 100% sure if i should be calculating a new neighborhood.  THis will be expensive.  But i can also do t magic to only consider like 50 things.

            for ii = 1:length(q_neigh)

                q_n = q_neigh(ii);

                if (q_n == q_new.parent)
                    continue
                end

                if ~isempty(q_n.iparent)
                    continue
                end

                %                 if isempty(q_n.parent)
                %                     pc = D(ii);
                %                 else
                %
                %                 end
                pc = q_new.cost +D(ii);

                if pc < q_n.cost
                    [~, reach] = this.extend(q_n, q_new, V); % find nearest neighbour

                    if reach == 1
                        q_n.parent = q_new;
                        q_n.cost = pc;
                    end

                end

            end

        end

    end

end
