
classdef RRT < handle
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

    end

    methods

        function this = RRT(C, qs_list, qg_list)
            this.C = C;
            this.qs_list = qs_list;
            this.qg_list = qg_list;

        end

        function [q_rand, isgoal] = bias_sample(this,tree)
            %This gives a random, valid q_rand 99% of the time.  1 % of the time it samples from  thre goal set
            if ~isempty(this.qg_list)

                if rand() < this.bias
                    q_rand = this.qg_list(randi(length(this.qg_list)));
                    disp(["goal sampled" num2str(randi(10))])
                    isgoal = true;
                else
                    if ~isempty(tree)
                        temp=vertcat(tree(:).q); 
                         q_rand = this.C.sample(1,max(temp(:,4)));
                    else
                        q_rand = this.C.sample();
                    end
                                     
                   
                    isgoal = false;
                end

            else
                q_rand = this.C.sample();
                isgoal = false;
            end

        end

        function path = solve(this, K, varargin)
            %% Init vars
            tree = this.qs_list;
            k = 1;

            %% Init drawing:
            if ~isempty(varargin)
                draw = any(cellfun(@(c) strcmp('draw', c), varargin));
            else
                draw=false;
            end

            if draw
                h = this.C.updraw_tree([], tree);
            end

            %% Main solve loop
            while true
                [q_rand, isgoal] = this.bias_sample(tree); % Should alwayus give valid q_rand or a sample from q_goal
                q_near = this.nearest(tree, q_rand); % find nearest neighbour
                [q_new, reach] = this.extend(q_near, q_rand); % find nearest neighbour
                assert(~isempty(q_new));
                tree = [tree; q_new];

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

                if isgoal && reach
                    break
                end

            end

            %find path
            q = q_new;
            path = q;

            while ~isempty(q.parent)

                if draw
                    h = this.C.updraw_tree(h, q, 'green');
                end

                path = [q.parent path];
                q = q.parent;

            end

        end

        function path = simplify(this, path,K)
            k=1;
        
            while length(path)>2

                ids=sort(randi(length(path),1,2));
                while ids(1)==ids(2)
                    ids=sort(randi(length(path),1,2));
                end
                if this.isSimplifiable(path,ids(1),ids(2))
                    path(ids(1)+1:ids(2))=[]; % crop
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
            q_travel=qs.new_from_vec(qs.q);

            ddq = this.e_inc * (qtt - qs) / norm(qtt - qs); % note the norm would include time.

            while this.C.dist(qtt, q_travel) > this.e_inc% q_new is more than one increment away

                q_travel.add(ddq); % this handles the wrap 2 pi

                if ~this.C.iisValid(q_travel) % This should capture any collision, out of bounds, or IK or whatever.
                    is=false;
                    return
                end

            end
            is=true;

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
                        q_new.parent = q_near;
                        q_near.children = [q_near.children q_new];
                        reach = 0; % extended, hit obstacle
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
                disp("q_near not found. Should never happen")
            end

        end

    end

end
