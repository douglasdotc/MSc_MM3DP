classdef CSE2TTask < CSE2T

    properties
        taskT
        task
        robotbbox
        convex_obstacles
        IRM
        IRMT
        IRMN
        cutoff
        ikobj

    end

    methods

        function this = CSE2TTask(xlim, ylim, taskT, robotbbox, IRM, cutoff)
            this = this@CSE2T(xlim, ylim);
            this.taskT = taskT;
            this.robotbbox = robotbbox;
            this.IRM = IRM;
            this.IRMT = cat(3, IRM.Vrms{IRM.N(:) > 0});
            this.IRMN = IRM.N(IRM.N(:) > 0);
            cutoff = prctile(this.IRMN(:), cutoff);
            %             cutoff=0;
            this.IRMT = this.IRMT(:, :, this.IRMN > cutoff);
            this.IRMN = this.IRMN(this.IRMN > cutoff);
            this.task = squeeze(this.taskT(1:2, 4, :))';
            this.cutoff = cutoff;

        end

        function T = sample_IRM_from_task(this, t, n)
            task = this.taskT(:, :, min(size(this.taskT, 3), max(1, ceil(size(this.taskT, 3) * t)))); % indexing into the task
            [T, smpl_ids] = CSE2TTask.sample_tform(this.IRMT, this.IRMN, n);
            T = TForm.tformX(task, T); %transform IRM to
            % [~,nn]=CSE2TTask.index_IRM_with_pose(this.IRM, QinTaskFrame);
            % if nn<this.cutoff
            % disp("This is bad");
            % end
        end

        function is = iisValid(this, q)
            is = iisValid@CSE2T(this, q);

            q_T = [cos(q.q(3)), -sin(q.q(3)), q.q(1); sin(q.q(3)) cos(q.q(3)) q.q(2); 0 0 1];
            robot_poly = q_T * this.robotbbox;

            % check obstacles
            if ~isempty(this.convex_obstacles)
                obs = vertcat(this.convex_obstacles{:});
                xq = obs(:, 1);
                yq = obs(:, 2);
                in = inpolygon(xq, yq, robot_poly(1, :)', robot_poly(2, :)');

                if any(in)
                    disp("Obstacle collision found")
                    is = false;
                    return
                end

            end

            % check task collision
            xq = this.task(1:round(length(this.task) * q.q(4)), 1);
            yq = this.task(1:round(length(this.task) * q.q(4)), 2);
            in = inpolygon(xq, yq, robot_poly(1, :)', robot_poly(2, :)');

            if any(in)
                disp("task collision found")
                is = false;
                return
            end

            %check  IF STILL IN IRM
            TaskT = this.taskT(:, :, min(size(this.taskT, 3), max(1, ceil(length(this.task) * q.q(4)))));
            Tq = q2tform(q);
            QinTaskFrame = tform2inv(TaskT) * Tq;
            [~, N] = CSE2TTask.index_IRM_with_pose(this.IRM, QinTaskFrame);

            if N < this.cutoff
                % disp(N)
                is = false;
                return
            end

            return
        end

        function q = rand_q(this, varargin)
            %% Ad  IRM bias here?
            %             qse2t = rand_q@CSE2T(this);
            if ~isempty(varargin)
                t = varargin{1};
                % t = t_max + randn() * 0.1;
                t = min(1, max(0, t));
            else
                t = rand();
            end

            T = this.sample_IRM_from_task(t, 1);
            q = QSE2T([tform2q(T), t]);
        end

        function h = updraw_tree(this, h, q_vec, varargin)
            color = 'blue';
            color_edge = 'k';

            if ~isempty(varargin) &&~isempty(varargin{1})
                color = varargin{1};
            end

            if length(varargin) > 1 &&~isempty(varargin{2})
                color_edge = varargin{2};
            end

            n = size(q_vec, 1);

            if n > 1
                qdata = q_vec(1).qmat(q_vec);
            else
                qdata = q_vec.q;
            end

            if isempty(h)
                figure()
                hold on

                if ~isempty(this.convex_obstacles)

                    for ii = 1:length(this.convex_obstacles)
                        ob = this.convex_obstacles{ii};
                        plot([ob(:, 1); ob(1, 1)], [ob(:, 2); ob(1, 2)], '-k', 'LineWidth', 3);
                    end

                end

                hold on
                temp = TForm.tform2vec(this.taskT);
                Visualization.draw_poses(temp, 0.3, [1 0 0]);
                %                 plot(this.task, this.convex_obstacles(:, 2), '-k', 'LineWidth', 3);
                if strcmp(color, 'red')
                    h = quiver(qdata(:, 1), qdata(:, 2), cos(qdata(:, 3)), sin(qdata(:, 3)), .5, 'color', [0.8500 0.3250 0.0980]);
                end

                h = quiver(qdata(:, 1), qdata(:, 2), cos(qdata(:, 3)), sin(qdata(:, 3)), .5, 'color', [0 0.4470 0.7410]);
                hold off
            else
                h.XData = [h.XData; qdata(:, 1)];
                h.YData = [h.YData; qdata(:, 2)];
                h.UData = [h.UData; cos(qdata(:, 3))];
                h.VData = [h.VData; sin(qdata(:, 3))];
                assert(n == 1)

                if strcmp(color_edge, 'red')
                    line([q_vec.parent.q(1) q_vec.q(1)], [q_vec.parent.q(2) q_vec.q(2)], [q_vec.parent.q(4) q_vec.q(4)], 'Color', [0.8500 0.3250 0.0980], 'LineStyle', ':', 'LineWidth', 1);
                elseif strcmp(color_edge, 'green')
                    line([q_vec.parent.q(1) q_vec.q(1)], [q_vec.parent.q(2) q_vec.q(2)], [q_vec.parent.q(4) q_vec.q(4)], 'Color', [0.4660 0.6740 0.1880], 'LineStyle', '-', 'LineWidth', 2.5);

                else
                    line([q_vec.parent.q(1) q_vec.q(1)], [q_vec.parent.q(2) q_vec.q(2)], [q_vec.parent.q(4) q_vec.q(4)], 'Color', color_edge);

                end

            end

            drawnow

        end

    end

    methods (Static)

        function [T, N] = index_IRM_with_pose(IRM, poseT)
            % poseT assumed in local task frame.
            eul = rotm2eul(poseT(1:3, 1:3), 'XYZ');
            pose = [poseT(1:3, 4)' eul];

            ids = zeros(1, 6);

            try

                for d = 1:3
                    ids(d) = find(abs(pose(d) - IRM.spans{d}) < IRM.radii(d));
                end

                %             ids(3)=1;
                %             disp("ids3 =1")
                ad = @(t, s)atan2(sin(t - s), cos(t - s)); %ang diff faster than angdif... ? lol

                for d = 4:6
                    ids(d) = find(abs(ad(pose(d), IRM.spans{d})) < IRM.radii(d));
                end

                T = IRM.Vrms{ids(1), ids(2), ids(3), ids(4), ids(5), ids(6)};
                N = IRM.N(ids(1), ids(2), ids(3), ids(4), ids(5), ids(6)); % note that IRM is in task frame. which means that N is usually empty for all Z axes pointing up. (cuz from task perspective, Z axes of robot base point down.)
            catch
                disp("IRM out of bounds");
                T = [];
                N = 0;
            end

        end

        function [T, smpl_ids] = sample_tform(T, val, n)
            % Samples SE3 Transforms given probability
            %T tform 4x4xm
            %val mx1 probability
            %n num of samples
            s = sum(val);
            cdf = cumsum(val);
            smpl_ids = sum(bsxfun(@le, cdf', rand(n, 1) * s), 2) + 1;
            T = T(:, :, smpl_ids);
        end

        function draw_full_path(path)
            figure
            path = vertcat(path.q);
            hold on

            q = quiver(path(:, 1), path(:, 2), cos(path(:, 3)), sin(path(:, 3)), 0.1);
            plot(path(:, 1), path(:, 2), '-r');
            quiver(path(1, 1), path(1, 2), cos(path(1, 3)), sin(path(1, 3)), 0.2, 'blue')
            quiver(path(end, 1), path(end, 2), cos(path(end, 3)), sin(path(end, 3)), 0.2, 'green')
            xlabel('x')
            ylabel('y')

        end

    end

end
