classdef CSE2TTask2 < CSE2T

    properties
        taskT
        task
        robotbbox
        convex_obstacles
        IRM
        taskTask

    end

    methods

        function this = CSE2TTask2(xlim, ylim, taskT, taskTask, robotbbox, IRM)
            this = this@CSE2T(xlim, ylim);
            this.taskT = taskT;
            this.robotbbox = robotbbox;
            this.IRM = IRM;
            this.taskTask = taskTask;

            this.task = squeeze(this.taskT(1:2, 4, :))';

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

            % Tq = q2tform(q);
            % QinTaskFrame = tform2inv(TaskT) * Tq;

            isin = InverseReachability.irm_isse2pin(this.IRM, q.q(1:3), TaskT(1:3, 4));
            is = is && isin;

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

            q = sample_qs_around_task(this, t, 1);

        end

        function qs = sample_qs_around_task(this, t, n)

            T = this.taskT(:, :, min(size(this.taskT, 3), max(1, ceil(size(this.taskT, 3) * t))));
            % indexing into the task
            [se2ps, ~] = InverseReachability.irm_samplese2p(this.IRM, n, T(1:3, 4));

            qs = arrayfun(@(id) QSE2T([se2ps(id, :), t]), 1:size(se2ps, 1))';

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
                %figure()
                hold on

                if ~isempty(this.convex_obstacles)

                    for ii = 1:length(this.convex_obstacles)
                        ob = this.convex_obstacles{ii};
                        plot([ob(:, 1); ob(1, 1)], [ob(:, 2); ob(1, 2)], '-k', 'LineWidth', 3);
                    end

                end

                hold on
                %temp = TForm.tform2vec(this.taskT);
               % Visualization.draw_poses(temp, 0.3, [1 0 0]);
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

        function draw_environment(self)

            hold on
            c = [0.4940 0.1840 0.4560]*0.3;

            for ii = 1:length(self.convex_obstacles)
                o = self.convex_obstacles{ii};
                plot(polyshape(o(:, 1:2)), 'EdgeColor', c, 'FaceAlpha', 1, 'FaceColor', c);
            end

        end

        function draw_task(self, varargin)
            hold on

            if ~isempty(varargin)
                self.taskTask.plot(varargin{:});
            else
                self.taskTask.plot();
            end

        end

    end

    methods (Static)

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
