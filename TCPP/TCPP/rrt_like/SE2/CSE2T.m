classdef CSE2T < CSE2

    properties
        tlim = [0, 1]
    end

    methods

        function this = CSE2T(xlim, ylim)
            this = this@CSE2(xlim, ylim);
        end

        function is = iisValid(this, q)
            is = iisValid@Cxy(this, q);

            if ~(this.tlim(1) <= q.q(4) && q.q(4) <= this.tlim(2))
                disp("Should never be out of bounds t")
                disp(q.q(4))
                is = false;
                return
            end

            return
        end

        function q = rand_q(this)
            qse2 = rand_q@CSE2(this);
            q = QSE2T([qse2.q rand() * (this.tlim(2) - this.tlim(1)) + this.tlim(1)]);
        end

        function h = updraw_tree(this, h, q_vec, varargin)
            color = 'blue';

            if ~isempty(varargin) &&~isempty(varargin{1})
                color = varargin{1};
            end

            if isempty(h)
                %Only draws vertices
                h = plot3(q_vec(:).q(1), q_vec(:).q(2), q_vec(:).q(4), '.r');
                return
            end

            h.XData = [h.XData q_vec.q(1)];
            h.YData = [h.YData q_vec.q(2)];
            h.ZData = [h.ZData q_vec.q(4)];
            line([q_vec.parent.q(1) q_vec.q(1)], [q_vec.parent.q(2) q_vec.q(2)], [q_vec.parent.q(4) q_vec.q(4)], 'Color', color);
            drawnow
        end

    end

    methods (Static)

        function m = dist(q1, q2)
            qd = q1 - q2; % NOTE: THis includes angle difference. minus is overloaded
            m = sqrt(sum((qd(:, 1:3)).^2, 2) .* (qd(:, 4) >= 0)); %normal
            m(qd(:, 4) <= 0 | qd(:, 4) > .3) = inf;
            % NOTE This is to fight the issue that when dealing with shapes that loop onto themselves  nearest neighbor should find t. %TODO consider adding path length along  progress time. I probably should not have this here.

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
