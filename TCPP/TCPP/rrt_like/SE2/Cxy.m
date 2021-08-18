classdef Cxy < handle

    properties
        xlim
        ylim

    end

    methods

        function this = Cxy(xlim, ylim)
            this.xlim = xlim;
            this.ylim = ylim;
        end

        function is = iisValid(this, q)
            is = false;

            if ~(this.xlim(1) < q.q(1) && q.q(1) < this.xlim(2))
                disp("Should never be out of bounds x")
                return
            end

            if ~(this.ylim(1) < q.q(2) && q.q(2) < this.ylim(2))
                disp("Should never be out of bounds y")
                return
            end

            is = true;
            return

        end

        function qs = sample(this, varargin)
            %Sample 1 from C. until a valid one is returned

            if length(varargin) == 1
                qs = this.rand_q(varargin{1});
                while ~this.iisValid(qs)
                    qs = this.rand_q(varargin{1});
                end

            elseif isempty(varargin)
                qs = this.rand_q();

                while ~this.iisValid(qs)
                    qs = this.rand_q();
                end

            else
                disp("unexpected")
            end

        end

        function q = rand_q(this)
            q = Q([rand() * (this.xlim(2) - this.xlim(1)) + this.xlim(1), rand() * (this.ylim(2) - this.ylim(1)) + this.ylim(1)]);
        end

    end

    methods (Static)

        function m = dist(q1, q2)
            m = sqrt(sum((Q.qmat(q1) - Q.qmat(q2)).^2, 2));
        end

        function h = updraw_tree(h, q_vec, varargin)
            color = 'blue';

            if ~isempty(varargin)
                color = varargin{1};
            end

            if isempty(h)
                %Only draws vertices
                q=q_vec.qmat(q_vec);
                h = plot(q(:,1),q(:,2), '.r');
                return
            end

            h.XData = [h.XData q_vec.q(1)];
            h.YData = [h.YData q_vec.q(2)];
            line([q_vec.parent.q(1) q_vec.q(1)], [q_vec.parent.q(2) q_vec.q(2)], 'Color', color);
            drawnow
        end

    end

end
