classdef CSE2 < Cxy

    properties
        thlim = [-pi, pi]
    end

    methods

        function this = CSE2(xlim, ylim)
            this = this@Cxy(xlim, ylim);
        end

        function is = iisValid(this, q)
            is = iisValid@Cxy(this, q);

            if ~(this.thlim(1) < q.q(3) && q.q(3) < this.thlim(2))
                disp("Should never be out of bounds th lim")
                is = false;
                return
            end

            return
        end

        function q = rand_q(this)
            qxy = rand_q@Cxy(this);
            q = QSE2([qxy.q  rand() * (this.thlim(2) - this.thlim(1)) + this.thlim(1)]);
        end

    end

    methods (Static)

        function m = dist(q1, q2)
            qd = q1 - q2;          
            m = sqrt(sum((qd).^2, 2));
        end

        function h = updraw_tree(h, q_vec, varargin)
            color = 'blue';

            if ~isempty(varargin)
                color = varargin{1};
            end 

            if isempty(h)
                %Only draws vertices
                h = plot3(q_vec(:).q(1), q_vec(:).q(2), q_vec(:).q(3), '.r');
                return
            end

            h.XData = [h.XData q_vec.q(1)];
            h.YData = [h.YData q_vec.q(2)];
            h.ZData = [h.ZData q_vec.q(3)];
            line([q_vec.parent.q(1) q_vec.q(1)], [q_vec.parent.q(2) q_vec.q(2)], [q_vec.parent.q(3) q_vec.q(3)], 'Color', color);
            drawnow
        end
        
       function draw_full_path(path)           
                figure
                path=vertcat(path.q);
                hold on
                q=quiver(path(:,1),path(:,2),cos(path(:,3)),sin(path(:,3)),0.1);
                plot(path(:,1),path(:,2),'-r');
                quiver(path(1,1),path(1,2),cos(path(1,3)),sin(path(1,3)),0.2,'blue')
                quiver(path(end,1),path(end,2),cos(path(end,3)),sin(path(end,3)),0.2,'green')
                xlabel('x')
                ylabel('y')             
              
        end

    end

    methods (Static, Access = private)

    end

end
