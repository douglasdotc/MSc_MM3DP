classdef QSE2 < Q

    properties
       
    end

    methods

        function this = QSE2(q)
            this = this@Q([q(1:2) wrapToPi(q(3)) q(4:end) ]);            
        end

        function r = minus(q1, q2)
            % Note the result is num vec
            angdiff = @(t, s)atan2(sin(t - s), cos(t - s));
            q1=QSE2.qmat(q1);
            q2=QSE2.qmat(q2);
            r = [q1(:,1:2) - q2(:,1:2),  .4*angdiff(q1(:,3), q2(:,3))];
        end

        function q = add(q, inc)
            q.q=q.q+inc;
            q.q(3)=wrapToPi(q.q(3));
        end

    end

    methods (Static)

        function mat = qmat(qarr)
            assert(size(qarr, 2) == 1)%collumn vector
            mat = vertcat(qarr(:).q);
        end

        function q_new = new_from_vec(q_data)
            q_new=QSE2(q_data);          
        end
        

    end

end
