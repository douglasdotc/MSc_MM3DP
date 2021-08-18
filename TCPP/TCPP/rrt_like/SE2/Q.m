classdef Q < handle

    properties
        parent = {}
        children = {}
        q       
    end

    methods

        function this = Q(q)
            this.q = q;            
        end
      
        
        function r = minus(q1, q2)
            % Note the result is num vec           
            q1=QSE2.qmat(q1);
            q2=QSE2.qmat(q2);
            r = q1 - q2;
        end

        function q = add(q, inc)
            q.q=q.q+inc;
        end

    end

    methods (Static)

        function mat = qmat(qarr)
            assert(size(qarr, 2) == 1)%collumn vector
            mat = vertcat(qarr(:).q);
        end

        function q_new = new_from_vec(q_data)
            q_new=Q(q_data);          
        end

    end

end
