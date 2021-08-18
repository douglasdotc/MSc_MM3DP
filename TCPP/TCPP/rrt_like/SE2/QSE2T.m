classdef QSE2T < QSE2

    properties
        iparent
        cost = 0
    end

    methods

        function this = QSE2T(q)
            this = this@QSE2(q);
        end

        function r = minus(q1, q2)
            % Note the result is num vec
            angdiff = @(t, s)atan2(sin(t - s), cos(t - s));
            q1 = QSE2.qmat(q1);
            q2 = QSE2.qmat(q2);
            r = [q1(:, 1:2) - q2(:, 1:2), angdiff(q1(:, 3), q2(:, 3)) q1(:, 4) - q2(:, 4)];
        end

    end

    methods (Static)

        function mat = qmat(qarr)
            assert(size(qarr, 2) == 1)%collumn vector
            mat = vertcat(qarr(:).q);

        end

        function q_new = new_from_vec(q_data)
            q_new = QSE2T(q_data);
        end

    end

end
