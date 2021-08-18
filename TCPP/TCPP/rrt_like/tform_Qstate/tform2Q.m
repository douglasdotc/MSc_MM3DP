function q_vec = tform2Q(T, t)
    % Input SE3 transforms and t
    % output nx1 vec of QSE2T objects
    n = size(T, 3);

    if size(t, 1) == 1 && n ~= 1
        t = ones(n, 1) * t;
    end

    e = rotm2eul(T(1:3, 1:3, :));

    vec = [squeeze(T(1:2, 4, :))', e(:, 1), t];

    %     q_vec(n) = QSE2T([0 0 0  0]);

    for ii = 1:n
        q_vec(ii) = QSE2T(vec(ii, :));
    end

end
