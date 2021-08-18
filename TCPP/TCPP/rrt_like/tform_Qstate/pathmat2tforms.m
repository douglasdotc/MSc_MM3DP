function T = pathmat2tforms(pathmat)
    T = zeros(4, 4, length(pathmat));

    for ii = 1:length(pathmat)
        q = pathmat(ii, :);
        T(:, :, ii) = [eul2rotm([q(3) 0 0]) [q(1:2) 0]'; 0 0 0 1];
    end

end
