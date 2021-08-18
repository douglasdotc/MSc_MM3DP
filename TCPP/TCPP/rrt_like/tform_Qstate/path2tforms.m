function T = path2tforms(path)
    T = zeros(4, 4, length(path));

    for ii = 1:length(path)
        q = path(ii);
        T(:,:,ii) = [eul2rotm([q.q(3) 0 0]) [q.q(1:2) 0]'; 0 0 0 1];
    end

end
