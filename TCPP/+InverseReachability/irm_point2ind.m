function ind = irm_point2ind(IRM, points)
    %given rm, convert point to index
    % Assert that grid is equidistant

    % Note this can be made faster without for loop

    assert(size(points, 2) == 4);
    assert(size(IRM.gridres, 2) == 4);
    n = size(points, 1);
    r = IRM.gridres / 2;
    ind = zeros(n, 1);

    for ii = 1:n
        xid = find(abs(points(ii, 1) - IRM.spans{1}) <= r(1), 1);
        yid = find(abs(points(ii, 2) - IRM.spans{2}) <= r(2), 1);
        zid = find(abs(points(ii, 3) - IRM.spans{3}) <= r(3), 1);
        thid = find(abs(points(ii, 4) - IRM.spans{4}) <= r(4), 1);

        if ~isempty(xid) &&~isempty(yid) &&~isempty(zid) &&~isempty(thid)
            ind(ii) = sub2ind(size(IRM.grid.X), xid, yid, zid, thid);
        else
            ind(ii) = nan;
            disp("irm_point2ind: point not in IRM")
        end

    end

end

%
