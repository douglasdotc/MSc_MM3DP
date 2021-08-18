function isin = irm_isse2pin(IRM, se2p, point)
    % given a se2p, is it inside the IRM for point.
    %     points = [squeeze(tforms(1:3, 4, :))' atan2(squeeze(tforms(2, 1, :)), squeeze(tforms(1, 1, :)))];
    %
    %
    %     % disp("Debug assert, remove for speedup")
    %     % assert(all(abs(squeeze(tforms(3, 1, :))) < 0.2));
    %     assert(all(abs(points(:,3))< 0.001));
    %     assert(all(tforms(3, 3, :) > 0));

    z = find(abs(point(3) - IRM.spans{3}) <= IRM.gridres(3) / 2, 1);
    se2p_local = se2p;
    se2p_local(1) = se2p_local(1) - point(1);
    se2p_local(2) = se2p_local(2) - point(2);
    ind = InverseReachability.irm_point2ind(IRM, [se2p_local(1:2) point(3) se2p_local(3)]);

    if isnan(ind)
        isin = false;
    else
        isin = IRM.voxRI(ind) > IRM.layers{z}.cutoff;
    end

end
