function isin = irm_istformin(tforms)

    points = [squeeze(tforms(1:3, 4, :)) atan2(squeeze(tforms(2, 1, :)), squeeze(tforms(1, 1, :)))];
    disp("Debug assert, remove for speedup")
    assert(abs(squeeze(tforms(3, 1, :))) < 0.2);

    isin = ~isnan(irm_point2ind(IRM, points));

end
