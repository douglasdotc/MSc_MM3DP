function [tforms, smpl_ids] = irm_sampletform(IRM, n)

    cdf = cumsum(IRM.voxRI);
    s = cdf(end);
    smpl_ids = sum(bsxfun(@le, cdf', rand(n, 1) * s), 2) + 1;

    tforms = eul2tform([IRM.voxCenters(smpl_ids, 4) zeros(n, 2)], 'ZYZ');
    tforms(1:3, 4, :) = IRM.voxCenters(smpl_ids, 1:3)';
    disp("TEST assert remove")
    assert(IRM.voxRI(smpl_ids) > 0);

end
