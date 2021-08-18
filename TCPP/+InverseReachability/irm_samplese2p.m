function [se2ps, smpl_ids] = irm_samplese2p(IRM, n, point)
    %given a task point xyz, sample from appropriate IRM to receive base pose xyth
    z = find(abs(point(3) - IRM.spans{3}) <= IRM.gridres(3) / 2, 1);
    % z should never be empty unless robot out of reach vertically

    layer = IRM.layers{z};
    s = layer.cdf(end);
    smpl_ids = sum(bsxfun(@le, layer.cdf', rand(n, 1) * s), 2) + 1;
    se2ps = IRM.voxCenters(layer.inds(smpl_ids), [true true false true]);
    se2ps(:, 1) = se2ps(:, 1) + point(1);
    se2ps(:, 2) = se2ps(:, 2) + point(2);
    % disp("debug  assert, delete for performance")
    % assert(all(abs(IRM.voxCenters(layer.inds(smpl_ids), 3) - IRM.spans{3}(z)) < 0.001))

    % %%
    % tforms = eul2tform([IRM.voxCenters(smpl_ids, 4) zeros(n, 2)], 'ZYZ');
    % tforms(1:3, 4, :) = IRM.voxCenters(smpl_ids, 1:3)';
    % %     assert(all(tforms(3, 3, :) > 0));
    % % disp("TEST assert remove")
    % % assert(all(IRM.voxRI(smpl_ids) > 0));
    % %% TODO gonna keep it this way buyt this hsould not spit out tforms

end
