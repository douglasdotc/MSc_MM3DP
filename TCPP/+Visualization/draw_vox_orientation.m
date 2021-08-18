function draw_vox_orientation(Vox, varargin)

    tforms = cat(3, Vox.Vrms{~cellfun(@isempty, Vox.Poses)});
    x = squeeze(tforms(1, 4, :));
    y = squeeze(tforms(2, 4, :));
    z = squeeze(tforms(3, 4, :));
    ax = squeeze(tforms(1, 1, :));
    ay = squeeze(tforms(2, 1, :));
    az = squeeze(tforms(3, 1, :));
    az=round(az,3);
    quiver3(x, y, z, ax, ay, az)
