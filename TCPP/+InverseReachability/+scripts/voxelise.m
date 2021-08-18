function Vox = voxelise(poses, vox_counts, varargin)
    % takes nx7 array of poses and creates a creates a 6D occupancy gridish
    % structure. vox_counts are amount of voxels along each dimension.

    % Currently this is based on comparison
    % Indexing based takes same amount of time

    eul = quat2eul(poses(:, 4:end), "XYZ"); % This should be RPY
    assert(all(mod(vox_counts(1:3), 2))); % should be odd
    assert(all(~mod(vox_counts(4:6), 2))); % should be even

    ranges = [max(abs(poses(:, 1:3)), [], 1)];
    radii = ranges ./ vox_counts(1:3);

    spans = cell(6, 1);

    for ii = 1:3
        c = (vox_counts(ii) + 1) / 2;
        lrange = fliplr(linspace(0, -(ranges(ii) - radii(ii)), c)); lrange = lrange(1:end - 1);
        rrange = linspace(0, ranges(ii) - radii(ii), c); rrange = rrange(2:end);
        assert(all(unique([lrange 0 rrange])' == [lrange 0 rrange]'))
        spans{ii} = unique([lrange 0 rrange])';
        assert(length(spans{ii}) == vox_counts(ii));
    end

    for ii = 4:6
        n = vox_counts(ii);
        nn = 0:n - 1;
        e = exp(nn .* 1i * pi * 2 / n);
        spans{ii} = atan2(imag(e), real(e));
        radii(ii) = spans{ii}(2) / 2; % alqyas second element
    end

    %% creating 4d mech grid vars
    [X, Y, Z, R, P, Yw] = ndgrid(spans{:});
    sz = size(X);
    n = length(X(:));
    Poses = cell(sz);
    VPoses = cell(sz);
    Vposes = cell(sz);
    Vrms = cell(sz);
    N = zeros(sz);

    WaitMessage = imports.parfor_wait(n, 'Waitbar', true);
    %Comparison based:

    if ~isempty(varargin)
        jnts = cell(sz);
        joint_solutions = varargin{1};
    end

    my_angdiff = @(t, s)atan2(sin(t - s), cos(t - s));

    parfor ii = 1:n
        lin_diffs = bsxfun(@minus, poses(:, 1:3), [X(ii), Y(ii), Z(ii)]);
        ang_diffs = bsxfun(my_angdiff, eul, [R(ii), P(ii), Yw(ii)]);
        logicals = bsxfun(@le, abs([lin_diffs ang_diffs]), radii);
        logicals = all(logicals, 2);
        N(ii) = sum(logicals);
        Poses{ii} = poses(logicals, :);
        VPoses{ii} = [X(ii), Y(ii), Z(ii), R(ii), P(ii), Yw(ii)];
        Vposes{ii} = [X(ii), Y(ii), Z(ii), eul2quat([R(ii), P(ii), Yw(ii)], 'XYZ')];
        rm = eul2rotm([R(ii), P(ii), Yw(ii)], 'XYZ');
        T = [rm, [X(ii); Y(ii); Z(ii)]; zeros(1, 3) 1];
        Vrms{ii} = T;

        if ~isempty(varargin)
            jnts{ii} = [joint_solutions(logicals, :)];
        end

        WaitMessage.Send;
    end

    WaitMessage.Destroy

    Vox.Poses = Poses; % This is a 4d cell array. empty cells means none of the poses lied in them. Otherse the cell is filled with mx7 array of poses inside it
    Vox.VPoses = VPoses; %6D array of poses xyz rpy
    Vox.Vposes = Vposes; %6D array of poses xyzqwqxqyqz
    Vox.Vrms = Vrms; %6D array of Transformation matrixes
    Vox.radii = radii;
    Vox.spans = spans;
    Vox.N = N; % int 4d array. each entry is how many solutions lie in that voxel

    if ~isempty(varargin)
        Vox.jnts = jnts; % int 4d array. each entry is how many solutions lie in that voxel
    end

end
