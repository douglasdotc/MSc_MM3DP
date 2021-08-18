function joint_tr = validate_solution(ikobj, task, solution_path, varargin)

    if isempty(varargin)
        locality = "local";
        visualise = true;
    else
        locality = varargin{1};
        visualise = varargin{2};
    end

    %% Discard repeating
    n = size(task, 3);
    t = linspace(0, 1, n);

    q = solution_path(1).qmat(solution_path');
    [~, uii, ~] = unique(q(:, 4));
    q = q(uii, :);
    q_dense = interp1(q(:, 4), q(:, 1:3), linspace(0, min(1, q(end, 4)), length(t))');

    %% Calc sols
    base_poses = [q_dense(:, 1:2) zeros(n, 1) eul2quat([zeros(n, 2) q_dense(:, 3)], 'XYZ')];
    base_T = vec2tform(base_poses);
    armT = tformX(tform2inv(base_T), task);
    default_settings = {"use_last", true, "timeout", 0.05, "attempts", 20};

    if locality == "local"
        ik_result = ikobj.get_ik(TForm.tform2vec(armT), "base_footprint", default_settings{:})
    elseif locality == "global"
        ik_result = ikobj.get_ik(TForm.tform2vec(task), "map", "base_jnts", q_dense(:, 1:3), default_settings{:})
    end

    %% Plot sols
    joint_tr = zeros(n, 10);

    for ii = 1:length(ik_result.sols_found)

        if isempty(ik_result.sols_found{ii}')
            joint_tr(ii, :) = missing;
        else
            joint_tr(ii, :) = ik_result.sols_found{ii}';
        end

    end

    if visualise
        figure
        subplot(2, 1, 1)
        plot(joint_tr(:, 1:3), '.-')
        subplot(2, 1, 2)
        plot(joint_tr(:, 4:end), '.-')
    end

end
