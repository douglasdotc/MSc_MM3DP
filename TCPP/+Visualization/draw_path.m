function draw_path(p, scale, c, varargin)
    % This creates xyz axes  ar points xyz and quat  pose.
    if isempty(varargin)
        plot(p(:, 1), p(:, 2), c);
        hold on
        quiver(p(:, 1), p(:, 2), cos(p(:, 3)), sin(p(:, 3)), scale, c);
        hold off
        return
    end

    if strcmp(varargin{1}, 'animate')

    end

    if strcmp(varargin{1}, 'connect')
        plot(p(:, 1), p(:, 2), '.r');
        hold on
        quiver(p(:, 1), p(:, 2), cos(p(:, 3)), sin(p(:, 3)), scale, c);
        hold on
        T = varargin{2};
        quiver(p(:, 1), p(:, 2), T(:, 1) - p(:, 1), T(:, 2) - p(:, 2), 0, 'k', 'ShowArrowHead', 'off')
        hold off
        return
    end

    if strcmp(varargin{1}, 'bodies')
        plot(p(:, 1), p(:, 2), '.r');
        hold on
        quiver(p(:, 1), p(:, 2), cos(p(:, 3)), sin(p(:, 3)), scale, c);
        hold on
        import robotics.core.internal.SEHelpers

        for ii = 1:size(p, 1)
            r = (SEHelpers.poseToTformSE2(p(ii, :)) * varargin{2}')';

            h = fill(r(:, 1), r(:, 2), 'r');
            set(h, 'FaceAlpha', 0.1);
        end

        return
    end

end
