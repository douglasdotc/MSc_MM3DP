classdef IRM < handle

    properties
        ikobj% ik object to call for iks
        reach% arm reach scalar
        taskrm%rotmat of task 3x3
        noiseamp% matrix for perturbations to explore 1x6 logical

    end

    methods

        function self = TaskAbilityMap()
            self.check_ros_setup()%Check that ros network is up
        end

    end

    methods (Static)

        function poses = getVoxelTestSet(method)

            points = spheretri(42);
            * deg2rad(15);
            points(:, 3) = 0
            poses = [zeros(42, 3) eul2quat(points, 'XYZ')];

            Visualization.draw_poses(poses, 0.1, [1 0 0], 'r');
            hold on
            Visualization.draw_poses(poses, 0.1, [0 1 0], 'g');
            Visualization.draw_poses(poses, 0.1, [0 0 1], 'b');
            hold off

        end

    end

end
