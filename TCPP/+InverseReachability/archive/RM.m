classdef RM < handle

    properties
        voxwidth
        frame
        voxels% {center 3x1, test_poses nx7, success nx1 logical , solutions ?x10 }
        spans%{}3x1 of
        centers%
    end

    methods

        function self = RM(ikobj, varargin)
            self.ikobj = ikobj;
            self.voxwidth = 0.05;
            self.radius = [self.ikobj.reach, self.ikobj.reach, self.ikobj.reach];
            idx_rad = ceil(self.radius ./ self.voxwidth);
            d = (-idx_rad:idx_rad) * self.voxwidth;
            self.frame = "base_footprint";

        end

        function discard_voxels(self, ikobj)
            %remove test poses of  bad voxels
        end

        function generate(self, ikobj, filename)
            %Computed this using KDL, timeout 0.1, attempts5. position only.
            ypik = InverseReachability.ik.YPIKObj();
            ik_result = ypik.get_ik(poses, "base_footprint");
            reachable_positions = ik_result.ik_exists_vec;
            experiment.reachable_positions = reachable_positions;

        end

        function poses = computeRI(self, voxel)

            switch lower(method)
                case "zacharias"
                    points = SpiralSampleSphere(n)
                case "noiseOnRT"
                    points = SpiralSampleSphere(n)
                case "neighborhood"
                    % take into account neighborhood.
                otherwise
                    error(['Unexpected method: ' method])
            end

        end

        function ri = indexByPoint(self, point)
            % point to index idx
            % ri=voxel{x,y,z}{3}
        end

        methods (Static)

            function poses = genRIPoses(method, n, varargin)
                % gen local poses for RI computation
                origin = eye(4);

                switch lower(method)
                    case "zacharias"
                        points = SpiralSampleSphere(n) * (self.voxwidth / 2);
                        R = eul2rotm([atan2(points(:, 2), points(:, 1)), -atan2(points(:, 3), sqrt(sum(points(:, 1:2).^2, 2))), zeros(n, 1)]);
                        R = rotm2tform(R);
                        R = TForm.tformX(R, rotm2tform(eul2rotm([0 pi / 2 0], 'ZYZ')));
                        R = TForm.tformX(R, eul2tform([linspace(0, 2 * pi, n)' zeros(n, 1) zeros(n, 1)], 'ZYZ'));
                        poses = [points tform2quat(R)];
                    case "noisegrid"
                        %Could make noise function of vox width
                        R_T = varargin{1};
                        noise = [0.025, 0.025, 0.025, deg2rad(30), deg2rad(30), 2 * pi];
                        perp = zeros(6, n);

                        for ii = 1:6
                            perp(ii, :) = linspace(-noise(ii), noise(ii), n);
                        end

                        [X, Y, Z, R, P, Y] = ndgrid(perp(1, :), perp(2, :), perp(3, :), perp(4, :), perp(5, :), perp(6, :));
                        m = size(X(:), 1);
                        poses = zeros(m, 7);
                        poses(:, 1:3) = [X(:), Y(:), Z(:)];
                        poses(:, 4:7) = eul2quat([R(:), P(:), Y(:)], 'XYZ');
                        poses = TForm.tform2vec(TForm.tformX(R_T, TForm.vec2tform(poses)));
                        disp("WARNING: set size is n^6");
                    otherwise
                        error(['Unexpected method: ' method]);
                end

            end

        end

    end

end
