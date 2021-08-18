%% fake RM
%% Imports
import IK.*
import InverseReachability.*
%% %%%%%% Params %%%%%%
params.gridrad = 1.5;
params.gridres = 0.05;
params.sample_n = 200;
params.ikobj = @YPIKObj;
params.frame = "base_footprint";


params.trial_name = "Monday25_night_new_method2";
params.trial_description = "Doing Zacharias straight up";
RM.params = params;
%% create grid
idx_rad = ceil(params.gridrad ./ params.gridres);
d = (-idx_rad:idx_rad) * params.gridres;
M = length(d);
MM = M^3;
[X, Y, Z] = meshgrid(d, d, d);
RM.spans = {d, d, d};
RM.grid.X = X;
RM.grid.Y = Y;
RM.grid.Z = Z;
%% create voxels
RM.voxCenters = [X(:), Y(:), Z(:)];
RM.voxRIPoses = cell(M^3, 1);
RM.voxIKSuccess = cell(M^3, 1);
RM.voxIKSolutions = cell(M^3, 1);
RM.voxValid = false(MM, 1);
RM.voxRI = zeros(MM,1);
%%
RM.voxValid = (rand(MM,1)<0.1);
%%

for ii = 1:MM

    if RM.voxValid(ii)

        sample_poses = RM.LocalRIPoses;
        sample_poses(:, 1:3) = sample_poses(:, 1:3) + repmat(RM.voxCenters(ii, :), params.sample_n, 1);        
        RM.voxRIPoses{ii} = sample_poses;
        RM.voxIKSuccess{ii} = rand(size(sample_poses,1),1)<0.2;
        RM.voxIKSolutions{ii} = cell(n,1);

    end
disp("||||||||||||||")
ii/MM
disp("||||||||||||||")
end
