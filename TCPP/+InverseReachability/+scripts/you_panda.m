%% Imports
import InverseReachability.*
import InverseReachability.PoseSampling.*
%% %%%%%% Params %%%%%%
params.sampling_func = @InverseReachability.PoseSampling.sample_cart_sphere_4dof;
params.sampling_radius = 1;
params.sampling_resolution = 0.025;
params.sampling_n = 50;

params.robot_ikobj = InverseReachability.ik.YPIKObj(); %Something that you give params to and poses, spits out IKs
params.robot_constraints = [];

params.samplingbias_obj = @IRMObj; %Something that you give poses and iks, spits out a Sampling Bias object
params.samplingbias_vox_size = [31 31 1 2 2 30];

params.trial_name = "_";
params.trial_description = "_";
experiment.params = params;

%% %%%%%% Gererate poses and compute IK %%%%%%
poses = params.sampling_func(params.sampling_radius, params.sampling_resolution, params.sampling_n);
poses = poses(abs(poses(:, 3)) < 0.01, :); % temphack --> Select ground level only to save time.
poses = poses + repmat([params.robot_ikobj.base_to_arm zeros(1, 4)], size(poses, 1), 1);

%% Visualise poses
if false
    sample_poses = datasample(poses, 500, 'Replace', false);
    Visualization.draw_poses(sample_poses, 0.05, [1 0 0], 'r');
    hold on
    Visualization.draw_poses(sample_poses, 0.05, [0 1 0], 'g');
    Visualization.draw_poses(sample_poses, 0.05, [0 0 1], 'b');
    hold off
    ros_helpers.send_rviz_poses(sample_poses, "base_footprint")
end

%% Compute ik
ik_result = params.robot_ikobj.get_ik(sample_poses, "base_footprint");
experiment.ik_result = ik_result;

%% %%%%%% Voxelise into IRM %%%%%%
poses = ik_result.ik_exists_vec;
RM = TForm.vec2tform(poses);
IRM = TForm.tform2inv(RM);
iposes = TForm.tform2vec(IRM);

%Voxelise
VoxIRM = voxelise(iposes, params.samplingbias_vox_size, horzcat(ik_result.sols_found{:})'); %xyzrpy
VoxRM = voxelise(poses, params.samplingbias_vox_size, horzcat(ik_result.sols_found{:})'); %xyzrpy
experiment.VoxIRM = VoxIRM;
experiment.VoxRM = VoxRM;
save(params.trial_name + "_exp", "experiment");

%% Visualisation I:
subplot(2, 2, 2)
title("IRM")
Visualization.draw_vox_occupancy(VoxIRM)
subplot(2, 2, 1)
title("RM")
Visualization.draw_vox_occupancy(VoxRM)

%%
subplot(2, 2, 4)
title("IRM")
Visualization.draw_vox_orientation(VoxIRM)
subplot(2, 2, 3)
title("RM")
Visualization.draw_vox_orientation(VoxRM)
