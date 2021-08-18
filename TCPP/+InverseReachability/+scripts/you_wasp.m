%% Imports
import InverseReachability.*
import InverseReachability.PoseSampling.*
%% Params
params.sampling_func=@InverseReachability.PoseSampling.sample_cart_sphere_4dof;
params.sampling_radius=1.5;
params.sampling_resolution=0.025;
params.sampling_n=100;

params.robot_ikobj=@YPIKObj;%Something that you give params to and poses, spits out IKs
params.robot_constraints=[];

params.samplingbias_obj=@IRMObj;%Something that you give poses and iks, spits out a Sampling Bias object
params.samplingbias_vox_size=[35 35  1  2  2 78];

params.trial_name="Sunday18_panda_one_plane_redo_vox";
params.trial_description="Gonna pick a negative z and just sample that distance. do 4dof. Redid vox from 41 41  1  2  2 78 to 35 35  1  2  2 78";
experiment.params=params
%% Gererate poses and compute IK
poses = params.sampling_func(params.sampling_radius,params.sampling_resolution,params.sampling_n);
poses=poses(abs(poses(:,3)-(-0.0750))<0.01,:);
poses=poses+repmat([0.167 0 0.142 zeros(1,4)],size(poses,1),1);

%%
ypik=ik.YPIKObj();
ik_result=ypik.get_ik(poses,"base_link");
experiment.ik_result=ik_result;

%% Take IK and 
poses=result.ik_exists_vec;
RM=TForm.vec2tform(poses);
IRM=TForm.tform2inv(RM); 
iposes=TForm.tform2vec(IRM);

%Voxelise
VoxIRM=voxelise(iposes,params.samplingbias_vox_size,horzcat(result.sols_found{:})');%xyzrpy
VoxRM=voxelise(poses,params.samplingbias_vox_size,horzcat(result.sols_found{:})');%xyzrpy
experiment.VoxIRM=VoxIRM;
experiment.VoxRM=VoxRM;
save(params.trial_name + "_exp","experiment");



%% Visualisation :
subplot(1,2,1)
Visualization.draw_vox_occupancy(VoxIRM)
subplot(1,2,2)
Visualization.draw_vox_occupancy(VoxRM)

%%
figure
subplot(1,2,1)
Visualization.quiver_vox_all(VoxIRM)
subplot(1,2,2)
Visualization.draw_vox_occupancy(VoxRM)