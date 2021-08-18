%% Imports
import IK.*
import InverseReachability.*
%% %%%%%% Params %%%%%%
params.gridrad = 1.5;
params.gridres = 0.05;
params.sample_n = 200;
params.ikobj = @YPIKObj;
params.frame = "base_footprint";


params.trial_name = "Wednesday";
params.trial_description = "Clearly the speed has affected!!";
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
%% Find valid voxels. NOTE KDL with POSITION ONLY SHOULD BE RUNNING
POSES = repmat(TForm.DOWN, 1, 1, MM);
POSES(1:3, 4, :) = RM.voxCenters';
sample_poses = TForm.tform2vec(POSES);
ikobj = params.ikobj();
result = ikobj.get_ik(sample_poses, "base_footprint");
RM.voxValid = result.received_answers == 1;
save(params.trial_name,"RM")
%% RI Poses
RM.LocalRIPoses = InverseReachability.zacharias(params.gridres / 2, params.sample_n);
% RM.LocalRIPoses = InverseReachability.double_zacharias(params.gridres / 2, params.sample_n);

%% Compute Voxel Ik Calls. NOTE::: THIS MUST BE Trac ik now. with Distance.
ikobj = params.ikobj();
% 142704
% 150899
% 157540
% 157655
%%% 183941
ss=sum(RM.voxValid);
t_s=tic
for ii = 183940:MM

    if RM.voxValid(ii)

        sample_poses = RM.LocalRIPoses;
        sample_poses(:, 1:3) = sample_poses(:, 1:3) + repmat(RM.voxCenters(ii, :), params.sample_n, 1);
        result = ikobj.get_ik(sample_poses, "base_footprint");
        RM.voxRIPoses{ii} = sample_poses;
        RM.voxIKSuccess{ii} = result.received_answers == 1;
        RM.voxIKSolutions{ii} = result.sols_found;       
    end
    
     t = toc(t_s);    
     jj=sum(RM.voxValid(1:ii));
     ssm=ss-jj;
     ang_calc=t/(jj-19495);
     [h, m, s] = hms(seconds(ang_calc*ssm));
   disp([jj / ss, h, m, s])

end
save("RM_complete_Finally","RM")
%% Computing RI Normal zacharias

RM.voxRIz= ri_zacharias(RM);
RM.voxRIzpr= ri_zachpr(RM);
RM.voxRIzprnn= ri_zachprnn(RM);
RM.voxRI=RM.voxRIz;

