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
ss=sum(RM.voxValid);
t_s=tic
for ii = 142703:MM

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
     ang_calc=t/(jj-9308);
     [h, m, s] = hms(seconds(ang_calc*ssm));
   disp([jj / ss, h, m, s])

end
save("RM_complete22","RM")
%% Computing RI Normal zacharias
RM.voxRI= ri_zacharias(RM);
RM.voxRIz= ri_zacharias(RM);
RM.voxRIzpr= ri_zachpr(RM);
RM.voxRIzprnn= ri_zachprnn(RM);

%% Computing RI for task 
% 
% q=tform2quat(TForm.vec2tform(RM.LocalRIPoses));
% tq=tform2quat(TForm.DOWN);
% z=quatmultiply(quatconj(q),tq);
% a = 2*acosd(z(:,4));
% mask=a<60;
% sum(mask)


TT=TForm.vec2tform(RM.LocalRIPoses);
mask=acosd(min(1,squeeze(abs(TT(3,3,:)))))<60 & squeeze(TT(3,3,:)<0);
sum(mask)

sample_poses=RM.LocalRIPoses(mask,:);


% RM.voxRIRT= vertcat(cellfun(@(c) 100*sum(c)/params.sample_n ,RM.voxIKSuccess ));

%%

%         function generate(self, ikobj, filename)
%             %Computed this using KDL, timeout 0.1, attempts5. position only.
%             ypik = InverseReachability.ik.YPIKObj();
%             ik_result = ypik.get_ik(poses, "base_footprint");
%             reachable_positions = ik_result.ik_exists_vec;
%             experiment.reachable_positions = reachable_positions;
%
%         end
%%
% function poses = computeRI(self, voxel)
%
%             switch lower(method)
%                 case "zacharias"
%                     points = SpiralSampleSphere(n)
%                 case "noiseOnRT"
%                     points = SpiralSampleSphere(n)
%                 case "neighborhood"
%                     % take into account neighborhood.
%                 otherwise
%                     error(['Unexpected method: ' method])
%             end
%
%         end
%%
%         function ri = indexByPoint(self, point)
%             % point to index idx
%             % ri=voxel{x,y,z}{3}
%         end
%%

%% Visualise poses
if false
%     sample_poses = datasample(poses, 500, 'Replace', false);
    %     sample_poses = datasample(reachable_positions, 500, 'Replace', false);
    %     sample_poses = TForm.tform2vec(poses);
    %     sample_poses = TForm.tform2vec(poses);
    Visualization.draw_poses(sample_poses, 0.05, [1 0 0], 'r');
    hold on
    Visualization.draw_poses(sample_poses, 0.05, [0 1 0], 'g');
    Visualization.draw_poses(sample_poses, 0.05, [0 0 1], 'b');
    hold off
%     ros_helpers.send_rviz_poses(sample_poses, "base_footprint")
end
