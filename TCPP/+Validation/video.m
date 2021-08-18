load("Experiment_")
% load("Floorplans10")
% load("FloorTask")
% Experiment.T=T;
% Experiment.t=t;
% 
% path=path{2};
%% ROS
% rosshutdown
ypik = IK.YPIKObj();
%% Params
hz=20;
%% Load from Experiment
T=Experiment.T;
t=Experiment.t;
path=Experiment.path;
obstacles=Experiment.obstacles;

%% Compute joint solution (Given Experiment)
t=linspace(0,t(end),round(t(end)*hz));
b = path(1).qmat(path');
b(:, 3) = unwrap(b(:, 3));
b(:,4)=t(end)*b(:,4);
b = interp1(b(:, 4), b(:, 1:3), t,'pchip');
% b = interp1(b(:, 4), b(:, 1:3), t);
assert(~any(isnan(b(:))))
B = pathmat2tforms(b);
binv=TForm.tform2inv(B);

tp=TForm.tform2vec(T);
tp = interp1(linspace(0,t(end),size(tp,1)), tp(:, 1:3), t);
tp(:,4:7)=zeros(size(tp,1),4);
tp(:,4)=1;
TT=TForm.vec2tform(tp);

A = TForm.tformX(binv, TT);
A = TForm.tformX(A, TForm.DOWN);
a=squeeze(A(1:2,4,:))';
%% COnstraints
q=eul2quat([pi 0 0],'XYZ');
constraint=rosmessage("moveit_msgs/OrientationConstraint");
constraint.LinkName="extruder_ee";
constraint.Orientation.W=q(1);
constraint.Orientation.X=q(2);
constraint.Orientation.Y=q(3);
constraint.Orientation.Z=q(4);
constraint.AbsoluteXAxisTolerance=0.1;
constraint.AbsoluteYAxisTolerance=0.1;
constraint.AbsoluteZAxisTolerance=2*pi;
constraint.Header.FrameId="base_footprint";
constraint.Weight=1;
ypik.add_constraints(constraint);
%%
n=size(A,3);
nn=n;
poses = TForm.tform2vec(A);
poses=poses(1:nn,:);
a=a(1:nn,:);
b=b(1:nn,:);
tp=tp(1:nn,:);

default_settings = {"use_last", true, "timeout", 0.1, "attempts", 10};
% default_settings = {"use_last", false, "timeout", 0.1, "attempts", 10, 'seed',[-1.8021 1.2630 -0.3648 -1.5029 0.7531 2.6214 0.0179]};
% default_settings = {"use_last", true, "timeout", 0.05, "attempts", 20};

ik_result = ypik.get_ik(poses, "base_footprint", default_settings{:});
% joint_tr = zeros(nn, 10);
joint_tr = zeros(nn, 7);

for ii = 1:length(ik_result.sols_found)

    if isempty(ik_result.sols_found{ii}')
        joint_tr(ii, :) = missing;
    else
        joint_tr(ii, :) = ik_result.sols_found{ii}';
        
    end

end

for ii = 4:10
    joint_tr(:, 1) = unwrap(joint_tr(:, 1));
end

joint_tr(:, 1:3) = b(1:nn, :);
% joint_tr(:, 10) = 0;
%%
video_struct.joint_tr=joint_tr;
video_struct.T=T;
video_struct.tp=tp;
video_struct.a=a;
video_struct.b=b;
video_struct.hz=hz;
save("FloorPlan_video2","video")
%%
plot(joint_tr(:,4:end),'.-')

