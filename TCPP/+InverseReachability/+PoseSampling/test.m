%% Docs 
import InverseReachability.PoseSampling.*
radius=0.6;
resolution=0.03;
n=30;
%%
poses=sample_cart_sphere_6dof(radius, resolution, n);
%% 5DOF 
poses=sample_cart_sphere_5dof(radius, resolution, n);
%% 4DOF Only down
poses=sample_cart_sphere_4dof(radius, resolution, n);
%% %%%%%%%%%%%% TESTS %%%%%%%%%
% Random viz
posestest=datasample(poses,100,'Replace',false);
Visualization.draw_poses(posestest,0.2,[1 0 0],'r')
hold on
Visualization.draw_poses(posestest,0.2,[0 1 0],'g')
Visualization.draw_poses(posestest,0.2,[0 0 1],'b')
hold off
%% Single point  viz
sample_point=poses(1000,:);
posestest=poses(sum([poses(:,1:3)-sample_point(1:3)].^2,2)<0.0001,:);
Visualization.draw_poses(posestest,0.2,[1 0 0],'r')
hold on
Visualization.draw_poses(posestest,0.2,[0 1 0],'g')
Visualization.draw_poses(posestest,0.2,[0 0 1],'b')
hold off