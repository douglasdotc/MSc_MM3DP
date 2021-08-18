[VoxIRM, VoxRM, ik_result]=load_experiment();

%% Visualisation Matlab I:
figure("Color",[1 1 1]);
%%
subplot(2, 2, 2)
title("IRM")
Visualization.draw_vox_occupancy(VoxIRM)
subplot(2, 2, 1)
title("RM")
Visualization.draw_vox_occupancy(VoxRM)

%%  Visualisation Matlab II:
subplot(2, 2, 4)
title("IRM")
Visualization.draw_vox_orientation(VoxIRM)
subplot(2, 2, 3)
title("RM")
Visualization.draw_vox_orientation(VoxRM)
%%
rosshutdown
Visualization.Tuples2RVizSpheres(VoxRM,'scale',[.03, .03, .03],'opacity',0.5,'id',0,'frame','base_footprint')
Visualization.Tuples2RVizSpheres(VoxIRM,'scale',[.03, .03, .03],'opacity',0.5,'id',1)
%%



function [VoxIRM, VoxRM, ik_result]=load_experiment()
load(uigetfile)
VoxIRM=experiment.VoxIRM;
VoxRM=experiment.VoxRM;
ik_result=experiment.ik_result;
end