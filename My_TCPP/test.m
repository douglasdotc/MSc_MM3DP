close all
clear all
IsDEBUG = true;
if IsDEBUG
    ax1 = subplot(1,2,1);
end
hold on

%% Robot box
ywx     = 0.57;
ywy     = 0.36;
robot   = [ ywx / 2,  ywy / 2, 1; 
            ywx / 2, -ywy / 2, 1; 
           -ywx / 2, -ywy / 2, 1; 
           -ywx / 2,  ywy / 2, 1]';

%% Create Printing Task
% PrintingTask = Tasks.StraightLinePath(0, 100);
PrintingTask = Tasks.LPath(100);
PrintingTask.smooth(0.075);
PrintingTask.scale([6,6,1])
PrintingTask.resample(0.01);
s        = PrintingTask.gett(0.01);
s        = s./max(s); % normalize
T        = PrintingTask.toTForm(PrintingTask);
T(3,4,:) = 0;
T        = TForm.tformX(T,TForm.DOWN);
PrintingTask.plot();
%% IRM
min_task_robot_dist = 0.15;
IRM                 = CLS_FakeIRM(min_task_robot_dist, IsDEBUG);
%% Obstacles
Obstacles_Poly = Obstacles(1.5, IsDEBUG);
% Obstacles_Poly = {};

%% Create Task Environment
Env                    = CLS_ENV_SE2(PrintingTask, T, s, robot, IRM, Obstacles_Poly, IsDEBUG);
IRM_overlap_threshold  = 0.5;
task_ROI_opening_angle = 180;
break_pts              = Env.Breakpoints_IRM_obs(IRM_overlap_threshold, task_ROI_opening_angle);
% break_pts              = [s(1); break_pts; s(end)];
ite_arr  = [];
time_arr = [];

if IsDEBUG
    axes(ax1);
end
sampling_intensity = 10;
kNN_num = 3;
for idx = 1:1
    FMTStar = CLS_FMTStar(Env, sampling_intensity, kNN_num);%, break_pts);
    tic
        [path, ite] = FMTStar.FMT_Star;
    time = toc;
    
    ite_arr  = [ite_arr, ite];
    time_arr = [time_arr, time];
    fprintf('%d, runtime: %f\n', idx, time);
end
% scatter(ite_arr, time_arr)
% xlabel('Number of iterations')
% ylabel('Runtime')