close all
clear all
IsDEBUG = true;
if IsDEBUG
%     ax1 = figure(1);
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
% % PrintingTask = Tasks.StraightLinePath(0, 100);
% PrintingTask = Tasks.LPath(100);
% PrintingTask.smooth(0.075);
% PrintingTask.scale([6,6,1])
% PrintingTask.resample(0.01);
% s        = PrintingTask.gett(0.01);
% s        = s./max(s); % normalize
% T        = PrintingTask.toTForm(PrintingTask);
% T(3,4,:) = 0;

p = Tasks.HPath(2, 100);
p.smooth(0.075);
p.scale([3, 3, 1]);
p.resample(0.001);
q = Tasks.UPath(.1);
q.scale([0.2 0.05 1]);
p.superimpose(q) 
PrintingTask=p;
PrintingTask.resample(0.01);
s = PrintingTask.gett(0.01);
s = s./max(s);
T = PrintingTask.toTForm(PrintingTask);
T(3, 4, :) = 0;

T        = TForm.tformX(T,TForm.DOWN);
% PrintingTask.plot();
% box on
% hold on
% xlabel('x (m)')
% ylabel('y (m)')

%% IRM
min_task_robot_dist = 0.15;
IRM                 = CLS_FakeIRM(min_task_robot_dist, IsDEBUG);

%% Obstacles
Obstacles_Poly = Obstacles(1.5, false);
Obstacles_Poly = {};
% drawnow
%% Create Task Environment
Env                    = CLS_ENV_SE2(PrintingTask, T, s, robot, IRM, Obstacles_Poly, IsDEBUG);
IRM_overlap_threshold  = 0.5;
task_ROI_opening_angle = 180;
% break_pts              = Env.Breakpoints_IRM_obs(IRM_overlap_threshold, task_ROI_opening_angle);
% break_pts              = [s(1); break_pts; s(end)];
ite_arr  = [];
time_arr = [];
sampling_intensity_rec = [];
r_search_rec = [];
path_len_rec = [];
% if IsDEBUG
%     axes(ax1);
% end

% sampling_intensity  = 1;
% r_search            = 0.1;
max_trial           = 10;
parfor sampling_intensity = 1:20
    for r_search = 0.1:0.05:0.5
        ax1 = figure(1); %
        PrintingTask.plot(); %
        box on %
        hold on %
        xlabel('x (m)') %
        ylabel('y (m)') %
        drawnow %
        FMTStar     = CLS_2DFMTStar(Env, sampling_intensity, r_search, max_trial);
        [path, ite, cost, time, record] = FMTStar.FMT_Star;
        saveas(ax1, "FMT_sample_int_"+string(sampling_intensity)+"_r"+string(r_search)+".fig")
        ite_arr  = [ite_arr, ite];
        time_arr = [time_arr, time];
        r_search_rec = [r_search_rec, r_search];
        sampling_intensity_rec = [sampling_intensity_rec, sampling_intensity];
        path_len_rec = [path_len_rec, length(path)];
        hold off %
        delete(ax1); %
    end
end
% scatter(ite_arr, time_arr)
% xlabel('Number of iterations')
% ylabel('Runtime')