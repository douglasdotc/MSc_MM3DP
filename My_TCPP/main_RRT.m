cd 'E:\Google Drive HNC\_My Study\_UCL_Master_Robotics and Computation\Courses\_Dissertation\Code\MSc_MM3DP\My_TCPP'
close all
clear all

IsDEBUG = true;
if IsDEBUG
    ax1 = figure(1);
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
% PrintingTask = Tasks.HPath(2, 100);
PrintingTask.smooth(0.075);
PrintingTask.scale([6,6,1])
PrintingTask.resample(0.01);
s        = PrintingTask.gett(0.01);
s        = s./max(s); % normalize
T        = PrintingTask.toTForm(PrintingTask);
T(3,4,:) = 0;


% p = Tasks.HPath(2, 100);
% p.smooth(0.075);
% p.scale([3, 3, 1]);
% p.resample(0.001);
% q = Tasks.UPath(.1);
% q.scale([0.2 0.05 1]);
% p.superimpose(q) 
% PrintingTask=p;
% PrintingTask.resample(0.01);
% s = PrintingTask.gett(0.01);
% s = s./max(s);
% T = PrintingTask.toTForm(PrintingTask);
% T(3, 4, :) = 0;


T        = TForm.tformX(T,TForm.DOWN);
PrintingTask.plot();
box on
hold on
xlabel('x (m)')
ylabel('y (m)')

%% Boundaries
% x_lim = [min(T(1, 4, :)) - 2, max(T(1, 4, :)) + 2];
% y_lim = [min(T(2, 4, :)) - 2, max(T(2, 4, :)) + 2];

%% IRM
min_task_robot_dist = 0.15;
IRM                 = CLS_FakeIRM(min_task_robot_dist, IsDEBUG);

%% Decomposition TEST:
% mean_time = [];
% num_break_pts_w = [];
% adj_w = 0.01:0.01:3;
% for idx = 1:size(adj_w, 2)
%     num_break_pts_adj_w = [];
%     time_adj_w          = [];
%     for jdx = 1:10
%         %% Obstacles
%         Obstacles_Poly = Obstacles(adj_w(idx), IsDEBUG);
%         % Obstacles_Poly = {};
% 
%         %% Create Task Environment
%         Env                    = CLS_ENV_SE2(PrintingTask, T, s, robot, IRM, Obstacles_Poly, IsDEBUG);
%         IRM_overlap_threshold  = 0.5;
%         task_ROI_opening_angle = 180;
%         tic
%             break_pts              = Env.Breakpoints_IRM_obs(IRM_overlap_threshold, task_ROI_opening_angle);
%         time = toc;
%         time_adj_w             = [time_adj_w; time];
%         num_break_pts_adj_w    = [num_break_pts_adj_w; size(break_pts, 1)];
%     end
%     mean_time       = [mean_time; mean(time_adj_w)];
%     num_break_pts_w = [num_break_pts_w; mean(num_break_pts_adj_w)];
%     fprintf('Width %d gives %d break points on average in %d.\n', adj_w(idx), mean(num_break_pts_adj_w), mean(time_adj_w));
% end

%% Obstacles
Obstacles_Poly = Obstacles(1.5, IsDEBUG);
% Obstacles_Poly = {};

%% Create Task Environment
Env                    = CLS_ENV_SE2(PrintingTask, T, s, robot, IRM, Obstacles_Poly, IsDEBUG);
% break_pts              = Env.Breakpoints_Sampling(100);
% break_pts              = Env.Breakpoints_obs_intersect();
% break_pts              = Env.Breakpoints_IRM();
% break_pts              = Env.Breakpoints_obs_task();

IRM_overlap_threshold  = 0.5;
task_ROI_opening_angle = 180;
% break_pts              = Env.Breakpoints_IRM_obs(IRM_overlap_threshold, task_ROI_opening_angle);
% break_pts              = [s(1); break_pts; s(end)];
%% TEST
% if IsDEBUG
%     axes(ax1);
% end

ite_arr  = [];
time_arr = [];
num_node = 200;
for idx = 1:1
    %% Sample starting points
%     num_node    = 200;
%     start_nodes = Env.sample_pts(0, num_node);
%     for jdx = 1:length(break_pts) - 1
%         start_nodes{jdx} = Env.sample_pts(break_pts(jdx), num_node);
%         if IsDEBUG
%             poses = cat(1, start_nodes{jdx}.pose);
%             scatter(poses(:,1), poses(:,2))
%             quiver(poses(:,1), poses(:,2), poses(:,3), poses(:,4))
%         end
%     end

    % start_nodes = Multi_sample(Env, break_pts, num_node, false);
    start_nodes{1} = Env.sample_pts(0, num_node); % TEMP
    start_nodes{2} = Env.sample_pts(1, num_node); % TEMP
    %% RRT*
%     RRTStar     = CLS_2DRRTStar(Env, start_nodes); % , break_pts);
    BRRTStar = CLS_2DBRRTStar(Env, start_nodes); % , break_pts);
    tic
%         [path, ite] = RRTStar.RRT_Star;
        [path, ite] = BRRTStar.BRRT_Star;
    time = toc;
    ite_arr  = [ite_arr, ite];
    time_arr = [time_arr, time];
    fprintf('%d in %.3f seconds, %d iterations.\n', idx, time, ite)
end
hold off
% scatter(ite_arr, time_arr)
% hold on
% xlabel('Number of iterations')
% ylabel('Time (second)')