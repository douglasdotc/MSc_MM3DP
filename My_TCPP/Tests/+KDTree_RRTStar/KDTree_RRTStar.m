cd 'E:\Google Drive HNC\_My Study\_UCL_Master_Robotics and Computation\Courses\_Dissertation\Code\MScMM3DP\My_TCPP'
close all
clear all
IsDEBUG = false;
if IsDEBUG
    figure(1);
end
hold on

%% Robot box
ywx   = 0.57; ywy = 0.36;
robot = [ywx / 2, ywy / 2, 1; ywx / 2, - ywy / 2, 1; -ywx / 2, - ywy / 2, 1; -ywx / 2, ywy / 2, 1]';

%% Create Printing Task
% Task = Tasks.StraightLinePath(0, 100);
PrintingTask = Tasks.LPath(100);
PrintingTask.smooth(0.075);
PrintingTask.scale([6,6,1])
PrintingTask.resample(0.01);
s        = PrintingTask.gett(0.01);
s        = s./max(s); % normalize
T        = PrintingTask.toTForm(PrintingTask);
T(3,4,:) = 0;
T        = TForm.tformX(T,TForm.DOWN);
% PrintingTask.plot();

%% Obstacles
Obstacles_Poly = Obstacles(IsDEBUG);
Obstacles_Poly = {};

%% Boundaries
% x_lim = [min(T(1, 4, :)) - 2, max(T(1, 4, :)) + 2];
% y_lim = [min(T(2, 4, :)) - 2, max(T(2, 4, :)) + 2];

%% Create Task Environment (C)
Env = ENV_SE2(PrintingTask, T, s, robot, Obstacles_Poly, IsDEBUG);

%% TEST
ite_arr  = [];
time_arr = [];
parfor idx = 1:1000
    %% Sample starting points
    num_node    = 50;
    start_nodes = Env.sample_pts(0, num_node);

    %% RRT*
    Env.ValidityCheck(start_nodes(1));
    RRTStar     = CLS_RRTStar(Env, start_nodes, CLS_KDTree);
    tic
        [path, ite] = RRTStar.RRT_Star;
    time = toc;
    ite_arr  = [ite_arr, ite];
    time_arr = [time_arr, time];
    disp([string(idx), ' in ', string(time), ' ite ', string(ite)])
end
hold off
scatter(ite_arr, time_arr)
hold on
xlabel('Number of iterations')
ylabel('Time (second)')