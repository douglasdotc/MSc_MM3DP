close all
clear all
IsDEBUG = false;
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

%% Sample starting points
ite              = 3000;
sq_norm_time_arr = [];
KDT_time_arr     = [];
parfor idxx = 1:ite
    Env   = ENV_SE2(PrintingTask, T, s, robot, Obstacles_Poly, IsDEBUG);
    nodes = Env.sample_pts(0, idxx);
    q_pt  = Env.sample_pts(0, 1);
    tic
        [NN, min_dist]      = CLS_KDTree.Nearest_Neighbour(q_pt, nodes(1), 'sq_norm');
    KDT_time = toc;
    tic
        [asq_val, asq_idx]  = min(dist_metric.method(q_pt, nodes,  'sq_norm'));
    sq_norm_time = toc;
    assert(min_dist == asq_val)
    sq_norm_time_arr = [sq_norm_time_arr, sq_norm_time];
    KDT_time_arr     = [KDT_time_arr, KDT_time];
    disp(idxx)
end
time = 1:ite;
scatter(1:ite, sq_norm_time_arr)
hold on 
scatter(1:ite, KDT_time_arr)
legend({'square norm', 'KD Tree with square norm'})
xlabel({'number of points'})
ylabel({'time (second)'})