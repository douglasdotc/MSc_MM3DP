cd 'E:\Google Drive HNC\_My Study\_UCL_Master_Robotics and Computation\Courses\_Dissertation\Code\MSc_MM3DP\My_TCPP'
close all
clear all

IsDEBUG = true;
hold on

%% Robot box
ywx     = 0.57;
ywy     = 0.36;
robot   = [ ywx / 2,  ywy / 2, 1; 
            ywx / 2, -ywy / 2, 1; 
           -ywx / 2, -ywy / 2, 1; 
           -ywx / 2,  ywy / 2, 1]';

%% Create Printing Task
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
T          = TForm.tformX(T,TForm.DOWN);

%% IRM
min_task_robot_dist = 0.15;
IRM                 = CLS_FakeIRM(min_task_robot_dist, IsDEBUG);

%% TEST
num_node               = 200;
IRM_overlap_threshold  = 0.5;
for obs_config_idx = 1:5
    for tdx = 1:5
        file_name                               = "RRT_Tests_Obstacle_Config_"+string(obs_config_idx)+"_T"+string(tdx);
        [Obstacles_Poly, Obstacle_break_Poly]   = CLS_Obstacles.Obstacle_Config_select(obs_config_idx, IsDEBUG);
%         Obstacle_break_Poly         = {};
        Env                                     = CLS_ENV_SE2(PrintingTask, T, s, robot, IRM, Obstacles_Poly, Obstacle_break_Poly, IsDEBUG);
        fprintf("Finding breakpoints...");
        break_pts                               = Env.Breakpoints_IRM_obs(IRM_overlap_threshold);
        break_pts                               = [s(1); break_pts; s(end)];
        fprintf("done\n");

        %% Sample starting points
        fprintf("Sampling start points...");
        start_nodes = Multi_sample(Env, break_pts, num_node, false);
        fprintf("done\n");

        %% RRT*
        RRTStar     = CLS_2DRRTStar(Env, start_nodes, break_pts, file_name);
        [path, ite] = RRTStar.RRT_Star;
    end
end
hold off