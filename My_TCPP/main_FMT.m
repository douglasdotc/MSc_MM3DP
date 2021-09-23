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

%% Create Task Environment
sampling_intensity          = 2;
adaptive_sampling_intensity = 10;
r_search                    = 0.3;
max_trials                  = 100;

for obs_config_idx = 1:6
    for tdx = 1:8
        file_name                               = "FMT_Tests_Obstacle_Config_"+string(obs_config_idx)+"_T"+string(tdx);
        [Obstacles_Poly, Obstacle_break_Poly]   = CLS_Obstacles.Obstacle_Config_select(obs_config_idx, IsDEBUG);
        Env                                     = CLS_ENV_SE2(PrintingTask, T, s, robot, IRM, Obstacles_Poly, Obstacle_break_Poly, IsDEBUG);
        FMTStar                                 = CLS_2DFMTStar(Env, sampling_intensity, adaptive_sampling_intensity, r_search, max_trials, file_name);
        [path, ite, cost, time, record]         = FMTStar.FMT_Star;
    end
end