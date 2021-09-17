cd 'E:\Google Drive HNC\_My Study\_UCL_Master_Robotics and Computation\Courses\_Dissertation\Code\MSc_MM3DP\My_TCPP'
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
%% IRM
min_task_robot_dist = 0.15;
IRM                 = CLS_FakeIRM(min_task_robot_dist, IsDEBUG);

%% Obstacles
% Obstacles_Poly = CLS_Obstacles.Obstacle_Config_select(3, IsDEBUG);
% Obstacles_Poly = {};
% drawnow
%% Create Task Environment
sampling_intensity  = 2;
r_search            = 0.3;
parfor obs_config_idx = 1:5
    for tdx = 1:5
        file_name = "FMT_Tests_Obstacle_Config_"+string(obs_config_idx)+"_T"+string(tdx);
        ax1 = figure(1); %
        PrintingTask.plot(); %
        box on %
        hold on %
        xlabel('x (m)') %
        ylabel('y (m)') %
        Obstacles_Poly = CLS_Obstacles.Obstacle_Config_select(obs_config_idx, IsDEBUG);
        drawnow %
        
        Env                             = CLS_ENV_SE2(PrintingTask, T, s, robot, IRM, Obstacles_Poly, IsDEBUG);
        FMTStar                         = CLS_2DFMTStar(Env, sampling_intensity, r_search, file_name);
        [path, ite, cost, time, record] = FMTStar.FMT_Star;
        saveas(ax1, file_name+".fig")
        hold off %
        delete(ax1); %
        
        % Plot path only
        ax1 = figure(1); %
        PrintingTask.plot(); %
        box on %
        hold on %
        xlabel('x (m)') %
        ylabel('y (m)') %
        Obstacles_Poly = CLS_Obstacles.Obstacle_Config_select(obs_config_idx, IsDEBUG);
        
        
        for pdx = 1:length(path)
            for kdx = length(path{pdx}):-1:2
                line([path{pdx}(kdx-1).pose(1), path{pdx}(kdx).pose(1)], [path{pdx}(kdx-1).pose(2), path{pdx}(kdx).pose(2)], [0,0], 'Color', '#316879', 'LineWidth', 4);
            end
            POSES = Env.Extract_item(path{pdx}, 'pose');
            quiver(POSES(:,1), POSES(:,2), POSES(:,3), POSES(:,4), 0.5, 'Color', '#316879', 'LineWidth', 2);
        end
        drawnow %
        saveas(ax1, file_name+"_Path_only.fig")
        hold off %
        delete(ax1); %
    end
end
% scatter(ite_arr, time_arr)
% xlabel('Number of iterations')
% ylabel('Runtime')