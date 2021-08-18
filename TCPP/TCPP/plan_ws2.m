close all
% clear all
%%
% load("IRM_complete_final2")
%% Youwasp collision box
ywx = 0.57; ywy = 0.36;
youwasp = [ywx / 2, ywy / 2, 1; ywx / 2, - ywy / 2, 1; -ywx / 2, - ywy / 2, 1; -ywx / 2, ywy / 2, 1]';
%%  create a task
% task = Tasks.ArcPath(pi)
task = Tasks.HPath(2, 100)
task.smooth(0.075);
task.scale([3,1,1]);
task.resample(0.01);
t = task.gett(0.01);
T = task.toTForm(task);
task.plot()
T(3, 4, :) = 0;
% T(3, 4, :) = linspace(0,0.3,size(T,3));


%% Set XY bounds
x_lim = [min(T(1, 4, :)) - 2, max(T(1, 4, :)) + 2];
y_lim = [min(T(2, 4, :)) - 2, max(T(2, 4, :)) + 2];
%% Select which IRI you like
% IRM.voxRI=IRM.voxRIz;
% IRM.voxRI=IRM.voxRIzpr;
IRM.voxRI = IRM.voxRIzprnn;
%% Compute layers

IRM.layers=cell(length(IRM.spans{3}),1);

for ii =1:length(IRM.spans{3})   
    layer.inds=find( abs(IRM.voxCenters(:,3) -  IRM.spans{3}(ii) )<0.01);
    layer.IRI=IRM.voxRI(layer.inds);    
    IRM.layers{ii}=layer;
    
end
% cellfun(@(c) sum(c.IRI)  ,IRM.layers)


%% prune IRM
prc=0;
cutoff = prctile(IRM.voxRI(IRM.voxRI>0), prc);
for ii =1:length(IRM.spans{3})   
    cutoff = prctile(IRM.voxRI(IRM.voxRI>0), prc);
    inds=IRM.layers{ii}.IRI>cutoff;
    IRM.layers{ii}.IRI=IRM.layers{ii}.IRI(inds);
    IRM.layers{ii}.inds=IRM.layers{ii}.inds(inds);    
    IRM.layers{ii}.cutoff=cutoff;
end

%% Compute cdf (this shoudl not be here but ok)
for ii=1:length(IRM.spans{3})
    IRM.layers{ii}.cdf= cumsum(IRM.layers{ii}.IRI);    
end

%%
C = CSE2TTask2(x_lim, y_lim, T, task, youwasp, IRM);
%%
qsn = 50;
qgn = 50;
qs = C.sample_qs_around_task(0, qsn);
qg = []

%% Obstacles
% C.convex_obstacles={[0.98 1.3; 1.02 1.3; 1.02 1.6;  1.02 2; .98 2; .98 1.6 ]+[0 0.1],[0.98 1.3; 1.02 1.3; 1.02 1.6;  1.02 2; .98 2; .98 1.6 ]+[0 -.75] };
%% is goal func
% is_qg=@(q) C.dist(qg,q)<0.3;
%%
close all
% rrt = RRTTask(C, qs, qg);
% rrt = RRTTaskOrdered(C, qs, qg);
rrt=RRTStarOrderedTask(C,qs,qg);
% rrt.is_qg=is_qg;
rrt.e_inc = 0.01; %default 0.01;
rrt.e_reach = 0.1; %default 0.01;
rrt.bias = 0.01; %default 0.01;
rrt.t_var = 0.05; %default 0.01;
rrt.t_timeout = 1000; %default 0.01;
rrt.q_neigh_rad=rrt.e_reach*3;

path = rrt.solve(1e4)
% path = rrt.solve(1e4, 'draw')
% path_simple = rrt.simplify(path, 1000);
% rrt.draw_path((path_simple));
rrt.draw_path((path));

% %% comments
% % Jump example:
% % reach = 0.1; noise .05 maybe.  t_timeout500
% %
% % rrt=RRTTask(C,qs,qg);
% rrt.e_inc=0.03; %default 0.01;
% rrt.e_reach=0.1; %default 0.01;
% rrt.bias=0.01; %default 0.01;
% rrt.t_timeout=500; %default 0.01;
% rrt.t_var=0.05; %default 0.01;
% path=rrt.solve(1e4,'draw')
% path_simple=rrt.simplify(path,1000);
% rrt.draw_path((path_simple));
% %%
%
% title("Path Interuption over narrow gap",'FontSize', 14);
% axis equal
% xlabel("X",'FontSize', 14)
% ylabel("Y",'FontSize', 14)
% % xlim([-0.1 0.5])
% % ylim([-0.35 0.35])
% ax=gca;
% ax.FontSize = 16;
% % c = colorbar;
% % c.Label.String="No. Valid IK solutions";
% view([0 90])
%
% %%
% title("Smoothed path with interruption",'FontSize', 14);
% axis equal
% xlabel("X",'FontSize', 14)
% ylabel("Y",'FontSize', 14)
% % xlim([-0.1 0.5])
% % ylim([-0.35 0.35])
% ax=gca;
% ax.FontSize = 16;
% % c = colorbar;
% % c.Label.String="No. Valid IK solutions";
% view([0 90])
