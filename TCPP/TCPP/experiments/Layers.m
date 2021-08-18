close all
clear all
%%
load("IRM_complete_final2")
%% Youwasp collision box
ywx = 0.57; ywy = 0.36;
youwasp = [ywx / 2, ywy / 2, 1; ywx / 2, - ywy / 2, 1; -ywx / 2, - ywy / 2, 1; -ywx / 2, ywy / 2, 1]';
%%  create a task

p = Tasks.ArcPath(4*pi);
% p.smooth(0.075);
p.scale([1, .5, 1]);
p.resample(0.001);
q = Tasks.UPath(.1);
q.scale([0.2 0.075 1]);
p.superimpose(q) 
task=p;
task.path=task.path(1:end-1,:);
task.resample(0.01);
task.path(ceil(size(task.path(:,3),1)/2):end,3)=0.03;
task.resample(0.01);

t = task.gett(0.01);
T = task.toTForm(task);
task.plot()
T(3, 4, :) = 0;
%% cREATE obstacles
den=0.1;
mycube=@(x,y,dx,dy) [ [x,y]; [repmat(x,ceil(dy/den),1), linspace(y,y+dy,ceil(dy/den))'] ; [linspace(x,x+dx,ceil(dx/den))', repmat(y+dy,ceil(dx/den),1)];    [repmat(x+dx,ceil(dy/den),1), linspace(y+dy,y,ceil(dy/den))']   ];

obstacles={};
% obstacles{1}=mycube(-.5,-.5,2-(.25/2),.25);
% obstacles{2}=mycube(1.5-(.25/2),-.5,0.25,2);
% 
% obstacles{3}=mycube(1.5-(.25/2),2.5,0.25,.75);
% 
% obstacles{4}=mycube(1.5-(.25/2),3.25,3.25-1.5+(.25/2),.25);
% obstacles{5}=mycube(3.25-.2,3.25-3.75,.2,3.75);


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
C = CSE2TTask2(x_lim, y_lim, T,task, youwasp, IRM);
C.convex_obstacles=obstacles;
%%
qsn = 50;
qs = C.sample_qs_around_task(0, qsn);
%%
rrt=RRTStarOrderedTask(C,qs,qg);
rrt.e_inc = 0.01; %default 0.01;
rrt.e_reach = 0.025; %default 0.01;
rrt.bias = 0.01; %default 0.01;
rrt.t_var = 0.05; %default 0.01;
rrt.t_timeout = 1000; %default 0.01;
rrt.q_neigh_rad=rrt.e_reach*3;
path = rrt.solve(1e4)
% path = rrt.solve(1e4, 'draw')
%%
% path_smooth = rrt.smooth(path, 100);
% path_simple = rrt.simplify(path, 1000);

rrt.rewire_path(path);
path_simple=rrt.trace_path(path(end))
%%
close
figure
C.draw_environment()
C.draw_task()
rrt.draw_tree();
rrt.draw_path(path);
% rrt.draw_path(path_simple);
axis equal
hold off
