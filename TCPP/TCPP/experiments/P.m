close all
clear all
%%
load("IRM_complete_final2")
%% Youwasp collision box
ywx = 0.57; ywy = 0.36;
youwasp = [ywx / 2, ywy / 2, 1; ywx / 2, - ywy / 2, 1; -ywx / 2, - ywy / 2, 1; -ywx / 2, ywy / 2, 1]';
%%  create a task

p = Tasks.PPath();
p.resample(0.01);
p.smooth(0.15);
p.scale([3,0.7,1]);

l2=flip(p.path,1);
l2(:,3)=0.03;
p.path=[p.path; l2];


p.resample(0.001);
q = Tasks.UPath(.1);
q.scale([0.25 0.075 1]);
p.superimpose(q) 
task=p;
% l2=flip(task.path,1);
% l2(:,3)=0.03;
% task.path=[task.path; l2];
% task.resample(0.01);
% task.path=task.path(1:end-1,:);
% task.resample(0.01);
% task.path(ceil(size(task.path(:,3),1)/2):end,3)=0.03;
% task.resample(0.01);

t = task.gett(0.01);
T = task.toTForm(task);
task.plot()
T(3, 4, :) = 0;
%% cREATE obstacles
den=0.05;
mycube=@(x,y,dx,dy) [ [x,y]; [repmat(x,ceil(dy/den),1), linspace(y,y+dy,ceil(dy/den))'] ; [linspace(x,x+dx,ceil(dx/den))', repmat(y+dy,ceil(dx/den),1)];    [repmat(x+dx,ceil(dy/den),1), linspace(y+dy,y,ceil(dy/den))']   ];


mycube = @(x, y, dx, dy) [[(x - dx / 2), (y - dy / 2), 1]; [(x - dx / 2), (y +dy / 2), 1]; [(x +dx / 2), (y +dy / 2), 1]; [(x +dx / 2),( y -dy / 2), 1]];

obstacles={};
obstacles{1}=mycube(1.5,-0.7,1.5,.45);
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
prc=30;
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
C = CSE2TTask2(x_lim, y_lim, T, task,youwasp, IRM);
C.convex_obstacles=obstacles;
%%
qsn = 50;
qgn = 50;
qs = C.sample_qs_around_task(0, qsn);
qg=[];
%%
rrt=RRTStarOrderedTask(C,qs,qg);
rrt.e_inc = 0.01; %default 0.01;
rrt.e_reach = 0.05; %not so much
% rrt.e_reach = 0.025; %smooth
rrt.bias = 0.01; %default 0.01;
rrt.t_var = 0.05; %default 0.01;
rrt.t_timeout = 500; %default 0.01;
rrt.q_neigh_rad=rrt.e_reach*5;
path = rrt.solve(1e4)
% % path = rrt.solve(1e4, 'draw')
% %%
% path_smooth = rrt.smooth(path, 100);
% path_simple = rrt.simplify(path, 1000);
% 
% rrt.rewire_path(path);
% path_simple=rrt.trace_path(path(end))
% %%#
%%
close
figure
% rrt.draw_tree();
C.draw_environment()
C.draw_task()
rrt.draw_path(path);

% ids=ceil(linspace(1,length(path),5));
ids=[5   73   145   175 217   278];
rrt.draw_robot(T,path(ids))
axis equal
hold off

% xlim([-1 4])
set(gcf,'Color',[1 1 1])
xlabel('x','fontweight','bold','fontsize',16);
ylabel('y','fontweight','bold','fontsize',16);
set(gca,'fontweight','bold','fontsize',12);
% 
% %save("PBest","path")
%%
% tic
%  path={};
% for ii=1:10
%     
% path{ii} = rrt.solve(1e4)
% end
% toc
% save("P10","path")