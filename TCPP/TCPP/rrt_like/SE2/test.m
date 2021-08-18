%%%%%%%%%%%%%%% XY TESTS %%%%%%%%%%%%%%%%%%%%%%
%% XY test simple
C=Cxy([0 3],[0 3]);
qs=Q([1,1]);
qg=Q([2,2]);
rrt=RRT(C,qs,qg);
path=rrt.solve(1e4,'draw')
%% XY test: non default settings
rrt.e_inc=0.005;%default 0.01;
rrt.e_reach=0.01;%default 0.01;
path=rrt.solve(1e4,'draw')

%% XY test:qs and Qg are sets
C=Cxy([0 3],[0 3]);
qs_list=[Q([1,1.1]);Q([1.1,1]);Q([.9,1]);Q([.8,1])]
qg_list=[Q([2,2]);Q([2,2.1]);Q([1.5,2.9]);Q([2.1,2.1])]
rrt=RRT(C,qs_list,qg_list);
path=rrt.solve(1e4,'draw')

%% XY test:  se_qg
C=Cxy([0 3],[0 3]);
qs_list=[Q([1,1.1]);Q([1.1,1]);Q([.9,1]);Q([.8,1]);]
qg=Q([2,2]);
is_qg=@(q) C.dist(q,qg)<0.2;
rrt=RRT(C,qs_list,[]);
rrt.is_qg=is_qg;
path=rrt.solve(1e4,'draw')

%% SE2 TESTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SE2 test simple  wrap case
C=CSE2([0 3],[0 3]);
qs=QSE2([1,1,-3]);
qg=QSE2([2,2,3]);
rrt=RRT(C,qs,qg);
rrt.e_inc=0.025;%default 0.01;
    rrt.e_reach=0.1;%default 0.01;
rrt.bias=0.05;%default 0.01;
path=rrt.solve(1e4,'draw')
%% SE2 test simple no wrap case
C=CSE2([0 3],[0 3]);
qs=QSE2([1,1,-1.5]);
qg=QSE2([2,2,1.5]);
rrt=RRT(C,qs,qg);
rrt.e_inc=0.025;%default 0.01;
rrt.e_reach=0.05;%default 0.01;
rrt.bias=0.05;%default 0.01;
path=rrt.solve(1e4,'draw')

%% SE2 small increments
C=CSE2([0 3],[0 3]);
qs=QSE2([1,1,1]);
qg=QSE2([2,2,2]);
rrt=RRT(C,qs,qg);
rrt.e_inc=0.005; %default 0.01;
rrt.e_reach=0.01; %default 0.01;
rrt.bias=0.05; %default 0.01;
path=rrt.solve(1e4,'draw')
%% SE2 big increments
C=CSE2([0 3],[0 3]);
qs=QSE2([1,1,1]);
qg=QSE2([2,2,2]);
rrt=RRT(C,qs,qg);
rrt.e_inc=0.05; %default 0.01;
rrt.e_reach=0.5; %default 0.01;
path=rrt.solve(1e4,'draw')
%% SE2T TESTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SE2T test simple
C=CSE2T([0 10],[0 10]);
qs=QSE2T([1,1,1,0]);
qg=QSE2T([9,9,9,1]);
is_qg=@(q) C.dist(qg,q)<0.2;
rrt=RRT(C,qs,qg);
rrt.is_qg=is_qg;
rrt.e_inc=0.05; %default 0.01;
rrt.e_reach=0.3; %default 0.01;
rrt.bias=0.05; %default 0.01;
path=rrt.solve(1e4,'draw')
%% SE2TTask TESTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SE2T test Collisions
ywx = 0.57; ywy = 0.36;
youwasp = [ywx / 2, ywy / 2, 1; ywx / 2, - ywy / 2, 1; -ywx / 2, - ywy / 2, 1; -ywx / 2, ywy / 2, 1]';
[T,t]=ntu_curve(0.01,0.01);
t=t./max(t);
task=tform2vec(T);
% yaw=
task=[task(:,1:2)]
C=CSE2TTask([0 10],[0 10], task, youwasp);
% C.convex_obstacles=[1.4 1.8; 1.8 1.8; 1.8 1.4; 1.4 1.4];
qs=QSE2T([task(1,1:3),0]);
qg=QSE2T([3 0.3 0 1]);% this is just for the bias
is_qg=@(q) C.dist(qg,q)<0.3;
rrt=RRT(C,qs,qg);
rrt.is_qg=is_qg;
rrt.e_inc=0.05; %default 0.01;
rrt.e_reach=0.3; %default 0.01;
rrt.bias=0.01; %default 0.01;
path=rrt.solve(1e4)
%%%%%%%%%%%%%%% SE2T + IK + Collisions TESTS %%%%%%%%%%%%%%%%%%%%%%
