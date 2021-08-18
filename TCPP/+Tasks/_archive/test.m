import Tasks.*

import Curves.*
%% %%% %%% %%% %%%Pattern Tests:
%% %%% CurlyPattern
pattern = CurlyPattern(3)
pattern.is_valid()
pattern.plot()
%%
pattern.scale([2, 10, 0.1])
pattern.is_valid()
pattern.plot()
%%
pattern.set_density(0.01);
pattern.is_valid()
pattern.plot()
%% %%% UPattern
upattern = UPattern(0.1)
upattern.is_valid()
upattern.plot()
%%
upattern.scale([.4, .15, 1])
upattern.plot()
%%
upattern.set_density(0.01);
upattern.is_valid()
upattern.plot()

%% %%% %%% %%% %%%Path Tests:
%% %%% HPath
path = HPath(3, 150);
path.scale([2, 3, 4])
path.plot()
%%
path.set_density(0.01);
path.plot()
%% %%% ArcPath
path = ArcPath(pi);
path.plot()
%%
path.set_density(0.01);
path.plot()
%% %%% CurlyPath
path = CurlyPath(3);
path.plot()
%%
path.set_density(0.01);
path.plot()

%% %%% UPath
path = UPath(.1);
path.plot()
%%
path.set_density(0.01);
path.plot()
%% %%% %%% %%% %%%Trajectory Tests:
%% %%% HPath
path = UPath(.1);
path.scale([10, 1, 1]);
traj = path.to_traj(0.01, 1);
traj.plot(20)
%% %%% ArcPath
path = ArcPath(pi);
path.scale([10, 1, 1]);
traj = path.to_traj(0.01, 1);
traj.plot(20)

%% %%% UPath
path = UPath(.1);
path.scale([10, 1, 1]);
traj = path.to_traj(0.01, 1);
traj.plot(20)

%% %%% %%% %%% %%%SuperImpose Tests:
%% %%% UPattern on  HPath
pattern = UPattern(.1);
pattern.scale([.4, .125, 1]);
pattern.plot()
%%
path = HPath(2, 100);
path.scale([3, 2, 1]);
path.plot()
traj = path.to_traj(0.01, 1);
traj.plot(20)
%%
traj.superimpose(pattern)
