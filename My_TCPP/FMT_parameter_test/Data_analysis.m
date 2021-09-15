filePattern = fullfile('*.mat');
matFiles = dir(filePattern);

sampling_intensity_rec = [];
radius_rec = [];
time_rec = [];
num_of_path_rec = [];
path_cost_rec = [];
radius_max_rec = [];
radius_mode_rec = [];
radius_mean_rec = [];
radius_std_rec = [];

for k = 1:length(matFiles)
  baseFileName = matFiles(k).name;
  fullFileName = fullfile(baseFileName);
  fprintf(1, 'Now reading %s\n', fullFileName);
  thisData = load(fullFileName);
  
  sampling_intensity = str2double(fullFileName(16));
  radius             = str2double(fullFileName(19:end-4));
  
  sampling_intensity_rec = [sampling_intensity_rec, sampling_intensity];
  radius_rec             = [radius_rec, radius];
  time_rec               = [time_rec, thisData.time];
  num_of_path_rec        = [num_of_path_rec, length(thisData.paths)];
  path_cost_rec          = [path_cost_rec, thisData.total_cost];
  radius_max_rec         = [radius_max_rec, max(thisData.record(:,4))];
  radius_mode_rec        = [radius_mode_rec, mode(thisData.record(:,4))];
  radius_mean_rec        = [radius_mean_rec, mean(thisData.record(:,4))];
  radius_std_rec         = [radius_std_rec, std(thisData.record(:,4))];
      
  if length(thisData.paths) == 1
      ax1 = figure(1);
      plot(thisData.record(:,4))
      box on
      hold on
      xlabel('Number of iterations')
      ylabel('Radius (m)')
      drawnow
      saveas(ax1, "FMT_ite_vs_radius_"+string(sampling_intensity)+"_r"+string(radius)+".png")
      hold off
      delete(ax1)
  end
end



ax1 = figure(1);
f = fit([sampling_intensity_rec',radius_rec'],time_rec', 'cubicinterp');
zPrediction = f(sampling_intensity_rec(1), radius_rec(1));
plot(f)
hold on
box on
xlabel('Sampling intensity (unit/progress)')
ylabel('Specified radius (m)')
zlabel('Time (s)')
h = colorbar;
ylabel(h, 'Time (s)')
saveas(ax1, "FMT_sample_int_vs_radius_vs_time.fig")
hold off
delete(ax1)

ax1 = figure(1);
f = fit([sampling_intensity_rec',radius_rec'],num_of_path_rec', 'cubicinterp');
zPrediction = f(sampling_intensity_rec(1), radius_rec(1));
plot(f)
hold on
box on
xlabel('Sampling intensity (unit/progress)')
ylabel('Specified radius (m)')
zlabel('Number of paths')
h = colorbar;
ylabel(h, 'Number of paths')
saveas(ax1, "FMT_sample_int_vs_radius_vs_num_paths.fig")
hold off
delete(ax1)

ax1 = figure(1);
f = fit([sampling_intensity_rec',radius_rec'],path_cost_rec', 'cubicinterp');
zPrediction = f(sampling_intensity_rec(1), radius_rec(1));
plot(f)
hold on
box on
xlabel('Sampling intensity (unit/progress)')
ylabel('Specified radius (m)')
zlabel('Path cost')
h = colorbar;
ylabel(h, 'Path cost')
saveas(ax1, "FMT_sample_int_vs_radius_vs_path_cost.fig")
hold off
delete(ax1)

ax1 = figure(1);
f = fit([sampling_intensity_rec',radius_rec'],radius_max_rec', 'cubicinterp');
zPrediction = f(sampling_intensity_rec(1), radius_rec(1));
plot(f)
hold on
box on
xlabel('Sampling intensity (unit/progress)')
ylabel('Specified radius (m)')
h = colorbar;
ylabel(h, 'Max. radius (m)')
saveas(ax1, "FMT_sample_int_vs_radius_vs_maxr.fig")
hold off
delete(ax1)
% sampling_intensity_rec + radius_rec + time_rec
% sampling_intensity_rec + radius_rec + num_of_path_rec

ax1 = figure(1);
f = fit([sampling_intensity_rec',radius_rec'],radius_mode_rec', 'cubicinterp');
zPrediction = f(sampling_intensity_rec(1), radius_rec(1));
plot(f)
hold on
box on
xlabel('Sampling intensity (unit/progress)')
ylabel('Specified radius (m)')
h = colorbar;
ylabel(h, 'Mode of radius (m)')
saveas(ax1, "FMT_sample_int_vs_radius_vs_moder.fig")
hold off
delete(ax1)

ax1 = figure(1);
f = fit([sampling_intensity_rec',radius_rec'],radius_mean_rec', 'cubicinterp');
zPrediction = f(sampling_intensity_rec(1), radius_rec(1));
plot(f)
hold on
box on
xlabel('Sampling intensity (unit/progress)')
ylabel('Specified radius (m)')
h = colorbar;
ylabel(h, 'Mean of radius (m)')
saveas(ax1, "FMT_sample_int_vs_radius_vs_meanr.fig")
hold off
delete(ax1)

ax1 = figure(1);
f = fit([sampling_intensity_rec',radius_rec'],radius_std_rec', 'cubicinterp');
zPrediction = f(sampling_intensity_rec(1), radius_rec(1));
plot(f)
hold on
box on
xlabel('Sampling intensity (unit/progress)')
ylabel('Specified radius (m)')
zlabel('Standard deviation of radius (m)')
h = colorbar;
ylabel(h, 'Standard deviation of radius (m)')
saveas(ax1, "FMT_sample_int_vs_radius_vs_stdr.fig")
hold off
delete(ax1)