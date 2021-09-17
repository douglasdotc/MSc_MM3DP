filePattern = fullfile('*.mat');
matFiles = dir(filePattern);

Data = [];

for k = 1:length(matFiles)
    baseFileName = matFiles(k).name;
    fullFileName = fullfile(baseFileName);
    fprintf(1, 'Now reading %s\n', fullFileName);
    thisData = load(fullFileName);

    config = str2double(fullFileName(27));
    trial  = str2double(fullFileName(30));
    
    Data = [Data; config, trial, thisData.time, thisData.sampling_time, thisData.ite, thisData.total_cost, thisData.sum_path_nodes,...
            mean(thisData.record(:,4)), std(thisData.record(:,4))];
    
    ax1 = figure(1);
    plot(thisData.record(:,1))
    box on
    hold on
    xlabel('Number of iterations')
    ylabel('Size of V_{open}')
    drawnow
    saveas(ax1, "FMT_Config_"+string(config)+"_T"+string(trial)+"V_open_size_change.png")
    hold off
    delete(ax1)
    
    ax1 = figure(1);
    plot(thisData.record(:,2))
    box on
    hold on
    plot(thisData.record(:,3))
    xlabel('Number of iterations')
    legend({'Size of V_{closed}','Size of V_{unvisited}'})
    drawnow
    saveas(ax1, "FMT_Config_"+string(config)+"_T"+string(trial)+"V_closed_V_unvisited_size_change.png")
    hold off
    delete(ax1)
        
    ax1 = figure(1);
    plot(thisData.record(:,4))
    box on
    hold on
    xlabel('Number of iterations')
    ylabel('Radius (m)')
    drawnow
    saveas(ax1, "FMT_Config_"+string(config)+"_T"+string(trial)+".png")
    hold off
    delete(ax1)
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