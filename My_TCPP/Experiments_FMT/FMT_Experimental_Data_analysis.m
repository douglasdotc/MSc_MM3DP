filePattern = fullfile('*.mat');
matFiles = dir(filePattern);

Data                = [];
V_open_size         = {};
V_closed_size       = {};
V_unvisited_size    = {};
radius_rec          = {};
s_max_rec           = {};

for k = 1:length(matFiles)
    baseFileName = matFiles(k).name;
    fullFileName = fullfile(baseFileName);
    fprintf(1, 'Now reading %s\n', fullFileName);
    thisData = load(fullFileName);

    config = str2double(fullFileName(27));
    trial  = str2double(fullFileName(30));
    
    Data = [Data; config, trial, thisData.time, thisData.sampling_time, thisData.ite, thisData.total_cost, length(thisData.paths), thisData.sum_path_nodes,...
            mean(thisData.record(:,4)), std(thisData.record(:,4))];
    
    V_open_size{end+1}         = [config, trial, thisData.record(:,1)'];
    V_closed_size{end+1}       = [config, trial, thisData.record(:,2)'];
    V_unvisited_size{end+1}    = [config, trial, thisData.record(:,3)'];
    radius_rec{end+1}          = [config, trial, thisData.record(:,4)'];
    s_max_rec{end+1}           = [config, trial, thisData.record(:,6)'];
end
Config_1_results = Data(Data(:,1) == 1,:);
Config_2_results = Data(Data(:,1) == 2,:);
Config_3_results = Data(Data(:,1) == 3,:);
Config_4_results = Data(Data(:,1) == 4,:);
Config_5_results = Data(Data(:,1) == 5,:);
Config_6_results = Data(Data(:,1) == 6,:);

ax1 = figure(1);
hold on
box on
xlabel('Number of iterations')
ylabel('s_{max}')
config = 1;
for idx = 1:length(s_max_rec)
    if config == s_max_rec{idx}(1)
        plot(s_max_rec{idx}(3:end))
    else
        legend({'Test 1', 'Test 2', 'Test 3', 'Test 4', 'Test 5'})
        drawnow
        saveas(ax1, "FMT_Config_"+string(config)+"_s_max_change.fig")
        hold off
        delete(ax1)
        config = s_max_rec{idx}(1);
        ax1 = figure(1);
        hold on
        box on
        xlabel('Number of iterations')
        ylabel('s_{max}')
        plot(s_max_rec{idx}(3:end))
    end
end
legend({'Test 1', 'Test 2', 'Test 3', 'Test 4', 'Test 5'})
drawnow
saveas(ax1, "FMT_Config_"+string(config)+"_s_max_change.fig")
hold off
delete(ax1)


ax1 = figure(1);
hold on
box on
xlabel('Number of iterations')
ylabel('Size of V_{open}')
config = 1;
for idx = 1:length(V_open_size)
    if config == V_open_size{idx}(1)
        plot(V_open_size{idx}(3:end))
    else
        legend({'Test 1', 'Test 2', 'Test 3', 'Test 4', 'Test 5'})
        drawnow
        saveas(ax1, "FMT_Config_"+string(config)+"_V_open_size_change.fig")
        hold off
        delete(ax1)
        config = V_open_size{idx}(1);
        ax1 = figure(1);
        hold on
        box on
        xlabel('Number of iterations')
        ylabel('Size of V_{open}')
        plot(V_open_size{idx}(3:end))
    end
end
legend({'Test 1', 'Test 2', 'Test 3', 'Test 4', 'Test 5'})
drawnow
saveas(ax1, "FMT_Config_"+string(config)+"_V_open_size_change.fig")
hold off
delete(ax1)


ax1 = figure(1);
hold on
box on
xlabel('Number of iterations')
ylabel('Size of V_{closed}')
config = 1;
for idx = 1:length(V_closed_size)
    if config == V_closed_size{idx}(1)
        plot(V_closed_size{idx}(3:end))
    else
        legend({'Test 1', 'Test 2', 'Test 3', 'Test 4', 'Test 5'})
        drawnow
        saveas(ax1, "FMT_Config_"+string(config)+"_V_closed_size_change.fig")
        hold off
        delete(ax1)
        config = V_closed_size{idx}(1);
        ax1 = figure(1);
        hold on
        box on
        xlabel('Number of iterations')
        ylabel('Size of V_{closed}')
        plot(V_closed_size{idx}(3:end))
    end
end
legend({'Test 1', 'Test 2', 'Test 3', 'Test 4', 'Test 5'})
drawnow
saveas(ax1, "FMT_Config_"+string(config)+"_V_closed_size_change.fig")
hold off
delete(ax1)


ax1 = figure(1);
hold on
box on
xlabel('Number of iterations')
ylabel('Size of V_{unvisited}')
config = 1;
for idx = 1:length(V_unvisited_size)
    if config == V_unvisited_size{idx}(1)
        plot(V_unvisited_size{idx}(3:end))
    else
        legend({'Test 1', 'Test 2', 'Test 3', 'Test 4', 'Test 5'})
        drawnow
        saveas(ax1, "FMT_Config_"+string(config)+"_V_unvisited_size_change.fig")
        hold off
        delete(ax1)
        config = V_unvisited_size{idx}(1);
        ax1 = figure(1);
        hold on
        box on
        xlabel('Number of iterations')
        ylabel('Size of V_{unvisited}')
        plot(V_unvisited_size{idx}(3:end))
    end
end
legend({'Test 1', 'Test 2', 'Test 3', 'Test 4', 'Test 5'})
drawnow
saveas(ax1, "FMT_Config_"+string(config)+"_V_unvisited_size_change.fig")
hold off
delete(ax1)


ax1 = figure(1);
hold on
box on
xlabel('Number of iterations')
ylabel('Size of changing radius (m)')
config = 1;
for idx = 1:length(radius_rec)
    if config == radius_rec{idx}(1)
        plot(radius_rec{idx}(3:end))
    else
        legend({'Test 1', 'Test 2', 'Test 3', 'Test 4', 'Test 5'})
        drawnow
        saveas(ax1, "FMT_Config_"+string(config)+"_size_of_changing_radius.fig")
        hold off
        delete(ax1)
        config = radius_rec{idx}(1);
        ax1 = figure(1);
        hold on
        box on
        xlabel('Number of iterations')
        ylabel('Size of changing radius (m)')
        plot(radius_rec{idx}(3:end))
    end
end
legend({'Test 1', 'Test 2', 'Test 3', 'Test 4', 'Test 5'})
drawnow
saveas(ax1, "FMT_Config_"+string(config)+"_size_of_changing_radius.fig")
hold off
delete(ax1)