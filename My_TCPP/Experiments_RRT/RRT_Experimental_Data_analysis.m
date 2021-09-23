filePattern = fullfile('*.mat');
matFiles = dir(filePattern);

Data                = [];
s_max_rec           = {};
stall_count         = {};

for k = 1:length(matFiles)
    baseFileName = matFiles(k).name;
    fullFileName = fullfile(baseFileName);
    fprintf(1, 'Now reading %s\n', fullFileName);
    thisData = load(fullFileName);

    config = str2double(fullFileName(27));
    trial  = str2double(fullFileName(30));
    
    Data = [Data; config, trial, thisData.time, thisData.ite, thisData.total_cost, length(thisData.paths), thisData.sum_path_nodes];
    
    s_max_rec{end+1}    = [config, trial, thisData.record(:,2)'];
    stall_count{end+1}  = [config, trial, thisData.record(:,3)'];
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
        saveas(ax1, "RRT_Config_"+string(config)+"_s_max_change.fig")
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
saveas(ax1, "RRT_Config_"+string(config)+"_s_max_change.fig")
hold off
delete(ax1)