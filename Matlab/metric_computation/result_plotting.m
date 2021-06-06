clear all
close all
clc
%% get obstalce data
% pathname= {'flocking_data/manually/','flocking_data/bra_manually/','flocking_data/allmetric/',...
%     'flocking_data/allmetric2/','flocking_data/velmetric/'};
pathname= {'/manually/','/bra_manually/',...
    '/allmetric2/','/velmetric/','/cross/'};
filename = {'flocking_metrics0.csv','flocking_metrics1.csv','flocking_metrics2.csv',...
    'flocking_metrics3.csv','flocking_metrics4.csv'};
metrics_mean = zeros(length(pathname),1);
metrics_std = zeros(length(pathname),1);

for j = 1:length(pathname)
    metrics_avg = zeros(5,1);
    %opening and loading logs file
    for i = 1:5
        fid= fopen(['obstacle_data',pathname{j},filename{i}]);
        data= textscan(fid,"%f %f %f %f %f","Delimiter",";","headerlines",1);

        %get values
        time= data{:,1};
        metrics = data{:,5};
        time = time(2:939);
        metrics = metrics(2:939);

        metrics_avg(i) = mean(metrics);
    end

    metrics_mean(j) = mean(metrics_avg);
    metrics_std(j) = std(metrics_avg);
end

%% get crossing data
% pathname= {'flocking_data/manually/','flocking_data/bra_manually/','flocking_data/allmetric/',...
%     'flocking_data/allmetric2/','flocking_data/velmetric/'};
pathname2= {'/manually/','/bra_manually/',...
    '/allmetric2/','/velmetric/','/cross/'};
filename2 = {'flocking_metrics0_0.csv','flocking_metrics1_0.csv','flocking_metrics2_0.csv',...
    'flocking_metrics3_0.csv','flocking_metrics4_0.csv'};
filename3 = {'flocking_metrics0_1.csv','flocking_metrics1_1.csv','flocking_metrics2_1.csv',...
    'flocking_metrics3_1.csv','flocking_metrics4_1.csv'};
metrics_mean2 = zeros(length(pathname2),1);
metrics_std2 = zeros(length(pathname2),1);

for j = 1:length(pathname2)
    metrics_avg2 = zeros(10,1);
    %opening and loading logs file
    for i = 1:5
        fid= fopen(['crossing_data',pathname2{j},filename2{i}]);
        data= textscan(fid,"%f %f %f %f %f","Delimiter",";","headerlines",1);

        %get values
        time= data{:,1};
        metrics = data{:,5};
        time = time(2:392);
        metrics = metrics(2:392);

        metrics_avg2(i) = mean(metrics);
    end
    
    for i = 1:5
        fid= fopen(['crossing_data',pathname2{j},filename3{i}]);
        data= textscan(fid,"%f %f %f %f %f","Delimiter",";","headerlines",1);

        %get values
        time= data{:,1};
        metrics = data{:,5};
        time = time(2:392);
        metrics = metrics(2:392);

        metrics_avg2(i+5) = mean(metrics);
    end

    metrics_mean2(j) = mean(metrics_avg2);
    metrics_std2(j) = std(metrics_avg2);
end
%% plot
figure('NumberTitle', 'off', 'Name', 'Results statistics',...
    'units','normalized','outerposition',[0 0 1 1]);
legends = ["Handcrafted","Half-PSO","PSO 1","PSO 2","PSO 3"];
y = [metrics_mean metrics_mean2]';
neg = [metrics_std metrics_std2]';
barweb(y,neg,1,{'Obstalce','Crossing'},[])
h=legend(legends, "location", 'northwest');
set(h,'Fontsize',30);
set(gca,'FontSize',20);
ylabel('\textbf{Overall metric}','Interpreter','latex')
xlabel('\textbf{Scenario}','Interpreter','latex')

% tightfig;
% saveName = "pso_result";
% print(saveName,'-dpdf','-bestfit');