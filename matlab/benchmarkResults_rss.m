% Performance comparison of the RSS with different planners.
% The experiment is in one scene with obstacles where the start and goal configurations
% are:
% State c_start = {0.5236, 1.74753, -1.8326, -1.4835,	1.5708,	0, 1.004278, 0.2729, 0.9486, -1.15011, 1.81001, -1.97739};
% State c_goal = {0.5236, 0.34907, 0.69813, -1.3963, 1.5708, 0, 0.7096, 1.8032, -1.7061, -1.6286, 1.9143, -2.0155}; // Robot 2 no backflip - Elbow down
% This while benchmarking the maximum step distance.
% With joint limits of the ABB robots.
% last updated: 09/26/17

clear all
clc

%%
planners = {'BiRRT','RRT','LazyRRT','SBL'};
plannerType = planners{4};
switch plannerType
    case 'BiRRT'
        D = load('Benchmark_BiRRT_SG_3poles_rB.txt'); D = D(D(:,2)==1,:);
    case 'RRT'
        D = load('Benchmark_RRT_SG_3poles_rB.txt'); D = D(D(:,2)==1,:);
    case 'LazyRRT'
        D = load('Benchmark_LazyRRT_SG_3poles_rB.txt'); %D = D(D(:,2)==1,:);
    case 'SBL'
        D = load('Benchmark_SBL_SG_3poles_rB.txt'); D = D(D(:,2)==1,:);
end

%% 
disp(['Results for ' plannerType ':']);

%%
rd = sort(unique(D(:,1)));
for i = 1:length(rd)
    M = D(D(:,1)==rd(i), 2:end);
    td(i) = mean(M(:,3));
    td_ste(i) = std(M(:,3))/sqrt(size(M,1));
end


%%
disp(' ');
[tdmin, id] = min(td);
% id = 2;
% tdmin = td(id);

%%
h = figure(1);
clf
errorbar(rd,td,td_ste,'-k','linewidth',2);
ylabel('mean runtime [sec]');
xlabel('max. local-connection distance');
% xlim([0 6]);
% xlim([min(rd) max(rd)]);

%% 
sS = rd(id);
tmin = tdmin;
D = D(D(:,1)==sS, 2:end);
verf = D(:,1)==1;
suc = D(:,2)==1;

disp('------------------------------------');
disp('RSS:');
disp(['Results of ' num2str(size(D,1)) ' queries.']);
disp(['Minimum avg. runtime is ' num2str(tmin) 'sec with d = ' num2str(sS) ]);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D(1,2)) ]);
disp(['Avg. runtime: ' num2str(mean(D(:,3)))  ' +/- ' num2str(std(D(:,3))/sqrt(size(D,1))) ' sec ']);
disp(['Min. runtime: ' num2str(min(D(:,3))) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D(:,9)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D(:,10)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D(:,4)))) ]);
disp(['Avg. local-connection time: ' num2str(mean(D(:,11)./D(:,12))*1e3)  ' +/- ' num2str(std(D(:,11)./D(:,12))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Avg. total local-connection time: ' num2str(mean(D(:,11))*1e3)  ' +/- ' num2str(std(D(:,11))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Avg. number of local connection checks: ' num2str(mean(D(:,12)))]);
disp(['Avg. collision check time: ' num2str(mean(D(:,7))*1e3) ' msec.']);
disp(['Percent of successful local connections: ' num2str(100*mean(D(:,13)./D(:,12)))]);

%%
%%
%%
td = D(:,3);
maxT = max(td);
T1 = linspace(0,maxT,5);
T1 = T1(2:end);
for i = 1:length(T1)
    sd = td < T1(i);
    md(i) = mean(td(sd));
    Md(i) = 1-sum(sd)/length(td);
end

%%
h = figure(2);
clf
plot(T1,Md*100,'-k','linewidth',2);
xlabel('maximum runtime [sec]');
ylabel('failure rate [%]');
xlim([0 max(T1)]);
title(plannerType);
