% Performance comparison between the PCS, GD (with KDL) and the Hybrid
% approach. The experiment is in one scene with obstacles where the start and goal configurations
% are:
% State c_start = {0.5236, 1.7453, -1.8326, -1.4835,	1.5708,	0, 1.004278, 0.2729, 0.9486, -1.15011, 1.81001, -1.97739};
% State c_goal = {0.5236, 0.34907, 0.69813, -1.3963, 1.5708, 0, 0.7096, 1.8032, -1.7061, -1.6286, 1.9143, -2.0155}; // Robot 2 no backflip - Elbow down
% This while benchmarking the maximum step distance.
% With joint limits of the ABB robots.
% last updated: 09/04/17

clear all
clc

%%
planners = {'BiRRT','RRT','LazyRRT','SBL'};
plannerType = planners{1};
switch plannerType
    case 'BiRRT'
        D1 = load('Benchmark_BiRRT_PCS_3poles_rB.txt');
        D2 = load('Benchmark_BiRRT_GD_3poles_rB.txt');
        D3 = load('Benchmark_BiRRT_HB_3poles_rB.txt');
    case 'RRT'
        D1 = load('Benchmark_RRT_PCS_3poles_rB.txt'); D1 = D1(D1(:,3)==1,:);
        D2 = load('Benchmark_RRT_GD_3poles_rB.txt'); D2 = D2(D2(:,3)==1,:);
        D3 = load('Benchmark_RRT_HB_3poles_rB.txt'); D3 = D3(D3(:,3)==1,:);
    case 'LazyRRT'
        D1 = load('Benchmark_LazyRRT_PCS_3poles_rB.txt');
        D2 = load('Benchmark_LazyRRT_GD_3poles_rB.txt');
        D3 = load('Benchmark_LazyRRT_HB_3poles_rB.txt');
    case 'SBL'
        D1 = load('Benchmark_SBL_PCS_3poles_rB.txt');
        D2 = load('Benchmark_SBL_GD_3poles_rB.txt');
        D3 = load('Benchmark_SBL_HB_3poles_rB.txt');
end

%% 
disp(['Results for ' plannerType ':']);


%%
% PCS
rd = sort(unique(D1(:,1)));
for i = 1:length(rd)
    M = D1(D1(:,1)==rd(i), 2:end);
    td(i) = mean(M(:,4))*1e3;
    td_ste(i) = 1e3*std(M(:,4))/sqrt(size(M,1));
end

%%
% GD
rg = sort(unique(D2(:,1)));
for i = 1:length(rg)
    M = D2(D2(:,1)==rg(i), 2:end);
%     disp([i-1 rg(i)  size(M,1)])
    tg(i) = mean(M(:,4))*1e3;
    tg_ste(i) = 1e3*std(M(:,4))/sqrt(size(M,1));
end

%%
% HB
rb = sort(unique(D3(:,1)));
for i = 1:length(rb)
    M = D3(D3(:,1)==rb(i), 2:end);
%     disp([i-1 rg(i)  size(M,1)])
    tb(i) = mean(M(:,4))*1e3;
    tb_ste(i) = 1e3*std(M(:,4))/sqrt(size(M,1));
end

%%
disp(' ');
[tdmin, id] = min(td);
[tgmin, ig] = min(tg);
[tbmin, ib] = min(tb);

%%
h = figure(1);
clf
errorbar(rd,td,td_ste,'-k','linewidth',2);
hold on
errorbar(rg,tg,tg_ste,'--k','linewidth',2);
errorbar(rb,tb,tb_ste,':k','linewidth',2);
hold off
ylabel('mean runtime [msec]');
xlabel('max. local-connection distance');
legend('PCS','GD','HB');
% xlim([0 6]);
% xlim([min(rd) max(rd)]);

%% 
sS = rd(id);
tmin = tdmin;
D1 = D1(D1(:,1)==sS, 2:end);
verf = D1(:,1)==1;
suc = D1(:,2)==1;

D = D1;
disp('------------------------------------');
disp('PCS:');
disp(['Results of ' num2str(size(D,1)) ' queries.']);
disp(['Minimum avg. runtime is ' num2str(tmin) 'msec with d = ' num2str(sS) ]);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D(1,3)) ]);
disp(['Avg. runtime: ' num2str(mean(D(:,4))*1e3)  ' +/- ' num2str(std(D(:,4))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Min. runtime: ' num2str(min(D(:,4))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D(:,10)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D(:,11)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D(:,5)))) ]);
disp(['Avg. local-connection time: ' num2str(mean(D(:,12)./D(:,13))*1e3)  ' +/- ' num2str(std(D(:,12)./D(:,13))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Avg. number of local connection checks: ' num2str(mean(D(:,13)))]);
disp(['Percent of successful local connections: ' num2str(100*mean(D(:,14))/mean(D(:,13)))]);

%%
sS = rg(ig);
tmin = tgmin;
D2 = D2(D2(:,1)==sS, 2:end);
verf = D2(:,1)==1;
suc = D2(:,2)==1;

D = D2;
disp('------------------------------------');
disp('GD:');
disp(['Results of ' num2str(size(D,1)) ' queries.']);
disp(['Minimum avg. runtime is ' num2str(tmin) 'msec with d = ' num2str(sS) ]);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D(1,3)) ]);
disp(['Avg. runtime: ' num2str(mean(D(:,4))*1e3)  ' +/- ' num2str(std(D(:,4))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Min. runtime: ' num2str(min(D(:,4))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D(:,10)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D(:,11)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D(:,5)))) ]);
disp(['Avg. local-connection time: ' num2str(mean(D(:,12)./D(:,13))*1e3)  ' +/- ' num2str(std(D(:,12)./D(:,13))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Avg. number of local connection checks: ' num2str(mean(D(:,13)))]);
disp(['Percent of successful local connections: ' num2str(100*mean(D(:,14))/mean(D(:,13)))]);

%%
sS = rb(ib);
tmin = tbmin;
D3 = D3(D3(:,1)==sS, 2:end);
verf = D3(:,1)==1;
suc = D3(:,2)==1;

D = D3;
disp('------------------------------------');
disp('HB:');
disp(['Results of ' num2str(size(D,1)) ' queries.']);
disp(['Minimum avg. runtime is ' num2str(tmin) 'msec with d = ' num2str(sS) ]);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D(1,3)) ]);
disp(['Avg. runtime: ' num2str(mean(D(:,4))*1e3)  ' +/- ' num2str(std(D(:,4))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Min. runtime: ' num2str(min(D(:,4))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D(:,10)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D(:,11)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D(:,5)))) ]);
disp(['Avg. local-connection time: ' num2str(mean(D(:,12)./D(:,13))*1e3)  ' +/- ' num2str(std(D(:,12)./D(:,13))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Avg. number of local connection checks: ' num2str(mean(D(:,13)))]);
disp(['Percent of successful local connections: ' num2str(100*mean(D(:,14))/mean(D(:,13)))]);

%%
disp(' ');
disp(['Best speed-up: t_{pcs}/t_{gd} = ' num2str(tdmin/tgmin) ]);
disp(['Best speed-up: t_{hb}/t_{gd} = ' num2str(tbmin/tgmin) ]);
disp(['Best speed-up: t_{hb}/t_{pcs} = ' num2str(tbmin/tdmin) ]);

%%
%%
%%
% PCS
td = D1(:,4);
maxT = max(td);
T1 = linspace(0,maxT,100);
T1 = T1(2:end);
for i = 1:length(T1)
    sd = td < T1(i);
    md(i) = mean(td(sd));
    Md(i) = 1-sum(sd)/length(td);
end
%%
% GD
tg = D2(:,4);
maxT = max(tg);
T2 = linspace(0,maxT,1000);
T2 = T2(2:end);
for i = 1:length(T2)
    sg = tg < T2(i);
    mg(i) = mean(tg(sg));
    Mg(i) = 1-sum(sg)/length(tg);
end

%%
% HB
tb = D3(:,4);
maxT = max(tg);
T3 = linspace(0,maxT,1000);
T3 = T3(2:end);
for i = 1:length(T3)
    sb = tb < T3(i);
    mb(i) = mean(tb(sb));
    Mb(i) = 1-sum(sb)/length(tb);
end
%%
h = figure(2);
clf
plot(T1,Md*100,'-k','linewidth',2);
hold on
plot(T2,Mg*100,'--k','linewidth',2);
plot(T3,Mb*100,':k','linewidth',2);
hold off
xlabel('maximum runtime [sec]');
ylabel('failure rate [%]');
legend('PCS','GD','HB');
xlim([0 max([T1 T2])]);
title(plannerType);
