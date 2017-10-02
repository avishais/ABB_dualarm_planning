% Performance comparison between the PCS, GD (with KDL) approaches. 
% The experiment is in one scene with obstacles where the start and goal configurations
% are: Env. II
% 		c_start = {1.1, 1.1, 0, 1.24, -1.5708, 0, -0.79567, 0.60136, 0.43858, -0.74986, -1.0074, -0.092294};
% 		c_goal = {-1.1, 1.35, -0.2, -1, -1.9, 0, 0.80875, 0.72363, -0.47891, -1.0484, 0.73278, 1.7491};
% This while benchmarking the maximum step distance.
% With joint limits of the ABB robots.
% last updated: 09/28/17

clear all
clc

%%
planners = {'BiRRT','RRT','LazyRRT','SBL'};
plannerType = planners{4};
switch plannerType
    case 'BiRRT'
        D1 = load('Benchmark_BiRRT_PCS_3poles_rB.txt'); D1 = D1(D1(:,2)==1,:);
        D2 = load('Benchmark_BiRRT_GD_3poles_rB.txt'); D2 = D2(D2(:,2)==1,:);
    case 'RRT'
        D1 = load('Benchmark_RRT_PCS_3poles_rB.txt');  D1 = D1(D1(:,2)==1,:);
        D2 = load('Benchmark_RRT_GD_3poles_rB.txt'); D2 = D2(D2(:,2)==1,:);
    case 'LazyRRT'
        D1 = load('Benchmark_LazyRRT_PCS_3poles_rB.txt'); D1 = D1(D1(:,2)==1,:); D1 = D1(D1(:,1)~=0.8 & D1(:,1)~=1.2,:);
        D2 = load('Benchmark_LazyRRT_GD_3poles_rB.txt');  D2 = D2(D2(:,2)==1,:);
    case 'SBL'
        D1 = load('Benchmark_SBL_PCS_3poles_rB.txt'); D1 = D1(D1(:,2)==1,:);
        D2 = [load('Benchmark_SBL_GD_3poles_rB_1.txt'); load('Benchmark_SBL_GD_3poles_rB_2.txt')]; D2 = D2(D2(:,2)==1,:);
        D2 = D2(D2(:,2)==1,:);
end

%% 
disp(['Results for ' plannerType ':']);

%%
% PCS
rd = sort(unique(D1(:,1)));
for i = 1:length(rd)
    M = D1(D1(:,1)==rd(i), 2:end);
    td(i) = mean(M(:,3))*1e3;
    td_ste(i) = 1e3*std(M(:,4))/sqrt(size(M,1));
end

%%
% GD
rg = sort(unique(D2(:,1)));
for i = 1:length(rg)
    M = D2(D2(:,1)==rg(i), 2:end);
%     disp([i-1 rg(i)  size(M,1)])
    tg(i) = mean(M(:,3))*1e3;
    tg_ste(i) = 1e3*std(M(:,4))/sqrt(size(M,1));
end

%%
disp(' ');
[tdmin, id] = min(td);
[tgmin, ig] = min(tg);

%%
h = figure(1);
clf
plot(rd,td,'-k','linewidth',2);
% errorbar(rd,td,td_ste,'-k','linewidth',2);
hold on
plot(rg,tg,'--k','linewidth',2);
% errorbar(rg,tg,tg_ste,'--k','linewidth',2);
hold off
ylabel('mean runtime [msec]');
xlabel('max. local-connection distance');
legend('PCS','GD');
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
disp(['Plan distance: ' num2str(D(1,3)) ]);
disp(['Avg. runtime: ' num2str(mean(D(:,3))*1e3)  ' +/- ' num2str(std(D(:,3))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Min. runtime: ' num2str(min(D(:,3))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D(:,9)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D(:,10)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D(:,4)))) ]);
disp(['Avg. local-connection time: ' num2str(mean(D(:,11)./D(:,12))*1e3)  ' +/- ' num2str(std(D(:,11)./D(:,12))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Avg. total local-connection time: ' num2str(mean(D(:,11))*1e3)  ' +/- ' num2str(std(D(:,11))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Avg. number of local connection checks: ' num2str(mean(D(:,12)))]);
disp(['Avg. collision check time: ' num2str(mean(D(:,7))*1e3) ' msec.']);
disp(['Percent of successful local connections: ' num2str(100*mean(D(:,13)./D(:,12)))]);

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
disp(['Plan distance: ' num2str(D(1,3)) ]);
disp(['Avg. runtime: ' num2str(mean(D(:,3))*1e3)  ' +/- ' num2str(std(D(:,3))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Min. runtime: ' num2str(min(D(:,3))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D(:,9)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D(:,10)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D(:,4)))) ]);
disp(['Avg. local-connection time: ' num2str(mean(D(:,11)./D(:,12))*1e3)  ' +/- ' num2str(std(D(:,11)./D(:,12))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Avg. total local-connection time: ' num2str(mean(D(:,11))*1e3)  ' +/- ' num2str(std(D(:,11))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Avg. number of local connection checks: ' num2str(mean(D(:,12)))]);
disp(['Avg. collision check time: ' num2str(mean(D(:,7))*1e3) ' msec.']);
disp(['Percent of successful local connections: ' num2str(100*mean(D(:,13)./D(:,12)))]);

%%
disp(' ');
disp(['Best speed-up: t_{pcs}/t_{gd} = ' num2str(tdmin/tgmin) ]);

%%
%%
%%
% PCS
td = D1(:,3);
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
tg = D2(:,3);
maxT = max(tg);
T2 = linspace(0,maxT,100);
T2 = T2(2:end);
for i = 1:length(T2)
    sg = tg < T2(i);
    mg(i) = mean(tg(sg));
    Mg(i) = 1-sum(sg)/length(tg);
end

%%
h = figure(2);
clf
plot(T1,Md*100,'-k','linewidth',2);
hold on
plot(T2,Mg*100,'--k','linewidth',2);
hold off
xlabel('maximum runtime (sec)');
ylabel('failure rate (%)');
legend('PCS','NR');
xlim([0 max([T1 T2])]);
% title(plannerType);
set(gca,'fontsize',13);
% set(h, 'Position', [100, 100, 800, 400]);
% print PCS_NR_runtime.eps -depsc -r200