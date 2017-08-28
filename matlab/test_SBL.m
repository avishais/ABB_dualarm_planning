% In this experiment I compare the performance of the GD (with KDL) and the
% PCS on one scene with obstacles where the start and goal configurations
% are:
% State c_start = {0.5236, 1.7453, -1.8326, -1.4835,	1.5708,	0, 1.004278, 0.2729, 0.9486, -1.15011, 1.81001, -1.97739};
% State c_goal = {0.5236, 0.34907, 0.69813, -1.3963, 1.5708, 0, 0.7096, 1.8032, -1.7061, -1.6286, 1.9143, -2.0155}; // Robot 2 no backflip - Elbow down
% This while benchmarking the maximum step distance.
% With joint limits of the ABB robots.
% SBL planner
% last updated: 08/27/17

clear all
clc

%%
D1 = load('benchmark_SBL_PCS_3poles_rangeB.txt'); 

%%
D2 = load('benchmark_SBL_GD_3poles_rangeB.txt'); 

%%
% PCS
rd = sort(unique(D1(:,1)));
for i = 1:length(rd)
    M = D1(D1(:,1)==rd(i), 2:end);
    td(i) = mean(M(:,4))*1e3;
    td_ste(i) = 1e3*std(M(:,4))/sqrt(size(M,1));
    nodes_d(i) =  mean(M(:,10));
    trees_d(i) =  mean(M(:,11));
    tlc_d(i) = mean(M(:,12))*1e3;
end

%%
% GD
rg = sort(unique(D2(:,1)));
for i = 1:length(rg)
    M = D2(D2(:,1)==rg(i), 2:end);
%     disp([i-1 rg(i)  size(M,1)])
    tg(i) = mean(M(:,4))*1e3;
    tg_ste(i) = 1e3*std(M(:,4))/sqrt(size(M,1));
    nodes_g(i) =  mean(M(:,10));
    trees_g(i) =  mean(M(:,11));
    tlc_g(i) = mean(M(:,12))*1e3;
end

%%
disp(' ');
[tdmin, id] = min(td);
[tgmin, ig] = min(tg);

%%
h = figure(1);
clf
errorbar(rd,td,td_ste,'-k','linewidth',2);
hold on
errorbar(rg,tg,tg_ste,'--k','linewidth',2);
hold off
ylabel('mean runtime [msec]');
xlabel('max. local-connection distance');
legend('PCS','GD');
% xlim([0 6]);
xlim([min(rd) max(rd)]);

%% 
D1 = D1(D1(:,1)==rd(id), 2:end);
verf = D1(:,1)==1;
suc = D1(:,2)==1;

disp('PCS:');
disp(['Results of ' num2str(size(D1,1)) ' queries.']);
disp(['Minimum avg. runtime for PCS is ' num2str(tdmin) 'msec with d = ' num2str(rd(id)) ]);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D1(1,3)) ]);
disp(['Avg. runtime: ' num2str(mean(D1(:,4))*1e3)  ' +/- ' num2str(std(D1(:,4))/sqrt(size(D1,1))*1e3) ' msec ']);
disp(['Avg. local-connection time: ' num2str(mean(D1(:,12)./D1(:,13))*1e3)  ' +/- ' num2str(std(D1(:,12)./D1(:,13))/sqrt(size(D1,1))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D1(:,10)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D1(:,11)))) ]);
disp(['Avg. number of IK solutions: ' num2str(floor(mean(D1(:,5)))) ]);

%%
D2 = D2(D2(:,1)==rg(ig), 2:end);
verf = D2(:,1)==1;
suc = D2(:,2)==1;

disp('GD:');
disp(['Results of ' num2str(size(D2,1)) ' queries.']);
disp(['Minimum avg. runtime for GD is ' num2str(tgmin) 'msec with d = ' num2str(rg(ig)) ]);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D2(1,3)) ]);
disp(['Avg. runtime: ' num2str(mean(D2(:,4))*1e3)  ' +/- ' num2str(std(D2(:,4))/sqrt(size(D2,1))*1e3) ' msec ']);
disp(['Avg. local-connection time: ' num2str(mean(D2(:,12)./D2(:,13))*1e3)  ' +/- ' num2str(std(D2(:,12)./D2(:,13))/sqrt(size(D2,1))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D2(:,10)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D2(:,11)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D2(:,5)))) ]);

%%
disp(' ');
disp(['Best speed-up: t_{pcs}/t_{gd} = ' num2str(tdmin/tgmin) ]);

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
h = figure(2);
clf
plot(T1,Md*100,'-k','linewidth',2);
hold on
plot(T2,Mg*100,'--k','linewidth',2);
hold off
xlabel('maximum runtime [sec]');
ylabel('failure rate [%]');
legend('PCS','GD');
xlim([0 max(T2)]);
title('LazyRRT');
