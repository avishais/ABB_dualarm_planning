% In this experiment I compare the performance of the GD (with KDL) and the
% PCS on one scene with obstacles where the start and goal configurations
% are:
% State c_start = {0.5236, 1.7453, -1.8326, -1.4835,	1.5708,	0, 1.004278, 0.2729, 0.9486, -1.15011, 1.81001, -1.97739};
% State c_goal = {0.5236, 0.34907, 0.69813, -1.3963, 1.5708, 0, 0.7096, 1.8032, -1.7061, -1.6286, 1.9143, -2.0155}; // Robot 2 no backflip - Elbow down
% This while benchmarking the maximum step distance.
% With joint limits of the ABB robots.
% BiRRR planner
% last updated: 08/03/17

clear all
clc

%%
D1 = load('benchmark_PCS_3poles_rangeB.txt'); 
verf = D1(:,2)==1;
suc = D1(:,3)==1;

disp('PCS:');
disp(['Results of ' num2str(size(D1,1)) ' queries.']);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D1(1,4)) ]);
disp(['Avg. runtime: ' num2str(mean(D1(:,5))*1e3)  ' +/- ' num2str(std(D1(:,5))/sqrt(size(D1,1))*1e3) ' msec ']);
disp(['Avg. local-connection time: ' num2str(mean(D1(:,13))*1e3)  ' +/- ' num2str(std(D1(:,13))/sqrt(size(D1,1))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D1(:,11)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D1(:,12)))) ]);
disp(['Avg. number of IK solutions: ' num2str(floor(mean(D1(:,6)))) ]);

%%
D2 = load('benchmark_GD_3poles_rangeB.txt'); 
verf = D2(:,2)==1;
suc = D2(:,3)==1;

disp('GD:');
disp(['Results of ' num2str(size(D2,1)) ' queries.']);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D2(1,4)) ]);
disp(['Avg. runtime: ' num2str(mean(D2(:,5))*1e3)  ' +/- ' num2str(std(D2(:,5))/sqrt(size(D2,1))*1e3) ' msec ']);
disp(['Avg. local-connection time: ' num2str(mean(D2(:,13))*1e3)  ' +/- ' num2str(std(D2(:,13))/sqrt(size(D2,1))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D2(:,11)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D2(:,12)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D2(:,6)))) ]);

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
disp(['Minimum avg. runtime for PCS is ' num2str(tdmin) 'msec with d = ' num2str(rd(id)) ]);
[tgmin, ig] = min(tg);
disp(['Minimum avg. runtime for GD is ' num2str(tgmin) 'msec with d = ' num2str(rg(ig)) ]);
disp(['Best speed-up: t_{pcs}/t_{gd} = ' num2str(tdmin/tgmin) ]);

%%
h = figure(1);
clf
subplot(211)
errorbar(rd,td,td_ste,'-k','linewidth',2);
hold on
errorbar(rg,tg,tg_ste,'--k','linewidth',2);
hold off
ylabel('mean runtime [msec]');
xlabel('max. local-connection distance');
legend('PCS','GD');
% xlim([0 6]);
xlim([0 max(rd)]);

subplot(212)
plot(rd,td./tg,'-k','linewidth',2);
hold on
plot(xlim, ones(2,1),':k');
ylabel('t_{pcs}/t_{gd}');
xlabel('max. local-connection distance');
xlim([0 max(rd)]);

% set(h, 'Position', [100, 100, 800, 800]);


% figure(2);
% clf
% subplot(211)
% plot(rd,nodes_d,'-k','linewidth',2);
% hold on
% plot(rg,nodes_g,'--k','linewidth',2);
% hold off
% ylabel('path length');
% xlabel('max. local-connection distance');
% legend('PCS','GD');
% 
% subplot(212)
% plot(rd,trees_d,'-k','linewidth',2);
% hold on
% plot(rg,trees_g,'--k','linewidth',2);
% hold off
% ylabel('path length');
% xlabel('max. local-connection distance');
% legend('PCS','GD');

