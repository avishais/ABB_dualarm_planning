% Test performance of SBL for planning on the ABB robots
% With joint limits of the ABB robots
% last updated: 08/25/17

clear all
clc


%%
D1 = load('benchmark_SBL_GD_3poles_range2.txt'); 
verf = D1(:,1)==1;
suc = D1(:,2)==1;

disp('GD:');
disp(['Results of ' num2str(size(D1,1)) ' queries.']);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D1(1,3)) ]);
disp(['Avg. runtime: ' num2str(mean(D1(:,4))*1e3)  ' +/- ' num2str(std(D1(:,4))/sqrt(size(D1,1))*1e3) ' msec ']);
disp(['Avg. local-connection time: ' num2str(mean(D1(:,12)./D1(:,13))*1e3)  ' +/- ' num2str(std(D1(:,12)./D1(:,13))/sqrt(size(D1,1))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D1(:,10)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D1(:,11)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D1(:,5)))) ]);

%%
D2 = load('benchmark_SBL_GD_3poles_range1.txt'); 
verf = D2(:,1)==1;
suc = D2(:,2)==1;

disp('GD:');
disp(['Results of ' num2str(size(D2,1)) ' queries.']);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D2(1,3)) ]);
disp(['Avg. runtime: ' num2str(mean(D2(:,4))*1e3)  ' +/- ' num2str(std(D2(:,4))/sqrt(size(D2,1))*1e3) ' msec ']);
disp(['Avg. local-connection time: ' num2str(mean(D2(:,12)./D2(:,13))*1e3)  ' +/- ' num2str(std(D2(:,12)./D2(:,13))/sqrt(size(D2,1))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D2(:,10)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D2(:,11)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D2(:,5)))) ]);

%%
D3 = load('benchmark_SBL_GD_3poles_range0.5.txt'); 
verf = D3(:,1)==1;
suc = D3(:,2)==1;

disp('GD:');
disp(['Results of ' num2str(size(D3,1)) ' queries.']);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D3(1,3)) ]);
disp(['Avg. runtime: ' num2str(mean(D3(:,4))*1e3)  ' +/- ' num2str(std(D3(:,4))/sqrt(size(D3,1))*1e3) ' msec ']);
disp(['Avg. local-connection time: ' num2str(mean(D3(:,12)./D3(:,13))*1e3)  ' +/- ' num2str(std(D3(:,12)./D3(:,13))/sqrt(size(D3,1))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D3(:,10)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D3(:,11)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D3(:,5)))) ]);

%%
% GD
tg = D1(:,4);
maxT = max(tg);
T1 = linspace(0,maxT,1000);
T1 = T1(2:end);
for i = 1:length(T1)
    sg = tg < T1(i);
    mg(i) = mean(tg(sg));
    Mg(i) = 1-sum(sg)/length(tg);
end
%%
h = figure(1);
clf
plot(T1,Mg*100,'--k','linewidth',2);
xlabel('maximum runtime [sec]');
ylabel('failure rate [%]');
legend('PCS','GD');
xlim([0 max([T1 T1])]);
title('RRT');
% set(h, 'Position', [100, 100, 800, 400]);

