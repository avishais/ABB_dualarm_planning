% Performance comparison between the reviewed methods for planning with PRM.
% The experiment is in one scene with obstacles where the start and goal configurations
% are:
% State c_start = {0.5236, 1.74753, -1.8326, -1.4835,	1.5708,	0, 1.004278, 0.2729, 0.9486, -1.15011, 1.81001, -1.97739};
% State c_goal = {0.5236, 0.349.007, 0.69813, -1.3963, 1.5708, 0, 0.7096, 1.8032, -1.7061, -1.6286, 1.9143, -2.0155}; // Robot 2 no backflip - Elbow down
% This while benchmarking the maximum step distance.
% With joint limits of the ABB robots.
% last updated: 10/04/17

clear all
clc

%%
D1 = load('Benchmark_PRM_GD_3poles_rB.txt');
D1 = D1(D1(:,2)==1,:);
D1 = D1(:,2:end);

D2 = load('Benchmark_PRM_PCS_3poles_rB.txt'); 
D2 = D2(D2(:,2)==1,:);
D2 = D2(:,2:end);

D3 = load('Benchmark_PRM_RLX_eps5_3poles_rB.txt'); 
% D3 = D3(D3(:,2)==1,:);
D3 = D3(:,2:end);

D4 = load('Benchmark_PRM_SG_3poles_rB.txt'); 
D4 = D4(D4(:,2)==1,:);
D4 = D4(:,2:end); % 113:end with sampling data

%% GD
suc = D1(:,1)==1;

D = D1;
disp('------------------------------------');
disp('GD:');
disp(['Results of ' num2str(size(D,1)) ' queries.']);
disp(['Avg. runtime: ' num2str(mean(D(:,3)))  ' +/- ' num2str(std(D(:,3))/sqrt(size(D,1))) ' sec ']);
disp(['Min. runtime: ' num2str(min(D(:,3))) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D(:,9)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D(:,10)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D(:,4)))) ]);
disp(['Avg. local-connection time: ' num2str(mean(D(:,11)./D(:,12))*1e3)  ' +/- ' num2str(std(D(:,11)./D(:,12))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Avg. total local-connection time: ' num2str(mean(D(:,11)))  ' +/- ' num2str(std(D(:,11))/sqrt(size(D,1))) ' sec ']);
disp(['Avg. number of local connection checks: ' num2str(mean(D(:,12)))]);
disp(['Local connection success rate: ' num2str(100*mean(D(:,13)./D(:,12))) '%.']);
disp(['Local connection checks per sec: ' num2str(mean(D(:,12)./D(:,3)))]);
disp(['Avg. collision check time: ' num2str(mean(D(:,7))) ' sec.']);
disp(['Avg. sampling time: ' num2str(mean(D(:,14)))  ' +/- ' num2str(std(D(:,14))/sqrt(size(D,1))) ' sec ']);
disp(['Avg. number of samples: ' num2str(mean(D(:,15)+D(:,16)))]);
disp(['Avg. number of successful samples: ' num2str(mean(D(:,15)))]);
disp(['Sampling fail rate: ' num2str(100-100*mean(D(:,15)./(D(:,15)+D(:,16)))) '%.']);

t = D(:,3);
maxT = max(t);
T = linspace(0,maxT,1000);
for i = 2:length(T)
    s = t < T(i);
    M(i) = 1-sum(s)/length(t);
end
M(1) = 1;
T1 = T;
M1 = M;

%% PCS
suc = D2(:,1)==1;

D = D2;
disp('------------------------------------');
disp('PCS:');
disp(['Results of ' num2str(size(D,1)) ' queries.']);
disp(['Avg. runtime: ' num2str(mean(D(:,3)))  ' +/- ' num2str(std(D(:,3))/sqrt(size(D,1))) ' sec ']);
disp(['Min. runtime: ' num2str(min(D(:,3))) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D(:,9)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D(:,10)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D(:,4)))) ]);
disp(['Avg. local-connection time: ' num2str(mean(D(:,11)./D(:,12))*1e3)  ' +/- ' num2str(std(D(:,11)./D(:,12))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Avg. total local-connection time: ' num2str(mean(D(:,11)))  ' +/- ' num2str(std(D(:,11))/sqrt(size(D,1))) ' sec ']);
disp(['Avg. number of local connection checks: ' num2str(mean(D(:,12)))]);
disp(['Local connection success rate: ' num2str(100*mean(D(:,13)./D(:,12))) '%.']);
disp(['Local connection checks per sec: ' num2str(mean(D(:,12)./D(:,3)))]);
disp(['Avg. collision check time: ' num2str(mean(D(:,7))) ' sec.']);
disp(['Avg. sampling time: ' num2str(mean(D(:,14)))  ' +/- ' num2str(std(D(:,14))/sqrt(size(D,1))) ' sec ']);
disp(['Avg. number of samples: ' num2str(mean(D(:,15)+D(:,16)))]);
disp(['Avg. number of successful samples: ' num2str(mean(D(:,15)))]);
disp(['Sampling fail rate: ' num2str(100-100*mean(D(:,15)./(D(:,15)+D(:,16)))) '%.']);

t = D(:,3);
maxT = max(t);
T = linspace(0,maxT,1000);
for i = 2:length(T)
    s = t < T(i);
    M(i) = 1-sum(s)/length(t);
end
M(1) = 1;
T2 = T;
M2 = M;

%% RLX
suc = D3(:,1)==1;

D = D3;
disp('------------------------------------');
disp('RLX:');
disp(['Results of ' num2str(size(D,1)) ' queries.']);
disp(['Avg. runtime: ' num2str(mean(D(:,3)))  ' +/- ' num2str(std(D(:,3))/sqrt(size(D,1))) ' sec ']);
disp(['Min. runtime: ' num2str(min(D(:,3))) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D(:,9)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D(:,10)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D(:,4)))) ]);
disp(['Avg. local-connection time: ' num2str(mean(D(:,11)./D(:,12))*1e3)  ' +/- ' num2str(std(D(:,11)./D(:,12))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Avg. total local-connection time: ' num2str(mean(D(:,11)))  ' +/- ' num2str(std(D(:,11))/sqrt(size(D,1))) ' sec ']);
disp(['Avg. number of local connection checks: ' num2str(mean(D(:,12)))]);
disp(['Local connection success rate: ' num2str(100*mean(D(:,13)./D(:,12))) '%.']);
disp(['Local connection checks per sec: ' num2str(mean(D(:,12)./D(:,3)))]);
disp(['Avg. collision check time: ' num2str(mean(D(:,7))) ' sec.']);
disp(['Avg. sampling time: ' num2str(mean(D(:,14)))  ' +/- ' num2str(std(D(:,14))/sqrt(size(D,1))) ' sec ']);
disp(['Avg. number of samples: ' num2str(mean(D(:,15)+D(:,16)))]);
disp(['Avg. number of successful samples: ' num2str(mean(D(:,15)))]);
disp(['Sampling fail rate: ' num2str(100-100*mean(D(:,15)./(D(:,15)+D(:,16)))) '%.']);

t = D(:,3);
maxT = max(t);
T = linspace(0,maxT,1000);
for i = 2:length(T)
    s = t < T(i);
    M(i) = 1-sum(s)/length(t);
end
M(1) = 1;
T3 = T;
M3 = M;

%% RSS
suc = D4(:,1)==1;

D = D4;
disp('------------------------------------');
disp('RSS:');
disp(['Results of ' num2str(size(D,1)) ' queries.']);
disp(['Avg. runtime: ' num2str(mean(D(:,3)))  ' +/- ' num2str(std(D(:,3))/sqrt(size(D,1))) ' sec ']);
disp(['Min. runtime: ' num2str(min(D(:,3))) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D(:,9)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D(:,10)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D(:,4)))) ]);
disp(['Avg. local-connection time: ' num2str(mean(D(:,11)./D(:,12))*1e3)  ' +/- ' num2str(std(D(:,11)./D(:,12))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Avg. total local-connection time: ' num2str(mean(D(:,11)))  ' +/- ' num2str(std(D(:,11))/sqrt(size(D,1))) ' sec ']);
disp(['Avg. number of local connection checks: ' num2str(mean(D(:,12)))]);
disp(['Local connection success rate: ' num2str(100*mean(D(:,13)./D(:,12))) '%.']);
disp(['Local connection checks per sec: ' num2str(mean(D(:,12)./D(:,3)))]);
disp(['Avg. collision check time: ' num2str(mean(D(:,7))) ' sec.']);
disp(['Avg. sampling time: ' num2str(mean(D(:,14)))  ' +/- ' num2str(std(D(:,14))/sqrt(size(D,1))) ' sec ']);
disp(['Avg. number of samples: ' num2str(mean(D(:,15)+D(:,16)))]);
disp(['Avg. number of successful samples: ' num2str(mean(D(:,15)))]);
disp(['Sampling fail rate: ' num2str(100-100*mean(D(:,15)./(D(:,15)+D(:,16)))) '%.']);

t = D(:,3);
maxT = max(t);
T = linspace(0,maxT,1000);
for i = 2:length(T)
    s = t < T(i);
    M(i) = 1-sum(s)/length(t);
end
M(1) = 1;
T4 = T;
M4 = M;

%%
%%
h = figure(2);
clf
plot(T1,M1*100,'-k','linewidth',2);
hold on
plot(T2,M2*100,'--k','linewidth',2);
plot(T3,M3*100,':k','linewidth',2);
plot(T4,M4*100,':k','linewidth',2);
hold off
xlabel('maximum runtime [sec]');
ylabel('failure rate [%]');
legend('NR','PCS','RLX','RSS');
xlim([0 max([T1 T2 T3 T4])]);
