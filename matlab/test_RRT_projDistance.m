% In this experiment I test the projection distance and the possible fix by
% assigning a new nearest neighbor.
% are:
% State c_start = {0.5236, 1.7453, -1.8326, -1.4835,	1.5708,	0, 1.004278, 0.2729, 0.9486, -1.15011, 1.81001, -1.97739};
% State c_goal = {0.5236, 0.34907, 0.69813, -1.3963, 1.5708, 0, 0.7096, 1.8032, -1.7061, -1.6286, 1.9143, -2.0155}; // Robot 2 no backflip - Elbow down
% This while benchmarking the maximum step distance.
% With joint limits of the ABB robots.
% Tested with RRT and BiRRT planner
% last updated: 08/30/17

clear all
clc

planner = 2;

%%
if planner==2
    D1 = load('test_RRT_PCS_projDistance.txt');
else
    D1 = load('test_BiRRT_PCS_projDistance.txt');
end

D = D1;

rd = sort(unique(D(:,4)));

for i = 1:length(rd)
    M = D(D(:,4)==rd(i), :);
    dd1(i) = mean(M(:,1)); % distance before projection
    dd2(i) = mean(M(:,2)); % distance after projection
    dd3(i) = mean(M(:,3)); % distance after new NN found
    sd(i) = sum(M(:,5))/size(M,1);
end


%%
if planner==1
    D2 = load('test_RRT_GD_projDistance.txt');
else
    D2 = load('test_BiRRT_GD_projDistance.txt');
end

D = D2;

rg = sort(unique(D(:,4)));

for i = 1:length(rg)
    M = D(D(:,4)==rg(i), :);
    dg1(i) = mean(M(:,1)); % distance before projection
    dg2(i) = mean(M(:,2)); % distance after projection
    dg3(i) = mean(M(:,3)); % distance after new NN found
    sg(i) = sum(M(:,5))/size(M,1);
end

%%
if planner == 1
    lim = [1.1 2.5];
else
    lim = [2.5 3.3];
end

%%

figure(1)
subplot(221)
plot(rd, dd2,'-', rd, dd3,'-','linewidth',2);
hold on
plot(rg, dg2,'--', rg, dg3,'--','linewidth',2);
hold off
legend('PCS-before new NN','PCS-after new NN.','GD-before new NN.','GD-after new NN.','location','southeast');
xlabel('max. step length');
ylabel('distance from NN');
xlim([min([rd; rg]) max([rd; rg])]);
if planner==2
    title('RRT');
else
    title('BiRRT');
end

subplot(222)
plot(rd, dd2./rd','-',rd, dd3./rd','-','linewidth',2);
hold on
plot(rg, dg2./rd','--',rg, dg3./rd','--','linewidth',2);
hold off
xlabel('max. step length');
ylabel('ratio of max. step size');
legend('PCS-before new NN','PCS-after new NN.','GD-before new NN.','GD-after new NN.');
xlim([min([rd; rg]) max([rd; rg])]);

subplot(2,2,3:4)
plot(rd, sd*100, '-k','linewidth',2);
hold on
plot(rg, sg*100, '--k','linewidth',2);
plot(lim(1)*[1 1], ylim,':k','linewidth',2);
plot(lim(2)*[1 1], ylim,'-.k','linewidth',2);
hold off
xlabel('max. step length');
ylabel('failed local-conn.');
legend('PCS','GD','PCS-opt. step','GD-opt. step');
xlim([min([rd; rg]) max([rd; rg])]);
