% last updated: 09/27/17

clear all
clc

D1 = load('Benchmark_BiRRT_PCS_3poles_rB.txt');
D2 = load('Benchmark_BiRRT_GD_3poles_rB.txt');
D3 = load('Benchmark_BiRRT_RLX_eps5_3poles_rB.txt');
D4 = load('Benchmark_BiRRT_SG_3poles_rB.txt');

%% PCS
d = 1;

clear t M
t = D1(D1(:,1)==d, 5);
maxT = max(t);
T = linspace(0,maxT,100);
T = T(2:end);
for i = 1:length(T)
    s = t < T(i);
    m(i) = mean(t(s));
    M(i) = 1-sum(s)/length(t);
end
T1 = T;
M1 = M;
%% GD
d = 2.6;

clear t M
t = D2(D2(:,1)==d, 5);
maxT = max(t);
T = linspace(0,maxT,100);
T = T(2:end);
for i = 1:length(T)
    s = t < T(i);
    m(i) = mean(t(s));
    M(i) = 1-sum(s)/length(t);
end
T2 = T;
M2 = M;
%% RLX
d = 0.3;

clear t M
t = D3(D3(:,1)==d, 4);
maxT = max(t);
T = linspace(0,maxT,50);
T = T(2:end);
for i = 1:length(T)
    s = t < T(i);
    m(i) = mean(t(s));
    M(i) = 1-sum(s)/length(t);
end
T3 = [0 T];
M3 = [1 M];
%% RSS
d = 0.6;

clear t M
t = D4(D4(:,1)==d,4);
maxT = max(t);
T = linspace(0,maxT,30);
T = T(2:end);
for i = 1:length(T)
    s = t < T(i);
    m(i) = mean(t(s));
    M(i) = 1-sum(s)/length(t);
end
T4 = [0 T T3(end)];
M4 = [1 M 0];
%%
h = figure(2);
clf

subplot(121)
plot(T1,M1*100,'-k','linewidth',2);
hold on
plot(T2,M2*100,'--k','linewidth',2);
hold off
xlabel('maximum runtime (sec)');
ylabel('failure rate (%)');
legend('PCS','NR');
xlim([0 1.2]);%max([T1 T2])]);
set(gca,'fontsize',13);

subplot(122)
plot(T3,M3*100,'-k','linewidth',2);
hold on
plot(T4,M4*100,':k','linewidth',2);
hold off
xlabel('maximum runtime (sec)');
ylabel('failure rate (%)');
legend('RLX','RSS');
xlim([0 60]);%max([T3 T4])]);
set(gca,'fontsize',13);

set(h, 'Position', [100, 100, 800, 300]);
% print runtimeProfile.eps -depsc -r200