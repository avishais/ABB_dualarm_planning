clear all
clc

Dgd = load('gd_rbs_verification_withObs_env2.txt');
Dpcs = load('pcs_rbs_verification_withObs_env2.txt');
Drss = load('rss_rbs_verification_withObs_env2.txt');
Drlx = load('rlx_rbs_verification_eps0.5_withObs_env2.txt');
%% gd

bins = 30;

D = Dgd;
clear V d
Dd = D(:,3);
max_d = max(Dd);
d = linspace(0, max_d, bins);
% V = zeros(bins,1);
for i = 2:length(d)
    S = D(D(:,3)>=d(i-1) & D(:,3)<d(i), 1);
    V(i-1) = (sum(S)/length(S)) * 100;
end
Vgd = V;
dgd = d(2:end);

%% pcs

bins = 20;

D = Dpcs;
clear V d
Dd = D(:,3);
max_d = max(Dd);
d = linspace(0, max_d, bins);
% V = zeros(bins,1);
for i = 2:length(d)
    S = D(D(:,3)>=d(i-1) & D(:,3)<d(i), 1);
    V(i-1) = (sum(S)/length(S)) * 100;
end
Vpcs = V;
dpcs = d(2:end);

%% rss

bins = 20;

D = Drss;
Dd = D(:,3);
clear V d
max_d = max(Dd);
d = linspace(0, max_d, bins);
% V = zeros(bins,1);
for i = 2:length(d)
    S = D(D(:,3)>=d(i-1) & D(:,3)<d(i), 1);
    V(i-1) = (sum(S)/length(S)) * 100;
end
Vrss = V;
drss = d(2:end);

%% rlx

bins = 15;

D = Drlx;
Dd = D(:,3);
clear V d
max_d = max(Dd);
d = linspace(0, max_d, bins);
% V = zeros(bins,1);
for i = 2:length(d)
    S = D(D(:,3)>=d(i-1) & D(:,3)<d(i), 1);
    V(i-1) = (sum(S)/length(S)) * 100;
end
Vrlx = (V); %medfilt1
drlx = d(2:end);

%% plot

h = figure(1);
clf
plot(dpcs, Vpcs,'-k','linewidth',2)
hold on
plot(dgd, Vgd,'--k','linewidth',2)
plot(drss, Vrss,':k','linewidth',2)
plot(drlx, Vrlx,'-.k','linewidth',2)
hold off
legend('PCS','NR','RSS','RLX','location','northeast');
set(gca,'fontsize',14);
xlim([max([dpcs(1) dgd(1) drss(1) drlx(1)]) max([dpcs(end) dgd(end) drss(end) drlx(end)])]);
set(h, 'Position', [100, 100, 800, 320]);
xlabel('distance: $\sqrt{(\phi_1-\phi_2)^T(\phi_1-\phi_2)}$','Interpreter','latex');
ylabel('success rate (%)');
% print successRate2.eps -depsc -r200