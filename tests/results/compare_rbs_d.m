clear all
clc

Dgd = load('gd_rbs_verification_withObs_d.txt');
Dpcs = load('pcs_rbs_verification_withObs_d1.txt');

clear Vgd Vpcs 
clear dgd dpcs 
%% gd

bins = 40;

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

bins = 40;

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



%% plot

figure(1);
clf
hold on
plot(dpcs, Vpcs,'-k','linewidth',2);
plot(dgd, Vgd,'--k','linewidth',2)
hold off
legend('PCS','NR','location','northeast');
set(gca,'fontsize',14);
xlim([max([dpcs(1) dgd(1)]) max([dpcs(end) dgd(end)])]);
xlabel('distance: $\sqrt{(\phi_1-\phi_2)^T(\phi_1-\phi_2)}$','Interpreter','latex');
ylabel('success rate (%)');


%%
%%

Dpcs = sortrows(Dpcs, 2);
Dgd = sortrows(Dgd, 2);

%% gd with d

bins = 40;

D = Dgd;
clear M d
Dd = D(:,2);
max_d = max(Dd);
d = linspace(0, max_d, bins);
% V = zeros(bins,1);
for i = 2:length(d)
    S = D(D(:,2)>=d(i-1) & D(:,2)<d(i), 3);
    M(i-1) = mean(S);
end
Mgd = M;
dgd = d(2:end);

%% pcs with d

bins = 40;

D = Dpcs;
clear M d
Dd = D(:,2);
max_d = max(Dd);
d = linspace(0, max_d, bins);
% V = zeros(bins,1);
for i = 2:length(d)
    S = D(D(:,2)>=d(i-1) & D(:,2)<d(i), 3);
    M(i-1) = mean(S);
end
Mpcs = M;
dpcs = d(2:end);


%%
h = figure(2);
plot(dpcs,Mpcs,'-k','linewidth',2);
hold on
plot(dgd,Mgd,'--k','linewidth',2);
hold off
legend('APC','NR','location','southeast');
set(gca,'fontsize',14);
xlim([max([dpcs(1) dgd(1)]) 7]);
xlabel('distance before proj.');
ylabel('distance after proj.');
set(h, 'Position', [100, 100, 800, 280]);
print dd.eps -depsc -r200

