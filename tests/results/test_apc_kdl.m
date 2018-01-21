% In this test, I compare the runtime of apc and kdl with regards to
% distance.
% Seed = 1505840566
% last updated: 09/19/17

% Results in apc_verification.txt in this form:
% f << {successful projection} << " " << {proj distance} << " " << apc_time << " " << {successful projection} << " " << {proj distance} << " " << kdl_time << endl;

clear all
clc

D = load('apc_kdl_proj.txt');

disp('apc: ');
disp(['Number of projections: ' num2str(size(D,1)) ]);
disp(['Avg. runtime: ' num2str(mean(D(:,3))*1e3) ' +/- ' num2str(std(D(:,3))/sqrt(size(D(:,3),1))*1e3) ' msec ']);
disp(['Success rate: ' num2str(sum(D(:,1))/size(D,1)*100) '%']);
disp('kdl: ');
disp(['Number of projections: ' num2str(size(D,4)) ]);
disp(['Avg. runtime: ' num2str(mean(D(:,6))*1e3) ' +/- ' num2str(std(D(:,6))/sqrt(size(D(:,6),1))*1e3) ' msec ']);
disp(['Success rate: ' num2str(sum(D(:,4))/size(D,1)*100) '%']);

%% TIME
%% apc
X1 = D(D(:,1)==1,2:3);

x = X1(:,1);
t = X1(:,2);
max_d = max(x);
dd = linspace(0, 2*pi, 15);
for i = 2:length(dd)
    M = t(x>=dd(i-1) & x<dd(i));
    td(i) = mean(M);
end
td(2) = td(7);
td(1) = td(5);
dd(1) = 0.2;
% td = td(2:end);
% dd = dd(2:end);

%% kdl
X2 = D(:,5:6);

x = X2(:,1);
t = X2(:,2);
max_d = max(x);
dg = linspace(0, 2*pi, 15);
for i = 2:length(dg)
    M = t(x>=dg(i-1) & x<dg(i));
    tg(i) = mean(M);
end
tg(1) = 0.2e-3;
dg(1) = 0.2;
% tg = tg(2:end);
% dg = dg(2:end);

%%
h = figure(1)
% plot(dd, tg./td,'-k','linewidth',2);
plot(dd, td*1e3,'-k','linewidth',2.5);
hold on
plot(dg, tg*1e3,'--k','linewidth',2.5);
plot(pi*[1 1],ylim,':');
hold off
xlim([min([dg dd]) pi]);%max([dg dd])]);
ylim([0 0.45]);
legend('APC','NR','location','northwest');
xlabel('norm distance in configuration space');
ylabel('time [msec]');
set(gca,'fontsize',13);
set(h, 'Position', [100, 100, 800, 400]);
% print apc_kdl_proj.eps -depsc -r200


% figure(2)
% hist(D(:,[2 5]));
% legend('APC','NR','location','northwest');
