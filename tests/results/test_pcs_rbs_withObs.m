% In this test, I test failure rate of local connections while considering
% the ABB joint limits and obstacles. This should give an idea about the
% performance of the Lazy strategy
% The success rate is also tracked to calculate visibility.
% last updated: 08/29/17

% pcs_rbs_verification.txt
% seed: 1504026552
% Results in the above files in this form:
% f << {success} << " " << {path verified} << " " << {distance between confs.} << " " << rbs_time << endl;

clear all
clc

D = load('pcs_rbs_verification_withObs (copy).txt');
% D = load('pcs_rbs_verification_withObs_env1.txt');
% D = load('pcs_rbs_verification_withObs_env2.txt');

% D = load('pcs_rbs_verification_withObs_env1_distMix.txt');

% D = D(1:5e4,:);

%% Verification

suc = D(:,1)==1;
verf = D(:,2)==1;

disp(['Results of ' num2str(size(D,1)) ' local-connection queries.']);

disp(['Percent of successful local-connections that were verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);

%%

disp(['Avg. runtime for successful local-connection: ' num2str(mean(D(suc,4))*1e3)  ' +/- ' num2str(std(D(suc,4))/sqrt(size(D(suc,4),1))*1e3) ' msec ']);
disp(['Avg. runtime for failed local-connection: ' num2str(mean(D(~suc,4))*1e3)  ' +/- ' num2str(std(D(suc,4))/sqrt(size(D(~suc,4),1))*1e3) ' msec ']);

disp(['Success rate/visibility for the RBS: ' num2str(sum(suc)/size(D,1)*100) '%']);

%% Runtime

clear td d
Dd = D(suc,3);
Td = D(suc,4);
max_d = max(Dd);
d = linspace(0, max_d, 8);
for i = 2:length(d)
    M = Td(Dd>=d(i-1) & Dd<d(i));
    td(i) = mean(M);
end
td(1) = 0;

figure(1)
hist(Dd, 40);
hold on
[hAx,hLine1,hLine2] = plotyy(0,0,d, td*1000);
hold off
set(hLine2,'color','k','linewidth',2)
set(hAx,{'ycolor'},{'b';'k'})
xlabel('distance');
ylabel(hAx(1),'number of points');
ylabel(hAx(2),'computation time [msec]');
% ylim(hAx(1),[0 8500]);
% ylim(hAx(2),[0 0.12]);

%% Check uniformity

figure(3)
hist(D(:,3));

%% Visibility

clear V d
Dd = D(:,3);
max_d = max(Dd);
d = linspace(0, max_d, 100);
for i = 2:length(d)
    S = D(D(:,3)>=d(i-1) & D(:,3)<d(i), 1);
    V(i-1) = (1-sum(S)/length(S)) * 100;
end
% V(1) = 100;

figure(2)
hist(Dd, 40);
hold on
[hAx,hLine1,hLine2] = plotyy(0,0, d(2:end), V);
hold off
set(hLine2,'color','k','linewidth',2)
set(hAx,{'ycolor'},{'b';'k'})
xlabel('distance');
ylabel(hAx(2),'failure rate');
ylabel(hAx(1),'number of points');
title('PCS');
grid on