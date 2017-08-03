% In this test, I project random points with the APC (Active-Passive Chains) and verify the constraint.
% Unsuccessful projections were disregarded.
% Seed = 1501609019
% last updated: 08/01/17

% Results in apc_verification.txt in this form:
% f << {successful projection} << " " << {verified constraint} << " " << apc_time << endl;

clear all
clc

D = load('apc_verification.txt');

disp(['Number of projections: ' num2str(size(D,1)) ]);
disp(['Avg. runtime: ' num2str(mean(D(:,3))*1e3) ' +/- ' num2str(std(D(:,3))/sqrt(size(D(:,3),1))*1e3) ' msec ']);

disp(['Success rate: ' num2str(sum(D(:,1))/size(D,1)*100) '%']);
disp(['Successful constraint verification: ' num2str(sum(D(:,2))/size(D,1)*100) '%']);


