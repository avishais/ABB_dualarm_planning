% In this test, I project 5e6 random points with the kdl and verify the constraint.
% Seed = 1501600724
% last updated: 08/01/17

% Results in kdl_verification.txt in this form:
% f << {successful projection} << " " << {verified constraint} << " " << kdl_time << endl;

clear all
clc

D = load('kdl_verification.txt');

disp(['Number of projections: ' num2str(size(D,1)) ]);
disp(['Avg. runtime: ' num2str(mean(D(:,3))*1e3) ' +/- ' num2str(std(D(:,3))/sqrt(size(D(:,3),1))*1e3) ' msec ']);

disp(['Success rate: ' num2str(sum(D(:,1))/size(D,1)*100) '%']);
disp(['Successful constraint verification: ' num2str(sum(D(:,2))/size(D,1)*100) '%']);


