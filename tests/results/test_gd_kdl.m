% In this test, I compare the projection of the kdl and my own gd.
% Projection of random points and compare distance of projectio.
% Seed = 1501558517
% last updated: 07/30/17

% Results in kdl_gd.txt in this form:
%  f << {kdl success} << " " << kdl_time << " " << {kdl projection distance} << " " << {gd success} << " " << gd_time << " " << {gd projection distance} << endl;


clear all
clc

D = load('kdl_gd.txt');

figure(1)
plot(D(:,3),D(:,6),'.k');
xlabel('distance with KDL');
ylabel('distance with GD');

disp(['Avg. runtime for the KDL: ' num2str(mean(D(:,2))*1e3) 'msec']);
disp(['Avg. runtime for the GD: ' num2str(mean(D(:,5))*1e3) 'msec']);

disp(['Success rate for the KDL: ' num2str(sum(D(:,1))/size(D,1)*100) '%']);
disp(['Success rate for the GD: ' num2str(sum(D(:,4))/size(D,1)*100) '%']);


