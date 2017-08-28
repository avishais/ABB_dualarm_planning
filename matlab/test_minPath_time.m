% Test the average runtime to compute the path. This will give some notion
% about the minimal time required to find a path.
% The path was smoothed before runtime computation.
% Last updated: 8/28/2017

clear all
clc

%%
% PCS
D1 = load('benchmark_BiRRT_PCS_3poles_minCpath.txt');

Td = mean(D1(:,4));
Td_std = std(D1(:,4))/sqrt(size(D1,1));

Tt = mean(D1(:,end-1));
Tt_std = std(D1(:,end-1))/sqrt(size(D1,1));

L = mean(D1(:,end));

IKc = mean(D1(:,5));
IKt = mean(D1(:,6));

LCt = mean(D1(:,12));
LCc = mean(D1(:,13));

numMilestones = mean(D1(:,10));

disp('PCS:');
disp(['Avg. runtime: ' num2str(Td*1e3) ' msec +/- ' num2str(Td_std*1e3) ' msec.']);
disp(['Minimal computation of a path: ' num2str(Tt*1e3) ' msec +/- ' num2str(Tt_std*1e3) ' msec.']);
disp(['Avg. path length: ' num2str(L)]);
disp(['An avg. of ' num2str(IKc) ' projections were made in avg. time of ' num2str(IKt*1e3) ' msec.']);
disp(['An avg. of ' num2str(LCc) ' local connections were checked in avg. time of ' num2str(LCt*1e3) ' msec.']);
disp(['Avg. number of milestones: ' num2str(numMilestones)]);


%%
disp(' ');
%%
% GD
D2 = load('benchmark_BiRRT_GD_3poles_minCpath.txt');

Tg = mean(D2(:,4));
Tg_std = std(D2(:,4))/sqrt(size(D2,1));

Tt = mean(D2(:,end-1));
Tt_std = std(D2(:,end-1))/sqrt(size(D2,1));

L = mean(D2(:,end));

IKc = mean(D2(:,5));
IKt = mean(D2(:,6));

LCt = mean(D2(:,12));
LCc = mean(D2(:,13));

numMilestones = mean(D2(:,10));

disp('GD:');
disp(['Avg. runtime: ' num2str(Tg*1e3) ' msec +/- ' num2str(Tg_std*1e3) ' msec.']);
disp(['Minimal computation of a path: ' num2str(Tt*1e3) ' msec +/- ' num2str(Tt_std*1e3) ' msec.']);
disp(['Avg. path length: ' num2str(L)]);
disp(['An avg. of ' num2str(IKc) ' projections were made in avg. time of ' num2str(IKt*1e3) ' msec.']);
disp(['An avg. of ' num2str(LCc) ' local connections were checked in avg. time of ' num2str(LCt*1e3) ' msec.']);
disp(['Avg. number of milestones: ' num2str(numMilestones)]);
