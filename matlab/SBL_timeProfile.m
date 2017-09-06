clear all
clc

%%

D = load('benchmark_SBL_GD_3poles_profileTime_JL.txt');
D = D(:,2:end);

totalTime = D(:,3)*1e3;
lc_time = D(:,11)*1e3;
lc_checks = D(:,12);
lc_suc_checks = D(:,13);

projSuccessTime = D(:,14)*1e3;
projFailedTime = D(:,15)*1e3;

numSuccess = D(:,16);
numFailed = D(:,17);

lcSuccessTime = D(:,18)*1e3;
lcFailedTime = D(:,19)*1e3;

%%

fprintf('Avg. time: %.2f msec.\n', mean(totalTime));

fprintf('\nAvg. successful projection time (of a random node): %.2f msec.\n', mean(projSuccessTime));
fprintf('Avg. failed projection time (of a random node): %.2f msec.\n', mean(projFailedTime));
fprintf('Avg. number of successful projections: %d.\n', round(mean(numSuccess)));
fprintf('Avg. number of failed projections: %d.\n', round(mean(numFailed)));

fprintf('\nAvg. local connection time: %.2f msec.\n', mean(lc_time));
fprintf('Avg. successful LC time (of a random node): %.2f msec.\n', mean(lcSuccessTime));
fprintf('Avg. failed LC time (of a random node): %.2f msec.\n', mean(lcFailedTime));
fprintf('Avg. number of successful LC checks: %d.\n', round(mean(lc_suc_checks)));
fprintf('Avg. number of failed LC checks: %d.\n', round(mean(lc_checks-lc_suc_checks)));

%%
P = [mean(lcSuccessTime) mean(lcFailedTime) mean(projSuccessTime) mean(projFailedTime)];
P = [P mean(totalTime)-sum(P)];


figure(1);
pie(P);
legend('LC success','LC failed','suc. proj.','failed proj.','misc');