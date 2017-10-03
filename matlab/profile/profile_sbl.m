clear all
clc

%%

D = load('profile_SBL_GD_3poles.txt');

suc = sum(D(:,1));

total_runtime = mean(D(:,3));
proj_count = round(mean(D(:,4)));
proj_time = mean(D(:,5));
col_count = round(mean(D(:,6)));
col_time = mean(D(:,7));
lc_count = round(mean(D(:,12)));
lc_time = mean(D(:,11));
lc_success_count = round(mean(D(:,13)));
sample_time = mean(D(:,14));
sample_success_count = round(mean(D(:,15)));
sample_fail_count = round(mean(D(:,16)));
path_size = mean(D(:,9));
trees_size = mean(D(:,10));

fprintf('NR:\n');
fprintf('Total runtime: %f sec.\n', total_runtime);
fprintf('Sampling time %f sec, is %f%% of the total runtime,\n', sample_time, sample_time/total_runtime*100);
fprintf('number of samples per sec: %d.\n', (sample_success_count+sample_fail_count)/sample_time);
fprintf('Failed %f%% of %d samples.\n', sample_fail_count/(sample_fail_count+sample_success_count)*100, sample_fail_count+sample_success_count);
fprintf('Succeeded %f%% of %d local-conenction checks.\n', lc_success_count/lc_count*100, lc_count);
fprintf('Local connection time: %f msec, is %f%% of the total runtime.\n', lc_time*1e3, lc_time/total_runtime*100);
fprintf('Avg. local connection time: %f msec.\n', lc_time/lc_count*1e3);
fprintf('number of local connections per sec: %d\n', lc_count/total_runtime);
fprintf('Collision time %f msec for %d checks.\n', col_time*1e3, col_count);
fprintf('Path size: %d.\n', round(path_size));
fprintf('Trees size: %d.\n\n', round(trees_size));

ygd = [sample_time lc_time col_time];
LC(:,1) =  D(:,11)./D(:,12)*1e3;

%%

D = load('profile_SBL_PCS_3poles.txt');

suc = sum(D(:,1));

total_runtime = mean(D(:,3));
proj_count = round(mean(D(:,4)));
proj_time = mean(D(:,5));
col_count = round(mean(D(:,6)));
col_time = mean(D(:,7));
lc_count = round(mean(D(:,12)));
lc_time = mean(D(:,11));
lc_success_count = round(mean(D(:,13)));
sample_time = mean(D(:,14));
sample_success_count = round(mean(D(:,15)));
sample_fail_count = round(mean(D(:,16)));
path_size = mean(D(:,9));
trees_size = mean(D(:,10));

fprintf('PCS:\n');
fprintf('Total runtime: %f sec.\n', total_runtime);
fprintf('Sampling time %f sec, is %f%% of the total runtime,\n', sample_time, sample_time/total_runtime*100);
fprintf('number of samples per sec: %d.\n', (sample_success_count+sample_fail_count)/sample_time);
fprintf('Failed %f%% of %d samples.\n', sample_fail_count/(sample_fail_count+sample_success_count)*100, sample_fail_count+sample_success_count);
fprintf('Succeeded %f%% of %d local-conenction checks.\n', lc_success_count/lc_count*100, lc_count);
fprintf('Local connection time: %f msec, is %f%% of the total runtime.\n', lc_time*1e3, lc_time/total_runtime*100);
fprintf('Avg. local connection time: %f msec.\n', lc_time/lc_count*1e3);
fprintf('number of local connections per sec: %d\n', lc_count/total_runtime);
fprintf('Collision time %f msec for %d checks.\n', col_time*1e3, col_count);
fprintf('Path size: %d.\n', round(path_size));
fprintf('Trees size: %d.\n\n', round(trees_size));

ypcs = [sample_time lc_time col_time];
LC(:,2) =  D(:,11)./D(:,12)*1e3;

%%

D = load('profile_SBL_RLX_3poles.txt');

suc = sum(D(:,1));

total_runtime = mean(D(:,3));
proj_count = round(mean(D(:,4)));
proj_time = mean(D(:,5));
col_count = round(mean(D(:,6)));
col_time = mean(D(:,7));
lc_count = round(mean(D(:,12)));
lc_time = mean(D(:,11));
lc_success_count = round(mean(D(:,13)));
sample_time = mean(D(:,14));
sample_success_count = round(mean(D(:,15)));
sample_fail_count = round(mean(D(:,16)));
path_size = mean(D(:,9));
trees_size = mean(D(:,10));

fprintf('RLX:\n');
fprintf('Total runtime: %f sec.\n', total_runtime);
fprintf('Sampling time %f sec, is %f%% of the total runtime,\n', sample_time, sample_time/total_runtime*100);
fprintf('number of samples per sec: %d.\n', (sample_success_count+sample_fail_count)/sample_time);
fprintf('Failed %f%% of %d samples.\n', sample_fail_count/(sample_fail_count+sample_success_count)*100, sample_fail_count+sample_success_count);
fprintf('Succeeded %f%% of %d local-conenction checks.\n', lc_success_count/lc_count*100, lc_count);
fprintf('Local connection time: %f msec, is %f%% of the total runtime.\n', lc_time*1e3, lc_time/total_runtime*100);
fprintf('Avg. local connection time: %f msec.\n', lc_time/lc_count*1e3);
fprintf('number of local connections per sec: %d\n', lc_count/total_runtime);
fprintf('Collision time %f msec for %d checks.\n', col_time*1e3, col_count);
fprintf('Path size: %d.\n', round(path_size));
fprintf('Trees size: %d.\n\n', round(trees_size));

yrlx = [sample_time lc_time col_time];
LC(:,3) =  D(:,11)./D(:,12)*1e3;

%%

D = load('profile_SBL_SG_3poles.txt'); 
D = D(D(:,1)==1, :);

suc = sum(D(:,1));

total_runtime = mean(D(:,3));
proj_count = round(mean(D(:,4)));
proj_time = mean(D(:,5));
col_count = round(mean(D(:,6)));
col_time = mean(D(:,7));
lc_count = round(mean(D(:,12)));
lc_time = mean(D(:,11));
lc_success_count = round(mean(D(:,13)));
sample_time = mean(D(:,14));
sample_success_count = round(mean(D(:,15)));
sample_fail_count = round(mean(D(:,16)));
path_size = mean(D(:,9));
trees_size = mean(D(:,10));

fprintf('RSS:\n');
fprintf('Total runtime: %f sec.\n', total_runtime);
fprintf('Sampling time %f sec, is %f%% of the total runtime,\n', sample_time, sample_time/total_runtime*100);
fprintf('number of samples per sec: %d.\n', (sample_success_count+sample_fail_count)/sample_time);
fprintf('Failed %f%% of %d samples.\n', sample_fail_count/(sample_fail_count+sample_success_count)*100, sample_fail_count+sample_success_count);
fprintf('Succeeded %f%% of %d local-conenction checks.\n', lc_success_count/lc_count*100, lc_count);
fprintf('Local connection time: %f msec, is %f%% of the total runtime.\n', lc_time*1e3, lc_time/total_runtime*100);
fprintf('Avg. local connection time: %f msec.\n', lc_time/lc_count*1e3);
fprintf('number of local connections per sec: %d\n', lc_count/total_runtime);
fprintf('Collision time %f msec for %d checks.\n', col_time*1e3, col_count);
fprintf('Path size: %d.\n', round(path_size));
fprintf('Trees size: %d.\n\n', round(trees_size));

yrss = [sample_time lc_time col_time];
% LC(:,4) =  D(:,11)./D(:,12)*1e3;

%%
figure(1)
clf
subplot(121)
bar([ygd; ypcs],'stacked');
legend('sample','LC','collision');

subplot(122)
bar([yrlx; yrss],'stacked');
legend('sample','LC','collision');