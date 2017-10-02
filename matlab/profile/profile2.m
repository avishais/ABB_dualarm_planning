clear all
clc

%%

D = load('profile_BiRRT_GD_3poles.txt');

i = 1:size(D,1);
total_runtime = mean(D(i,3));
lc_time = mean(D(i,11));
sample_time = mean(D(i,14));


ygd = [sample_time lc_time total_runtime-lc_time-sample_time];

%%

D = load('profile_BiRRT_PCS_3poles.txt');
D([7 15 49 69 87],:) = [];

i = 1:size(D,1);
total_runtime = mean(D(i,3));
lc_time = mean(D(i,11));
sample_time = mean(D(i,14));

ypcs = [sample_time lc_time total_runtime-lc_time-sample_time];

%%

D = load('profile_BiRRT_RLX_3poles.txt');

i = 1:size(D,1);
total_runtime = mean(D(i,3));
lc_time = mean(D(i,11));
sample_time = mean(D(i,14));

yrlx = [sample_time lc_time total_runtime-lc_time-sample_time];

%%

D = load('profile_BiRRT_SG_3poles.txt');

i = 1:size(D,1);
total_runtime = mean(D(i,3));
lc_time = mean(D(i,11));
sample_time = mean(D(i,14));

yrss = [sample_time lc_time total_runtime-lc_time-sample_time];

%%
figure(1)
clf
subplot(121)
bar([ygd; ypcs],'stacked');
legend('sample','LC','misc');

subplot(122)
bar([yrlx; yrss],'stacked');
legend('sample','LC','misc');
