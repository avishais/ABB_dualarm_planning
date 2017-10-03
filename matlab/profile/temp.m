clear all
clc

%% GD

D1 = load('profile_BiRRT_GD_3poles.txt');
D2 = load('profile_SBL_GD_3poles.txt');

t1 = mean(D1(:,3));
sample_success_count = round(mean(D1(:,15)));
sample_fail_count = round(mean(D1(:,16)));
n1 = sample_success_count+sample_fail_count;

t2 = mean(D2(:,3));
sample_success_count = round(mean(D2(:,15)));
sample_fail_count = round(mean(D2(:,16)));
n2 = sample_success_count+sample_fail_count;

disp([n1 / t1 n2 / t2]);
r(1) = (n2 / t2) / (n1 / t1)

%% PCS

D1 = load('profile_BiRRT_PCS_3poles.txt');
D2 = load('profile_SBL_PCS_3poles.txt');

t1 = mean(D1(:,3));
sample_success_count = round(mean(D1(:,15)));
sample_fail_count = round(mean(D1(:,16)));
n1 = sample_success_count+sample_fail_count;

t2 = mean(D2(:,3));
sample_success_count = round(mean(D2(:,15)));
sample_fail_count = round(mean(D2(:,16)));
n2 = sample_success_count+sample_fail_count;

disp([n1 / t1 n2 / t2]);
r(2) = (n2 / t2) / (n1 / t1)

%% RLX

D1 = load('profile_BiRRT_RLX_3poles.txt');
D2 = load('profile_SBL_RLX_3poles.txt');

t1 = mean(D1(:,3));
sample_success_count = round(mean(D1(:,15)));
sample_fail_count = round(mean(D1(:,16)));
n1 = sample_success_count+sample_fail_count;

t2 = mean(D2(:,3));
sample_success_count = round(mean(D2(:,15)));
sample_fail_count = round(mean(D2(:,16)));
n2 = sample_success_count+sample_fail_count;

disp([n1 / t1 n2 / t2]);
r(3) = (n2 / t2) / (n1 / t1)

%% RSS

D1 = load('profile_BiRRT_SG_3poles.txt');
D2 = load('profile_SBL_SG_3poles.txt');

t1 = mean(D1(:,3));
sample_success_count = round(mean(D1(:,15)));
sample_fail_count = round(mean(D1(:,16)));
n1 = sample_success_count+sample_fail_count;

t2 = mean(D2(:,3));
sample_success_count = round(mean(D2(:,15)));
sample_fail_count = round(mean(D2(:,16)));
n2 = sample_success_count+sample_fail_count;
    
disp([n1 / t1 n2 / t2]);
r(4) = (n2 / t2) / (n1 / t1)
