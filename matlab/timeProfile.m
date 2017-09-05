clear all
clc

%% BiRRT - GD

TP = load('../paths/profiles/timeProfileBiRRT_GD.txt');
total_time = TP(1);
lc_time = TP(2);
collisionTime = TP(3);
kdlTime = TP(4);
growTree_mode1 = TP(5); %t[0]
growTree_mode2 = TP(6); %t[1]
nn_search = TP(7); %t[2]
IKproject = TP(10); %t[5] % includes collisions, only for random point projections

G = [lc_time; IKproject];
G = [G; total_time-sum(G)];
G = G/sum(G);

P = [nn_search; kdlTime; collisionTime];
P = [P; total_time-sum(P)];
P = P/sum(P);

figure(1)
subplot(121)
pie(G);
legend('local-con.','rand. proj.','misc');
subplot(122)
pie(P);
legend('NN-search','KDL','collision','misc');

%% SBL - GD

%TP = load('../paths/profiles/timeProfileSBL_GD.txt');
TP = load('../paths/timeProfile.txt');
total_time = TP(1);
lc_time = TP(2);
collisionTime = TP(3);
kdlTime = TP(4);
IKproject = TP(5); %t[0] % includes collisions, only for random point projections
removeMotion = TP(6); %t[1]
sample = TP(7); %t[2]

G = [lc_time; IKproject; removeMotion];
G = [G; total_time-sum(G)];
G = G/sum(G);

P = [sample; kdlTime; collisionTime; removeMotion];
P = [P; total_time-sum(P)];
P = P/sum(P);

figure(1)
subplot(121)
pie(G);
legend('local-con.','rand. proj.','remove conn.','misc');
subplot(122)
pie(P);
legend('sampling','KDL','collision','remove conn.','misc');

projSuccess = TP(8);
projFailed = TP(9);