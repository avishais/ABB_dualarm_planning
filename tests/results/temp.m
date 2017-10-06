clear all

D1 = load('rss_rbs_verification_withObs_env2_distMix.txt');
D2 = load('rss_rbs_verification_withObs_env2.txt');

D = [D1; D2];

dlmwrite('rss_rbs_verification_withObs_env2.txt',D);