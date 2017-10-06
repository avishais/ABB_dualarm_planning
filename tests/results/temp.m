clear all

D1 = load('rss_rbs_verification_withObs_env1_distMix.txt');
D2 = load('rss_rbs_verification_withObs.txt');

D = [D1; D2];

dlmwrite('rss_rbs_verification_withObs.txt',D);