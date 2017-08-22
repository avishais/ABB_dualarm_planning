% Visibility plots based on the tests in: test_pcs_rbs.m, test_gd_rbs.m
% last updated: 08/02/17

clear all
clc

D1 = load('pcs_rbs_verification.txt');
D2 = load('gd_rbs_verification.txt');

%% Visibility

clear V1 d1
s1 = D1(:,3);
max_d = max(s1);
d1 = linspace(0, max_d, 30);
for i = 2:length(d1)
    S = D1(D1(:,3)>=d1(i-1) & D1(:,3)<d1(i), 1);
    V1(i-1) = sum(S)/length(S) * 100;
end

clear V2 d2
s2 = D2(:,3);
max_d = max(s2);
d2 = linspace(0, max_d, 20);
for i = 2:length(d2)
    S = D2(D2(:,3)>=d2(i-1) & D2(:,3)<d2(i), 1);
    V2(i-1) = sum(S)/length(S) * 100;
end


figure(4)
plot(d1(2:end), V1,'-k','linewidth',2.5);
hold on
plot(d2(2:end), V2,'--k','linewidth',2.5);
hold off
xlabel('distance');
ylabel('success rate / visibility [%]');
legend('PCS','GD');
