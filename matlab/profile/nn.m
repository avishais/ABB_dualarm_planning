clear all
clc

D1 = load('profile_nn_RRT_SG_3poles.txt');
D2 = load('profile_nn_RRT2_SG_3poles.txt');
D = [D1; D2];

%%
s = D(:,1);
t = D(:,2)*1e3;

%%
clear T S
maxs = max(s);
S = linspace(0,maxs,300);
T = zeros(1, length(S));
for i = 2:length(S)
    is = s < S(i) & s > S(i-1);
    T(i) = mean(t(is));
end

%%
% p = polyfit(T,S,2);
% Sp = polyval(p, T);

%%
s_cbirrt = 476;
i = knnsearch(S', s_cbirrt);
t_cbirrt = T(i);
s_rrt = 1941;
i = knnsearch(S', s_rrt);
t_rrt = T(i);
s_lazy = 107828;
i = knnsearch(S', s_lazy);
t_lazy = T(i);
s_sbl = 50124;
i = knnsearch(S', s_sbl);
t_sbl = T(i);

%%
h = figure(2);
clf
semilogx(S,T,'-k','linewidth',2.);
hold on
plot(s_cbirrt,t_cbirrt,'or','markerfacecolor','r','markersize',8);
plot(s_rrt,t_rrt,'or','markerfacecolor','r','markersize',8);
plot(s_lazy,t_lazy,'or','markerfacecolor','r','markersize',8);
plot(s_sbl,t_sbl,'or','markerfacecolor','r','markersize',8);
hold off
xlabel('size of tree');
ylabel('nearest-neighbor search time (msec)');
set(gca,'fontsize',13);
xlim([0 max(S)]);
% grid on
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'off';
set(h, 'Position', [100, 100, 800, 400]);
print nn_time.png -dpng -r200