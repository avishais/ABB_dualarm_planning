clear all
clc

% F = {'Benchmark_BiRRT_PCS_3poles_rB.txt','Benchmark_BiRRT_GD_3poles_rB.txt','Benchmark_BiRRT_HB_3poles_rB.txt'};
% op = [1 2.6 1.4];
% F = {'Benchmark_RRT_PCS_3poles_rB.txt','Benchmark_RRT_GD_3poles_rB.txt','Benchmark_RRT_HB_3poles_rB.txt'};
% op = [2.8 2.6 2.2];
F = {'Benchmark_SBL_PCS_3poles_rB.txt','Benchmark_SBL_GD_3poles_rB.txt','Benchmark_SBL_HB_3poles_rB.txt'};
op = [0.6 0.8 1];
color = 'rgb';
linestyle = {'-','--',':'};

figure(1)
clf
% ylim([0 100]);
hold on

for j = 1:3
    
    D = load(F{j});
    
    r = unique(D(:,1));
    
    for i = 1:length(r)
        M = D(D(:,1)==r(i),2:end);
        
        %ls(i) = mean(M(:,14)./M(:,13))*100;%s(i)/l(i)*100;
        %ls(i) = mean(M(:,10));
        ls(i) = mean(M(:,12)./M(:,4)*100);
        
    end
    
    plot(r, ls,'k','linewidth',2,'linestyle',linestyle{j});
    plot(op(j)*[1 1], ylim,'k','linestyle',linestyle{j});
    
end
hold off
legend('PCS','PCS-optimal step','GD','GD-optimal step','HB','HB-optimal step');
xlabel('max. step size');
ylabel('local-connection success rate');
title('Bi-RRT');