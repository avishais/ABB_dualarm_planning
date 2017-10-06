clear all
clc
%% GD
Do{1} = load('gd_rbs_verification_withObs.txt');

%% PCS

Do{2} = load('pcs_rbs_verification_withObs.txt');
%% RLX

Do{3} = load('rlx_rbs_verification_eps0.5_withObs_env1.txt');

%% RSS

Do{4} = load('rss_rbs_verification_withObs.txt');

%%
figure(2)
clf

for i = 1:size(Do,2)
    D = Do{i};
    
    clear V d
    Dd = D(:,3);
    max_d = max(Dd);
    d = linspace(0, max_d, 20)';
    c = zeros(length(d),1);
    for i = 2:length(d)
        S = D(D(:,3)>=d(i-1) & D(:,3)<d(i), 1);
        c(i) = length(S);
        %V(i-1) = (1-sum(S)/length(S)) * 100;
    end
    
    %%
    ic = knnsearch(d, 7);   
%     disp(min(c(2:ic)));
    l = 1000;%min(c)
    ic = max(find(c>l));
    
    disp(min(c(2:ic)));
    
    
    
    figure(1)
    bar(c(2:ic));
    %%
    Sv = [];
    for i = 2:ic
        S = D(D(:,3)>=d(i-1) & D(:,3)<d(i), 1);
        S = S(1:l);
        V(i-1) = (1-sum(S)/length(S)) * 100;
        Sv = [Sv; S];
    end
    figure(2)
    hold on
    plot(d(2:ic),100-V);
    hold off
    
    disp(['Success rate/visibility for the RBS: ' num2str(sum(Sv)/length(Sv)*100) '%']);
    
end

figure(2)
legend('NR','PCS','RLX','RSS');