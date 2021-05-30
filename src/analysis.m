%%
reward_gaussian = [];
step_gaussian = [];
reward_beta = [];
step_beta = [];
for i=1:10
    load(sprintf('PPO_0530_gaussian_3500_%d',i))
    training_stats_gaussian = trainingStats;
    reward_gaussian = [reward_gaussian, training_stats_gaussian.EpisodeReward(1:3500)];
    step_gaussian = [step_gaussian, training_stats_gaussian.EpisodeSteps(1:3500)];
end
for i=1:4
    load(sprintf('PPO_0530_beta_3500_%d',i))
    training_stats_beta = trainingStats;
    reward_beta = [reward_beta, training_stats_beta.AverageReward(1:3500)];
    step_beta = [step_beta, training_stats_beta.AverageSteps(1:3500)];
end
%%
index = trainingStats.EpisodeIndex;
%%
subplot(1,2,1)
shadedErrorBar(index,reward_gaussian',{@median,@std},'lineProps',{'b-','markerfacecolor','b'});
shadedErrorBar(index,reward_beta',{@median,@std},'lineProps',{'r-','markerfacecolor','r'});
subplot(1,2,2)
shadedErrorBar(index, step_gaussian',{@median,@std},'lineProps',{'b-','markerfacecolor','b'});
shadedErrorBar(index, step_beta',{@median,@std},'lineProps',{'r-','markerfacecolor','r'});
% plot_areaerrorbar(reward_gaussian');
%%
subplot(1,2,1)
plot(index_gaussian_1,reward_gaussian_1)
grid on; hold on;
plot(index_beta_1, reward_beta_1)

legend('Gaussian','Beta')
hold off
subplot(1,2,2)
plot(index_gaussian_1,step_gaussian_1)
grid on; hold on;
plot(index_beta_1,step_beta_1)
legend('Gaussian','Beta')
hold off