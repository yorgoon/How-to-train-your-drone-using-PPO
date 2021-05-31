%%
Reward_gaussian = [];
Step_gaussian = [];
total_step_gaussian = [];
Reward_beta = [];
Step_beta = [];
total_step_beta = [];
Reward_singletanh = [];
Step_singletanh = [];
total_step_singletanh = [];
for i=1:10
    load(sprintf('PPO_0530_gaussian_3500_%d',i),'trainingStats')
    training_stats_gaussian = trainingStats;
    Reward_gaussian = [Reward_gaussian, training_stats_gaussian.EpisodeReward(1:3500)];
    Step_gaussian = [Step_gaussian, training_stats_gaussian.EpisodeSteps(1:3500)];
    total_step_gaussian = [total_step_gaussian, training_stats_gaussian.TotalAgentSteps(1:3500)];
end
%%
for i=1:10
    load(sprintf('PPO_0530_beta_3500_%d',i),'trainingStats')
    training_stats_beta = trainingStats;
    Reward_beta = [Reward_beta, training_stats_beta.EpisodeReward(1:3500)];
    Step_beta = [Step_beta, training_stats_beta.EpisodeSteps(1:3500)];
    total_step_beta = [total_step_beta, training_stats_beta.TotalAgentSteps(1:3500)];
end
%%
for i=1:4
    load(sprintf('PPO_0531_singletanh_3500_%d',i),'trainingStats')
    training_stats_singletanh = trainingStats;
    Reward_singletanh = [Reward_singletanh, training_stats_singletanh.EpisodeReward(1:3500)];
    Step_singletanh = [Step_singletanh, training_stats_singletanh.EpisodeSteps(1:3500)];
    total_step_singletanh = [total_step_singletanh , training_stats_singletanh.TotalAgentSteps(1:3500)];
end

%%
index = trainingStats.EpisodeIndex;
reward_gaussian_mva = movmean(Reward_gaussian, 300);
step_gaussian_mva = movmean(Step_gaussian, 300);
reward_beta_mva = movmean(Reward_beta, 300);
step_beta_mva = movmean(Step_beta, 300);
reward_singletanh_mva = movmean(Reward_singletanh, 300);
step_singletanh_mva = movmean(Step_singletanh, 300);

%%
clf
subplot(1,2,1)
s = shadedErrorBar(index,reward_gaussian_mva',{@median,@std});
set(s.edge,'Visible',false)
s.mainLine.LineWidth = 2;
s.mainLine.Color = "#0091EA";
s.patch.FaceColor = "#80D8FF";
s.patch.FaceAlpha = 0.5;
grid on
s = shadedErrorBar(index,reward_beta_mva',{@median,@std});
s.mainLine.LineWidth = 2;
s.mainLine.Color = "#00C853";
s.patch.FaceColor = "#B9F6CA";
s.patch.FaceAlpha = 0.5;
set(s.edge,'Visible',false)
s = shadedErrorBar(index,reward_singletanh_mva',{@median,@std});
set(s.edge,'Visible',false)
s.mainLine.LineWidth = 2;
s.mainLine.Color = "#6200EA";
s.patch.FaceColor = "#B388FF";
s.patch.FaceAlpha = 0.5;
xlabel('episode');ylabel('reward/step')
%
subplot(1,2,2)
s = shadedErrorBar(index, step_gaussian_mva',{@median,@std});
set(s.edge,'Visible',false)
s.mainLine.LineWidth = 2;
s.mainLine.Color = "#0091EA";
s.patch.FaceColor = "#80D8FF";
s.patch.FaceAlpha = 0.5;
grid on
s = shadedErrorBar(index, step_beta_mva',{@median,@std});
s.mainLine.LineWidth = 2;
s.mainLine.Color = "#00C853";
s.patch.FaceColor = "#B9F6CA";
s.patch.FaceAlpha = 0.5;
set(s.edge,'Visible',false)
s = shadedErrorBar(index,step_singletanh_mva',{@median,@std});
set(s.edge,'Visible',false)
s.mainLine.LineWidth = 2;
s.mainLine.Color = "#6200EA";
s.patch.FaceColor = "#B388FF";
s.patch.FaceAlpha = 0.5;
legend('dual tanh+Gaussian+ET','dual tanh+Beta+ET','single tanh+Beta+ET')
xlabel('episode');ylabel('step')
%%
plot(total_step_gaussian(:,1),reward_gaussian_mva(:,1))