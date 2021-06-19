%%
clf
figure('DefaultAxesFontSize',10)
subplot(2,1,1)
plot(vecnorm(xsave_FM(1:3,:)),'-','linewidth',1.0);grid on;hold on;
plot(vecnorm(xsave_noFM(1:3,:)),'--','linewidth',1.0)
plot(vecnorm(xsave_geo(1:3,:)),':','linewidth',1.0)
ylabel('||e_{pos}||')
legend('Trained w/ disburances','Trained w/o disburances','geometric controller')
subplot(2,1,2)
plot(vecnorm(xsave_FM(4:6,:)),'-','linewidth',1.0);grid on;hold on;
plot(vecnorm(xsave_noFM(4:6,:)),'--','linewidth',1.0)
plot(vecnorm(xsave_geo(4:6,:)),':','linewidth',1.0)
xlabel('steps');ylabel('||e_{vel}||')
%%
set(gcf,'Units','inches');
screenposition = get(gcf,'Position');
set(gcf,...
    'PaperPosition',[0 0 screenposition(3:4)],...
    'PaperSize',[screenposition(3:4)]);
exportgraphics(gcf,'results.pdf')
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
rewards_gauss_FM = [];
steps_gauss_FM = [];
total_steps_gauss_FM = [];

for i=1:2
    load(sprintf('PPO_0531_gaussian_FM_%d',i),'trainingStats')
    training_stats_gauss_FM = trainingStats;
    rewards_gauss_FM= [rewards_gauss_FM, training_stats_gauss_FM.EpisodeReward];
    steps_gauss_FM= [steps_gauss_FM, training_stats_gauss_FM.EpisodeSteps];
    total_steps_gauss_FM= [total_steps_gauss_FM, training_stats_gauss_FM.TotalAgentSteps];
end
rewards_gauss_FM_mva = movmean(rewards_gauss_FM, 300); 
steps_gauss_FM_mva = movmean(steps_gauss_FM, 300);
index_FM = trainingStats.EpisodeIndex;
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
s = shadedErrorBar(index_FM,rewards_gauss_FM_mva',{@median,@std});
set(s.edge,'Visible',false)
s.mainLine.LineWidth = 2;
s.mainLine.Color = "#DD2C00";
s.patch.FaceColor = "#FF9E80";
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
s = shadedErrorBar(index_FM,steps_gauss_FM_mva',{@median,@std});
set(s.edge,'Visible',false)
s.mainLine.LineWidth = 2;
s.mainLine.Color = "#DD2C00";
s.patch.FaceColor = "#FF9E80";
s.patch.FaceAlpha = 0.5;
legend('dual tanh+Gaussian+ET','dual tanh+Beta+ET','single tanh+Beta+ET','dual tanh+FM')
xlabel('episode');ylabel('step')
%%
rewards = [];
steps = [];
total_steps = [];
for i=1:5
    load(sprintf('../results/0601_normal_FM_%d.mat',i),"trainingStats");
    rewards = [rewards, trainingStats.EpisodeReward(1:13000)];
    steps = [steps, trainingStats.EpisodeSteps(1:13000)];
    total_steps = [total_steps, trainingStats.TotalAgentSteps(1:13000)];
end
rewards_noET = [];
steps_noET = [];
total_steps_noET = [];

for i=1:1
    load(sprintf('../results/0602_normal_FM_noET_%d.mat',i),"trainingStats");
    rewards_noET = [rewards_noET, trainingStats.AverageReward];
    steps_noET = [steps_noET, trainingStats.EpisodeSteps];
    total_steps_noET = [total_steps_noET, trainingStats.TotalAgentSteps];
end
rewards_mva = movmean(rewards,200);

clf
shadedErrorBar(mean(total_steps,2),rewards_mva',{@mean,@std})
grid on
hold on
plot(mean(total_steps_noET,2),rewards_noET)
%% EMA
rewards_EMA_07 = [];
total_steps_EMA_07 = [];

rewards_EMA_08 = [];
total_steps_EMA_08 = [];

for i=1:5
    load(sprintf('../results/0601_normal_dual_15N_EMA_0.7_%d.mat',i),"trainingStats");
    rewards_EMA_07 = [rewards_EMA_07, trainingStats.EpisodeReward];
    total_steps_EMA_07 = [total_steps_EMA_07, trainingStats.TotalAgentSteps];
    load(sprintf('../results/0601_normal_dual_15N_EMA_0.8_%d.mat',i),"trainingStats");
    rewards_EMA_08 = [rewards_EMA_08, trainingStats.EpisodeReward];
    total_steps_EMA_08 = [total_steps_EMA_08, trainingStats.TotalAgentSteps];
end

rewards_EMA_07_mva = movmean(rewards_EMA_07, 100);
rewards_EMA_08_mva = movmean(rewards_EMA_08, 100);

clf
s=shadedErrorBar([1:length(rewards_EMA_07_mva)],rewards_EMA_07_mva',{@mean,@std});
set(s.edge,'Visible',false)
s.mainLine.LineWidth = 2;
s.mainLine.Color = "#0091EA";
s.patch.FaceColor = "#80D8FF";
s.patch.FaceAlpha = 0.5;
grid on
s=shadedErrorBar([1:length(rewards_EMA_08_mva)],rewards_EMA_08_mva',{@mean,@std});
set(s.edge,'Visible',false)
s.mainLine.LineWidth = 2;
s.mainLine.Color = "#00C853";
s.patch.FaceColor = "#B9F6CA";
s.patch.FaceAlpha = 0.5;
legend('07','08')
%% Final
rewards=cell(1,5);total_steps=cell(1,5);
rewards_noET=cell(1,5);total_steps_noET=cell(1,5);
rewards_single=cell(1,2);total_steps_single=cell(1,2);
for i=1:5
    load(sprintf('../results/0601_normal_FM_%d.mat',i),"trainingStats");
    rewards{i} = trainingStats.EpisodeReward;
    total_steps{i} = trainingStats.TotalAgentSteps;
    load(sprintf('../results/0602_normal_FM_noET_%d.mat',i),"trainingStats");
    rewards_noET{i} = trainingStats.EpisodeReward;
    total_steps_noET{i} = trainingStats.TotalAgentSteps;
end
for i=1:5
    rewards{i} = rewards{i}(1:min(cellfun('length',rewards)));
    total_steps{i} = total_steps{i}(1:min(cellfun('length',total_steps)));
    rewards_noET{i} = rewards_noET{i}(1:min(cellfun('length',rewards_noET)));
    total_steps_noET{i} = total_steps_noET{i}(1:min(cellfun('length',total_steps_noET)));
end
for i=1:2
    load(sprintf('../results/0601_normal_FM_single_%d.mat',i),"trainingStats");
    rewards_single{i} = trainingStats.EpisodeReward;
    total_steps_single{i} = trainingStats.TotalAgentSteps;
end
for i=1:2
    rewards_single{i} = rewards_single{i}(1:min(cellfun('length',rewards_single)));
    total_steps_single{i} = total_steps_single{i}(1:min(cellfun('length',total_steps_single)));
end
rewards = cell2mat(rewards);
total_steps = cell2mat(total_steps);
rewards_noET = cell2mat(rewards_noET);
total_steps_noET = cell2mat(total_steps_noET);
rewards_single = cell2mat(rewards_single);
total_steps_single = cell2mat(total_steps_single);

rewards = movmean(rewards,500);
rewards_noET = movmean(rewards_noET,500);
rewards_single = movmean(rewards_single,500);
steps = mean(total_steps,2);
steps_noET = mean(total_steps_noET,2);
steps_single = mean(total_steps_single,2);

%% PLOT
clf
s = shadedErrorBar(steps,rewards',{@mean,@std});
set(s.edge,'Visible',false)
set(gca,'FontSize',10)
s.mainLine.LineWidth = 3;
s.mainLine.Color = "#0091EA";
s.patch.FaceColor = "#80D8FF";
s.patch.FaceAlpha = 0.5;
grid on
s = shadedErrorBar(steps_noET,rewards_noET',{@mean,@std});
set(s.edge,'Visible',false)
s.mainLine.LineWidth = 3;
s.mainLine.Color = "#00C853";
s.patch.FaceColor = "#B9F6CA";
s.patch.FaceAlpha = 0.5;
s = shadedErrorBar(steps_single,rewards_single',{@mean,@std});
set(s.edge,'Visible',false)
s.mainLine.LineWidth = 3;
s.mainLine.Color = "#6200EA";
s.patch.FaceColor = "#B388FF";
s.patch.FaceAlpha = 0.5;
legend('double tanh+ET','double tanh','single tanh','Location','Northwest')
xlabel('average steps');ylabel('average return')