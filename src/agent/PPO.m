% Observation and Action Information
numObs = 16;
obsInfo = rlNumericSpec([numObs 1]);
obsInfo.Name = 'Quad States';
numAct = 4;
actInfo = rlNumericSpec([numAct 1]);
actInfo.Name = 'Quad Action';
actInfo.LowerLimit = [0 0 0 0]';

% Define environment
env = rlFunctionEnv(obsInfo,actInfo,'myStep','myReset');
% Actor network
actor = actorNetwork(obsInfo, actInfo);
% Critic network
critic = criticNetwork(obsInfo);
% Create agent
agentOpts = rlPPOAgentOptions('SampleTime',0.01);
agent = rlPPOAgent(actor,critic,agentOpts);
%%
agent.AgentOptions.ExperienceHorizon = 2^11;
agent.AgentOptions.MiniBatchSize = 2^11;
agent.AgentOptions.UseDeterministicExploitation = false;
agent.AgentOptions.EntropyLossWeight = 0.4;
%agent.AgentOptions.NumEpoch = 10;
trainOpts = rlTrainingOptions(...
    'MaxEpisodes',10000000, ...
    'MaxStepsPerEpisode',10000, ...
    'Verbose',false, ...
    'StopTrainingCriteria',"AverageReward",...
    'StopTrainingValue',10000000,...
    'ScoreAveragingWindowLength',100);
trainOpts.UseParallel = true;
% trainOpts.ParallelizationOptions.StepsUntilDataIsSent = 2^11;
% agent.AgentOptions.ClipFactor = 0.1;
%%
trainingStats = train(agent,env,trainOpts);
%%
agent.AgentOptions.UseDeterministicExploitation = true;
simOptions = rlSimulationOptions('MaxSteps',10000);
experience = sim(env,agent,simOptions);
t = experience.Observation.QuadStates.Time;
xsave = squeeze(experience.Observation.QuadStates.Data(:,1,:));
action = squeeze(experience.Action.QuadAction.Data(:,1,:));
%
global Traj State Action_hist;
tau_vec = Traj.tau_vec;
P = Traj.P;
t_ref = 0.01:0.01:sum(tau_vec)*0.999;
pos_ref = getPos(tau_vec, t_ref', P);
figure(1)
subplot(2,1,1)
plot(t, State(1,:),'Color','red')
grid on
hold on
plot(t, State(2,:),'Color','green')
plot(t, State(3,:),'Color','blue')
plot(t_ref,pos_ref(:,1),'--','Color','red')
plot(t_ref,pos_ref(:,2),'--','Color','green')
plot(t_ref,pos_ref(:,3),'--','Color','blue')
legend('x','y','z')
hold off
subplot(2,1,2)
plot(t, State(4,:),'Color','red')
grid on
hold on
plot(t, State(5,:),'Color','green')
plot(t, State(6,:),'Color','blue')
hold off
figure(2)
subplot(2,1,1)
plot(t, State(7,:))
grid on
hold on
plot(t, State(8,:))
plot(t, State(9,:))
legend('r','p','y')
hold off
subplot(2,1,2)
plot(t, State(10,:))
grid on
hold on
plot(t, State(11,:))
plot(t, State(12,:))
hold off
windowSize = 2;
b = (1/windowSize)*ones(1,windowSize);
a=1;
action_filtered = filter(b,a,action');
action_filtered = action_filtered';
figure(3)
plot(t(10:end), action_filtered(1,9:end))
hold on
plot(t(10:end), action_filtered(2,9:end))
plot(t(10:end), action_filtered(3,9:end))
plot(t(10:end), action_filtered(4,9:end))
grid on
hold off
%%  
global Fext_hist;
size(Fext_hist)
fig = figure(4);
plot3(State(1,:),State(2,:),State(3,:))
grid on
axis equal
hold on
plot3(pos_ref(:,1),pos_ref(:,2),pos_ref(:,3),'--','Color','red')
hold off
%%
filename = '../videos/0529_canopy_1';
target_fps = 50;
video_gen(fig, t, State', filename, target_fps, Fext_hist)